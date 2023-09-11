# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import atexit
import collections
import concurrent.futures
import dataclasses
import logging
import os
import queue
import threading
import typing
import weakref

import rclpy.executors


class AutoScalingThreadPool(concurrent.futures.Executor):
    """
    A concurrent.futures.Executor subclass based on a thread pool.

    Akin to the concurrent.futures.ThreadPoolExecutor class but with
    autoscaling capabilities.
    """

    _lock: threading.Lock = threading.Lock()
    _interpreter_shutdown: bool = False
    _all_runqueues: weakref.WeakSet = weakref.WeakSet()
    _all_workers: weakref.WeakSet = weakref.WeakSet()

    @classmethod
    def _on_interpreter_shutdown(cls) -> None:
        # Gracefully shutdown daemonized threads
        # upon interpreter shutdown.
        with cls._lock:
            cls._interpreter_shutdown = True
            for queue in cls._all_runqueues:
                queue.put(None)
            cls._all_runqueues.clear()
            for worker in cls._all_workers:
                if worker.is_alive():
                    worker.join()
            cls._all_workers.clear()

    @dataclasses.dataclass
    class Work:
        future: concurrent.futures.Future
        fn: typing.Callable[..., typing.Any]
        args: typing.Tuple[typing.Any, ...]
        kwargs: typing.Dict[str, typing.Any]

        def execute(self) -> None:
            try:
                self.future.set_result(
                    self.fn(*self.args, **self.kwargs))
            except Exception as e:
                self.future.set_exception(e)

        def cancel(self) -> None:
            self.future.cancel()

        def cancelled(self) -> bool:
            return self.future.cancelled()

        def when_done(
            self,
            callback: typing.Callable[['AutoScalingThreadPool.Work'], None]
        ) -> None:
            self.future.add_done_callback(lambda f: callback(self))

    def __init__(
        self, *,
        min_workers: typing.Optional[int] = None,
        max_workers: typing.Optional[int] = None,
        submission_quota: typing.Optional[int] = None,
        submission_patience: typing.Optional[float] = None,
        max_idle_time: typing.Optional[float] = None
    ):
        """
        Initializes the thread pool.

        :param min_workers: minimum number of workers in the pool, 0 by default.
        :param max_workers: maximum number of workers in the pool, 32 times the
          number of available CPU threads by default (assuming I/O bound work).
        :param submission_quota: maximum number of concurrent submissions for a
          a given callable. Up to the maximum number of workers by default. Useful
          when serving multiple users to prevent anyone from starving the rest.
        :param submission_patience: time to wait in seconds for a worker to become
          available before upscaling the pool. 100 ms by default.
        :param max_idle_time: time in seconds for a worker to wait for work before
          shutting itself down, effectively downscaling the pool. 60 seconds by default.
        """

        if min_workers is None:
            min_workers = 0
        if min_workers < 0:
            raise ValueError(
                'Minimum number of workers '
                'must be a nonnegative number')
        self._min_workers = min_workers

        if max_workers is None:
            max_workers = 32 * (os.cpu_count() or 1)
        if max_workers < min_workers:
            raise ValueError(
                'Maximum number of workers must be larger than'
                ' or equal to the minimum number of workers')
        self._max_workers = max_workers

        if max_idle_time is None:
            max_idle_time = 60.0
        if max_idle_time <= 0:
            raise ValueError(
                'Maximum idle time for workers'
                ' must be a positive number')
        self._max_idle_time = max_idle_time

        if submission_quota is None:
            submission_quota = max_workers
        if submission_quota <= 0:
            raise ValueError(
                'Quota for submission '
                'must be a positive number')
        self._submission_quota = submission_quota

        if submission_patience is None:
            submission_patience = 0.1
        if submission_patience < 0:
            raise ValueError(
                'Patience for submission '
                'must be a nonnegative number')
        self._submission_patience = submission_patience

        self._shutdown = False
        self._submit_lock = threading.Lock()
        self._shutdown_lock = threading.Lock()

        with AutoScalingThreadPool._lock:
            if AutoScalingThreadPool._interpreter_shutdown:
                raise RuntimeError(
                    'cannot create thread pool while'
                    ' interpreter is shutting down')
            # register runqueue for external wake up  on interpreter shutdown
            runqueue: queue.SimpleQueue = queue.SimpleQueue()
            AutoScalingThreadPool._all_runqueues.add(runqueue)
            self._runqueue = runqueue
            weak_self = weakref.ref(
                self, lambda ref: runqueue.put(None))
            self._workers: weakref.WeakSet = weakref.WeakSet()
            for _ in range(self._min_workers):  # fire up stable worker pool
                # careful with back references to self!
                worker = threading.Thread(
                    target=AutoScalingThreadPool._do_work,
                    args=(weak_self, False), daemon=True)
                worker.start()
                self._workers.add(worker)
            # register workers for external joining on interpreter shutdown
            AutoScalingThreadPool._all_workers.update(self._workers)

        self._waitqueues: typing.Dict[
            typing.Callable[..., typing.Any],
            collections.deque
        ] = collections.defaultdict(collections.deque)
        self._runlists: typing.Dict[
            typing.Callable[..., typing.Any], list
        ] = collections.defaultdict(list)
        self._runslots = threading.Semaphore(0)

    @property
    def workers(self) -> typing.List[threading.Thread]:
        with self._submit_lock:
            # avoid races and proactively drop shutdown workers
            self._workers = weakref.WeakSet([
                w for w in self._workers if w.is_alive()])
        return list(self._workers)

    def _do_cleanup_after(self, work: 'AutoScalingThreadPool.Work') -> None:
        with self._submit_lock:
            self._runlists[work.fn].remove(work)
            if self._waitqueues[work.fn]:  # continue with pending work
                work = self._waitqueues[work.fn].popleft()
                while work.cancelled() and self._waitqueues[work.fn]:
                    work = self._waitqueues[work.fn].popleft()
                if not work.cancelled():
                    self._do_submit(work)  # actually submit work
                    self._runlists[work.fn].append(work)
                    work.when_done(self._do_cleanup_after)
            if not self._runlists[work.fn]:
                del self._runlists[work.fn]
            if not self._waitqueues[work.fn]:
                del self._waitqueues[work.fn]

    @classmethod
    def _do_work(
        cls,
        executor_weakref: weakref.ref,
        stop_on_timeout: bool = True
    ) -> None:
        logger = logging.getLogger(__name__)
        try:
            while not cls._interpreter_shutdown:
                executor: typing.Optional[
                    AutoScalingThreadPool
                ] = executor_weakref()
                if executor is None:
                    break

                runqueue, runslots, timeout = (
                    executor._runqueue,
                    executor._runslots,
                    executor._max_idle_time)

                if executor._shutdown:
                    runqueue.put(None)
                    break

                del executor  # drop reference

                runslots.release()
                try:
                    work = runqueue.get(
                        block=True,
                        timeout=timeout)
                except queue.Empty:
                    if stop_on_timeout:  # non-stable worker
                        if runslots.acquire(blocking=False):
                            break
                    continue

                if work is not None:
                    work.execute()
                del work  # drop reference
        except BaseException as e:
            logger.error(f'Worker threw an exception: {e}')

    def _do_submit(self, work: 'AutoScalingThreadPool.Work') -> None:
        self._workers = weakref.WeakSet([w for w in self._workers if w.is_alive()])
        while not self._runslots.acquire(timeout=self._submission_patience):
            if len(self._workers) >= self._max_workers:
                break
            runqueue = self._runqueue
            weak_self = weakref.ref(
                self, lambda ref: runqueue.put(None))
            # careful with back references to self!
            worker = threading.Thread(
                target=AutoScalingThreadPool._do_work,
                args=(weak_self,), daemon=True)
            worker.start()
            with AutoScalingThreadPool._lock:
                AutoScalingThreadPool._all_workers.add(worker)
            self._workers.add(worker)
        self._runqueue.put(work)

    def submit(self, fn, /, *args, **kwargs) -> concurrent.futures.Future:
        with self._submit_lock, self._shutdown_lock:
            if self._shutdown:
                raise RuntimeError('cannot submit to a shutdown pool')
            future: concurrent.futures.Future = concurrent.futures.Future()
            WorkT = AutoScalingThreadPool.Work
            work = WorkT(future, fn, args, kwargs)
            if self._submission_quota > len(self._runlists[work.fn]):
                if self._waitqueues[work.fn]:  # prioritize pending work
                    self._waitqueues[work.fn].append(work)
                    work = self._waitqueues[work.fn].popleft()
                    while work.cancelled() and self._waitqueues[work.fn]:
                        work = self._waitqueues[work.fn].popleft()
                    if not self._waitqueues[work.fn]:
                        del self._waitqueues[work.fn]
                if not work.cancelled():
                    self._do_submit(work)  # actually submit work
                    self._runlists[work.fn].append(work)
                    work.when_done(self._do_cleanup_after)
            else:
                self._waitqueues[work.fn].append(work)
            return future

    def shutdown(
        self,
        wait: bool = True, *,
        cancel_futures: bool = False
    ) -> None:
        with self._shutdown_lock:
            self._shutdown = True

        if cancel_futures:
            with self._submit_lock:
                for runlist in self._runlists.values():
                    for work in runlist:
                        work.cancel()
                for waitqueue in self._waitqueues.values():
                    for work in waitqueue:
                        work.cancel()

        self._runqueue.put(None)
        if wait:
            for worker in self._workers:
                worker.join()
            self._workers.clear()


atexit.register(AutoScalingThreadPool._on_interpreter_shutdown)


class AutoScalingMultiThreadedExecutor(rclpy.executors.Executor):
    """
    An rclpy.executors.Executor subclass based on an AutoScalingThreadPool.

    Akin to the rclpy.executors.MultiThreadedExecutor class but with autoscaling capabilities.
    Moreover, a concurrency quota can be defined on a per callback + callback group basis to
    avoid thread pool starvation and/or exhaustion of system resources (e.g. when using reentrant
    callback groups).
    """

    class TaskInCallbackGroup:
        """A bundle of executable task and associated callback group."""

        def __init__(
            self,
            task: rclpy.task.Task,
            callback_group: rclpy.callback_groups.CallbackGroup
        ):
            self.task = task
            self.callback_group = callback_group

        def __call__(self) -> None:
            self.task.__call__()

        def __getattr__(self, name: str) -> typing.Any:
            return getattr(self.task, name)

        def __hash__(self) -> int:
            # ignore task arguments, which typically vary from
            # execution to execution (e.g. different messages for
            # subscription callbacks)
            return hash((self.task._handler, self.callback_group))

    def __init__(
        self,
        max_threads: typing.Optional[int] = None,
        max_thread_idle_time: typing.Optional[float] = None,
        max_threads_per_callback_group: typing.Optional[int] = None, *,
        context: typing.Optional[rclpy.context.Context] = None
    ) -> None:
        """
        Initializes the executor.

        :param max_threads: maximum number of threads to spin at any given time.
          See AutoScalingThreadPool documentation for reference on defaults.
        :param max_thread_idle_time: time in seconds for a thread to wait for work
          before shutting itself down. See AutoScalingThreadPool documentation for
          reference on defaults.
        :param max_threads_per_callback_group: optional maximum number of concurrent
          callbacks to service for a given callback group. Useful to avoid reentrant
          callback groups from starving the pool.
        """
        super().__init__(context=context)
        self._thread_pool = AutoScalingThreadPool(
            max_workers=max_threads,
            max_idle_time=max_thread_idle_time,
            submission_quota=max_threads_per_callback_group)
        self._futures: typing.List[
            AutoScalingMultiThreadedExecutor.TaskInCallbackGroup
        ] = []

    @property
    def thread_pool(self) -> AutoScalingThreadPool:
        return self._thread_pool

    def _do_spin_once(self, *args, **kwargs) -> None:
        TaskInCallbackGroupT = \
            AutoScalingMultiThreadedExecutor.TaskInCallbackGroup
        try:
            task, entity, node = \
                self.wait_for_ready_callbacks(*args, **kwargs)
        except rclpy.executors.ExternalShutdownException:
            pass
        except rclpy.executors.ShutdownException:
            pass
        except rclpy.executors.TimeoutException:
            pass
        except rclpy.executors.ConditionReachedException:
            pass
        else:
            callback_group = None
            if entity is not None:
                callback_group = entity.callback_group
            task = TaskInCallbackGroupT(task, callback_group)
            self._thread_pool.submit(task)
            self._futures.append(task)

            for task in self._futures[:]:
                if task.done():
                    self._futures.remove(task)
                    task.result()

    def spin_once(self, timeout_sec: typing.Optional[float] = None) -> None:
        self._do_spin_once(timeout_sec)

    def spin_once_until_future_complete(
        self,
        future: rclpy.task.Future,
        timeout_sec: typing.Optional[float] = None
    ) -> None:
        future.add_done_callback(lambda f: self.wake())
        self._do_spin_once(timeout_sec, condition=future.done)

    def shutdown(self, timeout_sec: typing.Optional[float] = None) -> bool:
        done = super().shutdown(timeout_sec)
        if done:
            self._thread_pool.shutdown()
        return done
