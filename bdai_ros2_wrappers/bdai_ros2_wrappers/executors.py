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
        max_idle_time: typing.Optional[float] = None
    ):
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
                'Quotas for submission '
                'must be a positive number')
        self._submission_quota = submission_quota

        self._submit_lock = threading.Lock()
        self._shutdown_lock = threading.Lock()

        runqueue: queue.SimpleQueue = queue.SimpleQueue()
        self._runqueue = runqueue
        self._weakref = weakref.ref(
            self, lambda ref: runqueue.put(None))
        self._workers: typing.List[threading.Thread] = [
            threading.Thread(
                target=self._do_work,
                args=(self._weakref, False),
                daemon=True
            ) for _ in range(self._min_workers)
        ]
        self._waitqueues: typing.Dict[
            typing.Callable[..., typing.Any],
            collections.deque
        ] = collections.defaultdict(collections.deque)
        self._runlists: typing.Dict[
            typing.Callable[..., typing.Any], set
        ] = collections.defaultdict(set)
        self._runslots = threading.Semaphore(0)

    def _do_cleanup_after(self, work: 'AutoScalingThreadPool.Work') -> None:
        with self._submit_lock:
            self._runlists[work.fn].remove(work)
            if self._waitqueues[work.fn]:
                work = self._waitqueues[work.fn].popleft()
                while work.cancelled() and self._waitqueues[work.fn]:
                    work = self._waitqueues[work.fn].popleft()
                if not work.cancelled():
                    self._do_submit(work)  # actually submit work
                    self._runlists[work.fn].add(work)
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
            while True:
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

                del executor

                runslots.release()
                try:
                    work = runqueue.get(
                        block=True,
                        timeout=timeout)
                except queue.Empty:
                    if stop_on_timeout:
                        break
                    continue

                if work is not None:
                    work.execute()
                    del work
        except BaseException as e:
            logger.error(f'Worker threw an exception: {e}')

    def _do_submit(self, work: 'AutoScalingThreadPool.Work') -> None:
        if not self._runslots.acquire(blocking=False):
            self._workers = [
                w for w in self._workers if w.is_alive()]
            if self._max_workers > len(self._workers):
                worker = threading.Thread(
                    target=self._do_work,
                    args=(self._weakref,),
                    daemon=True)
                self._workers.append(worker)
        self._runqueue.put(work)

    def submit(self, fn, /, *args, **kwargs) -> concurrent.futures.Future:
        with self._submit_lock, self._shutdown_lock:
            if self._shutdown:
                raise RuntimeError()
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
                    self._runlists[work.fn].add(work)
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


class AutoScalingMultiThreadedExecutor(rclpy.executors.Executor):

    class TaskInCallbackGroup:

        def __init__(
            self,
            task: rclpy.task.Task,
            callback_group: rclpy.callback_groups.CallbackGroup
        ):
            self.task = task
            self.callback_group = callback_group

        def __getattr__(self, name: str) -> typing.Any:
            return getattr(self.task, name)

        def __hash__(self) -> int:
            return hash((self.task, self.callback_group))

    def __init__(
        self,
        max_threads: typing.Optional[int] = None,
        max_thread_idle_time: typing.Optional[float] = None,
        max_threads_per_callback_group: typing.Optional[int] = None, *,
        context: typing.Optional[rclpy.context.Context] = None
    ) -> None:
        super().__init__(context=context)
        self._executor = AutoScalingThreadPool(
            max_workers=max_threads,
            max_idle_time=max_thread_idle_time,
            submission_quota=max_threads_per_callback_group)
        self._tasks: typing.List[
            AutoScalingMultiThreadedExecutor.TaskInCallbackGroup
        ] = []

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
            task = TaskInCallbackGroupT(
                task, entity.callback_group)
            self._executor.submit(task)
            self._tasks.append(task)

            for task in self._tasks[:]:
                if task.done():
                    self._tasks.remove(task)
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
