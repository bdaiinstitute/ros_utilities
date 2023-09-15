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


def classname(obj: typing.Any) -> str:
    """Computes the fully qualified class name for a given object."""
    cls = obj.__class__
    return f"{cls.__module__}.{cls.__qualname__}"


class AutoScalingThreadPool(concurrent.futures.Executor):
    """
    A concurrent.futures.Executor subclass based on a thread pool.

    Akin to the concurrent.futures.ThreadPoolExecutor class but with
    autoscaling capabilities. Within a given range, the number of
    workers increases with demand (ie. submissions) and decreases
    with time (ie. when idle for long enough).

    See concurrent.futures.Executor documentation for further reference.
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
            for q in cls._all_runqueues:
                q.put(None)
            cls._all_runqueues.clear()
            for worker in cls._all_workers:
                if worker.is_alive():
                    worker.join()
            cls._all_workers.clear()

    @dataclasses.dataclass
    class Work:
        """A work submission to process and execute."""

        future: concurrent.futures.Future
        fn: typing.Callable[..., typing.Any]
        args: typing.Tuple[typing.Any, ...]
        kwargs: typing.Dict[str, typing.Any]

        def execute(self) -> None:
            """Executes work and resolves its future."""
            if not self.future.set_running_or_notify_cancel():
                return
            try:
                self.future.set_result(self.fn(*self.args, **self.kwargs))
            except BaseException as e:
                self.future.set_exception(e)

        def cancel(self) -> bool:
            """Cancels work."""
            return self.future.cancel()

        def pending(self) -> bool:
            """Checks if work is pending."""
            return not self.future.done()

        def cancelled(self) -> bool:
            """Checks if work has been cancelled."""
            return self.future.cancelled()

        def notify_cancelation(self) -> None:
            """Notifies those waiting on the future about work being cancelled."""
            assert self.future.cancelled()
            self.future.set_running_or_notify_cancel()

    class Worker(threading.Thread):
        def __init__(self, executor_weakref: weakref.ref, stop_on_timeout: bool = True) -> None:
            super().__init__(daemon=True)
            self._executor_weakref = executor_weakref
            executor = executor_weakref()
            assert executor is not None
            self._runqueue = executor._runqueue
            if stop_on_timeout:
                self._timeout = executor._max_idle_time
            else:
                self._timeout = None
            self._logger = executor._logger
            self.start()

        def run(self) -> None:
            try:
                work: typing.Optional[AutoScalingThreadPool.Work] = None
                while True:
                    if AutoScalingThreadPool._interpreter_shutdown:
                        self._runqueue.put(None)
                        break

                    executor: typing.Optional[AutoScalingThreadPool] = self._executor_weakref()
                    if executor is None or executor._shutdown:
                        self._runqueue.put(None)
                        break

                    runslots = executor._runslots
                    if work is not None:
                        if executor._cleanup_after(work):
                            runslots.release()
                        work = None
                    else:
                        runslots.release()
                    del executor  # drop reference

                    try:
                        work = self._runqueue.get(block=True, timeout=self._timeout)
                    except queue.Empty:
                        if runslots.acquire(blocking=False):
                            break
                        continue

                    if work is not None:
                        work.execute()
            except BaseException:
                import textwrap
                import traceback

                trace = textwrap.indent(traceback.format_exc(), "  ")
                self._logger.error("Worker threw an exception: \n%s", trace)
            finally:
                executor = self._executor_weakref()
                if executor is not None:
                    with executor._scaling_event:
                        executor._workers.remove(self)
                        executor._scaling_event.notify_all()

    def __init__(
        self,
        *,
        min_workers: typing.Optional[int] = None,
        max_workers: typing.Optional[int] = None,
        submission_quota: typing.Optional[int] = None,
        submission_patience: typing.Optional[float] = None,
        max_idle_time: typing.Optional[float] = None,
        logger: typing.Optional[logging.Logger] = None,
    ):
        """
        Initializes the thread pool.

        Args:
            min_workers: optional minimum number of workers in the pool, 0 by default.
            max_workers: optional maximum number of workers in the pool, 32 times the
                number of available CPU threads by default (assuming I/O bound work).
            submission_quota: optional maximum number of concurrent submissions for a
                a given callable. Up to the maximum number of workers by default. Useful
                when serving multiple users to prevent anyone from starving the rest.
            submission_patience: optional time to wait in seconds for a worker to
                become available before upscaling the pool. 100 ms by default.
            max_idle_time: optional time in seconds for a worker to wait for work
                before shutting itself down, effectively downscaling the pool. 60 seconds
                by default.
            logger: optional user provided logger for the pool.

        Raises:
            ValueError: if any argument is invalid.
            RuntimeError: if the interpreter is shutting down.
        """
        if min_workers is None:
            min_workers = 0
        if min_workers < 0:
            raise ValueError("Minimum number of workers must be a nonnegative number")
        self._min_workers = min_workers

        if max_workers is None:
            max_workers = 32 * (os.cpu_count() or 1)
        if max_workers < min_workers:
            raise ValueError("Maximum number of workers must be larger than or equal to the minimum number of workers")
        self._max_workers = max_workers

        if max_idle_time is None:
            max_idle_time = 60.0
        if max_idle_time <= 0:
            raise ValueError("Maximum idle time for workers must be a positive number")
        self._max_idle_time = max_idle_time

        if submission_quota is None:
            submission_quota = max_workers
        if submission_quota <= 0:
            raise ValueError("Quota for submission must be a positive number")
        self._submission_quota = submission_quota

        if submission_patience is None:
            submission_patience = 0.1
        if submission_patience < 0:
            raise ValueError("Patience for submission must be a nonnegative number")
        self._submission_patience = submission_patience

        if logger is None:
            logger = logging.getLogger(classname(self))
        self._logger = logger

        self._shutdown = False
        self._submit_lock = threading.Lock()
        self._shutdown_lock = threading.Lock()
        self._scaling_event = threading.Condition()

        self._waitqueues: typing.Dict[typing.Callable[..., typing.Any], collections.deque] = collections.defaultdict(
            collections.deque
        )
        self._runlists: typing.Dict[typing.Callable[..., typing.Any], typing.List[AutoScalingThreadPool.Work]] = (
            collections.defaultdict(list)
        )
        self._runslots = threading.Semaphore(0)

        runqueue: queue.SimpleQueue = queue.SimpleQueue()
        self._weak_self = weakref.ref(self, lambda ref: runqueue.put(None))

        with AutoScalingThreadPool._lock:
            if AutoScalingThreadPool._interpreter_shutdown:
                raise RuntimeError("cannot create thread pool while interpreter is shutting down")
            # register runqueue for external wake up  on interpreter shutdown
            self._runqueue = runqueue
            AutoScalingThreadPool._all_runqueues.add(runqueue)

            self._workers: weakref.WeakSet[AutoScalingThreadPool.Worker] = weakref.WeakSet()
            with self._scaling_event:
                for _ in range(self._min_workers):  # fire up stable worker pool
                    worker = AutoScalingThreadPool.Worker(self._weak_self, stop_on_timeout=False)
                    # register worker for external joining on interpreter shutdown
                    AutoScalingThreadPool._all_workers.add(worker)
                    self._workers.add(worker)
                    self._scaling_event.notify_all()

    @property
    def workers(self) -> typing.List[threading.Thread]:
        """Current set of worker threads."""
        with self._submit_lock:
            return list(self._workers)

    @property
    def scaling_event(self) -> threading.Condition:
        """A waitable condition triggered on pool (re)scaling."""
        return self._scaling_event

    @property
    def working(self) -> bool:
        """Whether work is ongoing or not."""
        with self._submit_lock:
            return any(work.pending() for runlist in self._runlists.values() for work in runlist) or any(
                work.pending() for waitqueue in self._waitqueues.values() for work in waitqueue
            )

    @property
    def capped(self) -> bool:
        """Whether submission quotas are in force or not."""
        with self._submit_lock:
            return any(work.pending() for waitqueue in self._waitqueues.values() for work in waitqueue)

    def wait(self, timeout: typing.Optional[float] = None) -> bool:
        """
        Waits for all work in the pool to complete.

        Only ongoing work at the time of invocation is watched after.
        Work added during the wait will not be considered.

        Args:
            timeout: optional timeout, in seconds, for the wait.

        Returns:
            True if all work completed, False if the wait timed out.
        """
        with self._submit_lock:
            futures = [work.future for runlist in self._runlists.values() for work in runlist]
            futures += [work.future for waitqueue in self._waitqueues.values() for work in waitqueue]
        done, not_done = concurrent.futures.wait(futures, timeout=timeout)
        return len(not_done) == 0

    def _cleanup_after(self, work: "AutoScalingThreadPool.Work") -> bool:
        complete = True
        with self._submit_lock:
            self._runlists[work.fn].remove(work)
            if work.fn in self._waitqueues and self._waitqueues[work.fn]:  # continue with pending work
                work = self._waitqueues[work.fn].popleft()
                while work.cancelled() and self._waitqueues[work.fn]:
                    work.notify_cancelation()
                    work = self._waitqueues[work.fn].popleft()
                if not self._waitqueues[work.fn]:
                    del self._waitqueues[work.fn]
                if not work.cancelled():
                    self._runlists[work.fn].append(work)
                    self._runqueue.put(work)  # actually submit work
                    complete = False
                else:
                    work.notify_cancelation()
            if not self._runlists[work.fn]:
                del self._runlists[work.fn]
        return complete

    def _do_submit(self, work: "AutoScalingThreadPool.Work") -> None:
        while not self._runslots.acquire(timeout=self._submission_patience):
            if len(self._workers) >= self._max_workers:
                break
            with self._scaling_event:
                worker = AutoScalingThreadPool.Worker(self._weak_self)
                with AutoScalingThreadPool._lock:
                    AutoScalingThreadPool._all_workers.add(worker)
                self._workers.add(worker)
                self._scaling_event.notify_all()
        self._runqueue.put(work)

    # NOTE(hidmic): cannot recreate type signature for method override
    # See https://github.com/python/typeshed/blob/main/stdlib/concurrent/futures/_base.pyi.
    def submit(  # type: ignore
        self, fn: typing.Callable[..., typing.Any], /, *args: typing.Any, **kwargs: typing.Any
    ) -> concurrent.futures.Future:
        """
        Submits work to the pool.

        Args:
            fn: a callable to execute. Must be immutable and hashable for
                the pool to track concurrent submissions and apply quotas.
            args: optional positional arguments to forward.
            kwargs: optional keyword arguments to forward.

        Returns:
            A future for the result of the work submitted.

        Raises:
            RuntimeError: if the pool has been shutdown.
        """
        with self._submit_lock, self._shutdown_lock:
            if self._shutdown:
                raise RuntimeError("cannot submit to a shutdown pool")
            future: concurrent.futures.Future = concurrent.futures.Future()
            WorkT = AutoScalingThreadPool.Work
            work = WorkT(future, fn, args, kwargs)
            if self._submission_quota > len(self._runlists[work.fn]):
                if work.fn in self._waitqueues and self._waitqueues[work.fn]:  # prioritize pending work
                    self._waitqueues[work.fn].append(work)
                    work = self._waitqueues[work.fn].popleft()
                    while work.cancelled() and self._waitqueues[work.fn]:
                        work.notify_cancelation()
                        work = self._waitqueues[work.fn].popleft()
                    if not self._waitqueues[work.fn]:
                        del self._waitqueues[work.fn]
                if not work.cancelled():
                    self._runlists[work.fn].append(work)
                    self._do_submit(work)  # actually submit work
                else:
                    work.notify_cancelation()
            else:
                self._waitqueues[work.fn].append(work)
            return future

    def shutdown(self, wait: bool = True, *, cancel_futures: bool = False) -> None:
        """
        Shuts down the pool.

        Args:
            wait: whether to wait for all worker threads to shutdown.
            cancel_futures: whether to cancel all ongoing work (and associated futures).
        """
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
            with self.scaling_event:

                def predicate() -> bool:
                    return len(self._workers) == 0

                self.scaling_event.wait_for(predicate)


atexit.register(AutoScalingThreadPool._on_interpreter_shutdown)


class AutoScalingMultiThreadedExecutor(rclpy.executors.Executor):
    """
    An rclpy.executors.Executor subclass based on an AutoScalingThreadPool.

    Akin to the rclpy.executors.MultiThreadedExecutor class but with autoscaling capabilities.
    Moreover, a concurrency quota can be defined on a per callback + callback group basis to
    avoid thread pool starvation and/or exhaustion of system resources (e.g. when using reentrant
    callback groups).

    See rclpy.executors.Executor documentation for further reference.
    """

    class Task:
        """A bundle of an executable task and its associated entity."""

        def __init__(self, task: rclpy.task.Task, entity: rclpy.executors.WaitableEntityType) -> None:
            self.task = task
            self.entity = entity
            self.callback_group = entity.callback_group if entity is not None else None

        def __call__(self) -> None:
            self.task.__call__()

        def __getattr__(self, name: str) -> typing.Any:
            return getattr(self.task, name)

        def __hash__(self) -> int:
            # ignore the task itself, as it changes from execution to execution
            return hash((self.entity, self.callback_group))

    def __init__(
        self,
        max_threads: typing.Optional[int] = None,
        max_thread_idle_time: typing.Optional[float] = None,
        max_threads_per_callback_group: typing.Optional[int] = None,
        *,
        context: typing.Optional[rclpy.context.Context] = None,
        logger: typing.Optional[logging.Logger] = None,
    ) -> None:
        """
        Initializes the executor.

        Args:
            max_threads: optional maximum number of threads to spin at any given time.
                See AutoScalingThreadPool documentation for reference on defaults.
            max_thread_idle_time: optional time in seconds for a thread to wait for work
                before shutting itself down. See AutoScalingThreadPool documentation for
                reference on defaults.
            max_threads_per_callback_group: optional maximum number of concurrent
                callbacks to service for a given callback group. Useful to avoid
                reentrant callback groups from starving the pool.
        """
        super().__init__(context=context)
        if logger is None:
            logger = rclpy.logging.get_logger(classname(self))
        self._thread_pool = AutoScalingThreadPool(
            max_workers=max_threads,
            max_idle_time=max_thread_idle_time,
            submission_quota=max_threads_per_callback_group,
            logger=logger,
        )
        self._futures: typing.List[AutoScalingMultiThreadedExecutor.Task] = []

    @property
    def thread_pool(self) -> AutoScalingThreadPool:
        """Autoscaling thread pool in use."""
        return self._thread_pool

    def _do_spin_once(self, *args: typing.Any, **kwargs: typing.Any) -> None:
        try:
            task, entity, node = self.wait_for_ready_callbacks(*args, **kwargs)
        except rclpy.executors.ExternalShutdownException:
            pass
        except rclpy.executors.ShutdownException:
            pass
        except rclpy.executors.TimeoutException:
            pass
        except rclpy.executors.ConditionReachedException:
            pass
        else:
            task = AutoScalingMultiThreadedExecutor.Task(task, entity)
            self._thread_pool.submit(task)
            self._futures.append(task)

            for task in self._futures[:]:
                if task.done():
                    self._futures.remove(task)
                    task.result()

    def spin_once(self, timeout_sec: typing.Optional[float] = None) -> None:
        self._do_spin_once(timeout_sec)

    def spin_once_until_future_complete(
        self, future: rclpy.task.Future, timeout_sec: typing.Optional[float] = None
    ) -> None:
        future.add_done_callback(lambda f: self.wake())
        self._do_spin_once(timeout_sec, condition=future.done)

    def shutdown(self, timeout_sec: typing.Optional[float] = None) -> bool:
        done = super().shutdown(timeout_sec)
        if done:
            self._thread_pool.shutdown()
        return done
