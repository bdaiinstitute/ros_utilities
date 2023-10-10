# Copyright (c) 2023 Boston Dynamics AI Institute Inc.  All rights reserved.
import atexit
import collections
import concurrent.futures
import contextlib
import dataclasses
import inspect
import logging
import os
import queue
import threading
import typing
import weakref

import rclpy.executors

from bdai_ros2_wrappers.utilities import bind_to_thread, fqn


class AutoScalingThreadPool(concurrent.futures.Executor):
    """
    A concurrent.futures.Executor subclass based on a thread pool.

    Akin to the concurrent.futures.ThreadPoolExecutor class but with
    autoscaling capabilities. Within a given range, the number of
    workers increases with demand and decreases with time. This is
    achieved by tracking the number of available runslots ie. the
    number of idle workers waiting for work, and using timeouts to
    wait for work. Workers add runslots prior to blocking on the
    runqueue. On submission, a runslot will be taken. If there is no
    runslot to take, the pool will be scaled up. If a work has been
    waiting long enough and no work has come along, it will be self
    terminate, effectively downscaling the pool.

    Additionally, individual submissions are tracked and monitored against
    a configurable quota to avoid any given piece of work from starving the pool.
    To do this, the implementations takes after CPython's implementation
    of the concurrent.futures.ThreadPoolExecutor class, adding runlists
    and waitqueues per submission "type" (ie. callable hashes) to the main
    runqueue. Runlists track work either pending execution in the runqueue
    or executing. Waitqueues track work that is to pushed into the runqueue
    once the configured quota allows it.

    If not shutdown explictly or via context management, the pool will
    self terminate when either the executor is garbage collected or the
    interpreter shuts down.

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

        def __str__(self) -> str:
            return f"{fqn(self.fn) or fqn(type(self.fn))} ({hash(self.fn)})"

    class Worker(threading.Thread):
        """A worker in its own daemonized OS thread."""

        def __init__(self, executor_weakref: weakref.ref, stop_on_timeout: bool = True) -> None:
            """
            Initializes the worker.

            Args:
                executor_weakref: a weak reference to the parent autoscaling thread pool.
                stop_on_timeout: whether the worker should auto-terminate if it times out
                waiting for work.
            """
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
            self._logger.debug("Starting worker...")
            self.start()

        def run(self) -> None:
            """Runs work loop."""
            try:
                self._logger.debug("Worker started")
                work: typing.Optional[AutoScalingThreadPool.Work] = None
                while True:
                    if AutoScalingThreadPool._interpreter_shutdown:
                        self._logger.debug("Interpreter is shutting down! Terminating worker...")
                        self._runqueue.put(None)
                        break

                    executor: typing.Optional[AutoScalingThreadPool] = self._executor_weakref()
                    if executor is None:
                        self._logger.debug("Executor is gone! Terminating worker...")
                        self._runqueue.put(None)
                        break

                    if executor._shutdown:
                        self._logger.debug("Executor is shutting down! Terminating worker...")
                        self._runqueue.put(None)
                        break

                    runslots = executor._runslots
                    if work is not None:
                        if executor._cleanup_after(work):
                            self._logger.debug("Making worker available for work...")
                            runslots.release()
                            self._logger.debug("Worker made available for work")
                        work = None
                    else:
                        self._logger.debug("Making worker available for work...")
                        runslots.release()
                        self._logger.debug("Worker made available for work")
                    del executor  # drop reference

                    self._logger.debug("Worker waiting for work...")
                    try:
                        work = self._runqueue.get(block=True, timeout=self._timeout)
                    except queue.Empty:
                        self._logger.debug("Worker timed out waiting!")
                        if runslots.acquire(blocking=False):
                            self._logger.debug("Terminating worker...")
                            break
                        self._logger.debug("Work incoming, worker back to waiting")
                        continue

                    if work is not None:
                        self._logger.debug(f"Worker executing work '{work}'...")
                        work.execute()
                        self._logger.debug(f"Worker done executing work '{work}'")
            except BaseException:
                import textwrap
                import traceback

                trace = textwrap.indent(traceback.format_exc(), "  ")
                self._logger.error(f"Worker threw an exception: \n{trace}")
            finally:
                self._logger.debug("Worker terminated")
                executor = self._executor_weakref()
                if executor is not None:
                    self._logger.debug("Downscaling pool...")
                    with executor._scaling_event:
                        # NOTE(hidmic): worker removes itself from the pool
                        # to avoid race conditions between the scaling event
                        # and the thread actually getting clobbered.
                        executor._workers.remove(self)
                        executor._scaling_event.notify_all()
                    self._logger.debug("Done downscaling")

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
        if max_workers <= 0:
            raise ValueError("Maximum number of workers must be a positive number")
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
            logger = logging.getLogger(fqn(self.__class__))
        self._logger = logger

        self._logger.debug("Initializing thread pool...")
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
            self._runqueue = runqueue
            self._logger.debug("Registering runqueue for external wake up on interpreter shutdown...")
            AutoScalingThreadPool._all_runqueues.add(runqueue)
            self._logger.debug("Done registering runqueue")

            self._workers: weakref.WeakSet[AutoScalingThreadPool.Worker] = weakref.WeakSet()
            if self._min_workers > 0:
                with self._scaling_event:
                    self._logger.debug(f"Pre-populating pool with {self._min_workers} workers")
                    for _ in range(self._min_workers):  # fire up stable worker pool
                        worker = AutoScalingThreadPool.Worker(self._weak_self, stop_on_timeout=False)
                        # register worker for external joining on interpreter shutdown
                        self._logger.debug("Registering worker for external joining on interpreter shutdown...")
                        AutoScalingThreadPool._all_workers.add(worker)
                        self._logger.debug("Done registering worker")
                        self._logger.debug("Adding worker to the pool...")
                        self._workers.add(worker)
                        self._logger.debug("Worker added")
                    self._scaling_event.notify_all()
                    self._logger.debug("Done pre-populating")
            self._logger.debug("Done initializing thread pool")

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
            self._logger.debug(f"Cleaning up after work '{work}'")
            self._runlists[work.fn].remove(work)
            if work.fn in self._waitqueues and self._waitqueues[work.fn]:  # continue with pending work
                self._logger.debug("Have similar work pending!")
                self._logger.debug("Fetching pending work...")
                work = self._waitqueues[work.fn].popleft()
                while work.cancelled() and self._waitqueues[work.fn]:
                    self._logger.debug(f"Work '{work}' was cancelled, notify it")
                    work.notify_cancelation()
                    work = self._waitqueues[work.fn].popleft()
                self._logger.debug(f"Got work '{work}'")
                if not self._waitqueues[work.fn]:
                    del self._waitqueues[work.fn]
                if not work.cancelled():
                    self._logger.debug(f"Proceed with work '{work}'")
                    self._runlists[work.fn].append(work)
                    self._runqueue.put(work)  # actually submit work
                    complete = False
                else:
                    self._logger.debug(f"Work '{work}' was cancelled, notify it")
                    work.notify_cancelation()
            if not self._runlists[work.fn]:
                del self._runlists[work.fn]
        return complete

    def _do_submit(self, work: "AutoScalingThreadPool.Work") -> None:
        self._logger.debug("Looking for workers...")
        while not self._runslots.acquire(timeout=self._submission_patience):
            self._logger.debug(f"Not enough workers to execute work '{work}'")
            if len(self._workers) >= self._max_workers:
                self._logger.debug("Pool already hit its maximum size, nothing to do")
                break
            self._logger.debug("Upscaling pool...")
            with self._scaling_event:
                worker = AutoScalingThreadPool.Worker(self._weak_self)
                with AutoScalingThreadPool._lock:
                    AutoScalingThreadPool._all_workers.add(worker)
                self._workers.add(worker)
                self._scaling_event.notify_all()
            self._logger.debug("Done upscaling")
        self._logger.debug("Got worker!")
        self._logger.debug(f"Queuing work '{work}' for execution...")
        self._runqueue.put(work)
        self._logger.debug(f"Work '{work}' queued")

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
            work = AutoScalingThreadPool.Work(future, fn, args, kwargs)
            self._logger.debug(
                f"Submitting work '{work}'...",
            )
            if self._submission_quota > len(self._runlists[work.fn]):
                if work.fn in self._waitqueues and self._waitqueues[work.fn]:  # prioritize pending work
                    self._logger.debug("Have similar work pending")
                    self._logger.debug(f"Work '{work}' put to wait", work)
                    self._waitqueues[work.fn].append(work)
                    self._logger.debug("Fetching pending work...")
                    work = self._waitqueues[work.fn].popleft()
                    while work.cancelled() and self._waitqueues[work.fn]:
                        self._logger.debug(f"Work '{work}' was cancelled, notify it")
                        work.notify_cancelation()
                        work = self._waitqueues[work.fn].popleft()
                    if not self._waitqueues[work.fn]:
                        del self._waitqueues[work.fn]
                    self._logger.debug(f"Got work '{work}'")
                if not work.cancelled():
                    self._logger.debug(f"Proceed with work '{work}'")
                    self._runlists[work.fn].append(work)
                    self._do_submit(work)  # actually submit work
                else:
                    self._logger.debug(f"Work '{work}' was cancelled, notify it")
                    work.notify_cancelation()
            else:
                self._logger.debug(f"Hit quota for work '{work}'")
                self._waitqueues[work.fn].append(work)
                self._logger.debug(f"Work '{work}' put to wait")
            self._logger.debug(f"Done submitting work '{work}'")
            return future

    def shutdown(self, wait: bool = True, *, cancel_futures: bool = False) -> None:
        """
        Shuts down the pool.

        Args:
            wait: whether to wait for all worker threads to shutdown.
            cancel_futures: whether to cancel all ongoing work (and associated futures).
        """
        self._logger.debug("Shutting down pool...")
        with self._shutdown_lock:
            self._shutdown = True
        self._logger.debug("Pool shutdown")

        if cancel_futures:
            with self._submit_lock:
                self._logger.debug("Canceling all work in progress...")
                for runlist in self._runlists.values():
                    for work in runlist:
                        work.cancel()
                self._logger.debug("Work in progress cancelled")
                self._logger.debug("Canceling all pending work...")
                for waitqueue in self._waitqueues.values():
                    for work in waitqueue:
                        work.cancel()
                self._logger.debug("Pending work cancelled")

        self._logger.debug("Wake up all workers for cleanup")
        self._runqueue.put(None)
        if wait:
            self._logger.debug("Waiting for workers to terminate...")
            with self.scaling_event:

                def predicate() -> bool:
                    return len(self._workers) == 0

                self.scaling_event.wait_for(predicate)
            self._logger.debug("Workers terminated, pool is empty")


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

        def __init__(self, task: rclpy.task.Task, entity: typing.Optional[rclpy.executors.WaitableEntityType]) -> None:
            self.task = task
            self.entity = entity
            self.callback_group = entity.callback_group if entity is not None else None

        def __call__(self) -> None:
            self.task.__call__()

        def __getattr__(self, name: str) -> typing.Any:
            return getattr(self.task, name)

        def cancel(self) -> None:
            # This is a re-implementation to cope with rclpy.task.Task
            # leaving coroutines unawaited upon cancellation.
            schedule_callbacks = False
            with self.task._task_lock, self.task._lock:
                if not self.task._done:
                    if inspect.iscoroutine(self.task._handler):
                        self.task._handler.close()
                    self.task._done = self.task._cancelled = True
                    schedule_callbacks = True
                else:
                    self.task.exception()  # always retrieve exception
            if schedule_callbacks:
                self.task._schedule_or_invoke_done_callbacks()

        def __hash__(self) -> int:
            # Ignore the task itself, as it changes from execution to execution
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
            logger = rclpy.logging.get_logger(fqn(self.__class__))
        self._is_shutdown = False
        self._spin_lock = threading.Lock()
        self._shutdown_lock = threading.RLock()
        self._thread_pool = AutoScalingThreadPool(
            max_workers=max_threads,
            max_idle_time=max_thread_idle_time,
            submission_quota=max_threads_per_callback_group,
            logger=logger,
        )
        self._wip: typing.Dict[AutoScalingMultiThreadedExecutor.Task, concurrent.futures.Future] = {}

    @property
    def thread_pool(self) -> AutoScalingThreadPool:
        """Autoscaling thread pool in use."""
        return self._thread_pool

    def _do_spin_once(self, *args: typing.Any, **kwargs: typing.Any) -> None:
        with self._spin_lock:
            try:
                task, entity, node = self.wait_for_ready_callbacks(*args, **kwargs)
                task = AutoScalingMultiThreadedExecutor.Task(task, entity)
                with self._shutdown_lock:
                    if self._is_shutdown:
                        # Ignore task, let shutdown clean it up.
                        return
                    # The following guards against a TOCTOU race between rclpy.executors.Executor
                    # base implementation checking for executing tasks and tasks actually executing
                    # in the thread pool. That is, a task may be executing at the time of check but
                    # be done by the time it is about to be submitted to the pool. The only source
                    # of truth in that scenario is the future of the last submission (which
                    # guarantees the task is not executing if done).
                    #
                    # Another race remains, however, for asynchronous tasks (ie. using coroutines)
                    # between the time an execution cycle is complete and the corresponding
                    # submission future is done. That is, a task could be legitimately ready for
                    # dispatch and be missed. Fortunately, this will only delay dispatch until the
                    # next spin cycle.
                    if task not in self._wip or (self._wip[task].done() and not task.done()):
                        self._wip[task] = self._thread_pool.submit(task)
                    for task in list(self._wip):
                        if task.done():
                            del self._wip[task]
                            try:
                                task.result()
                            except rclpy.executors.InvalidHandle:
                                # ignore concurrent entity destruction
                                pass
            except rclpy.executors.ConditionReachedException:
                pass
            except rclpy.executors.ExternalShutdownException:
                pass
            except rclpy.executors.ShutdownException:
                pass
            except rclpy.executors.TimeoutException:
                pass

    def spin_once(self, timeout_sec: typing.Optional[float] = None) -> None:
        self._do_spin_once(timeout_sec)

    def spin_once_until_future_complete(
        self, future: rclpy.task.Future, timeout_sec: typing.Optional[float] = None
    ) -> None:
        future.add_done_callback(lambda f: self.wake())
        self._do_spin_once(timeout_sec, condition=future.done)

    def shutdown(self, timeout_sec: typing.Optional[float] = None) -> bool:
        with self._shutdown_lock:
            # Before actual shutdown and resource cleanup, all pending work
            # must be waited on. Work tracking in rclpy.executors.Executor
            # base implementation is subject to races, so block thread pool
            # submissions and wait for all futures to finish. Then shutdown.
            done = self._thread_pool.wait(timeout_sec)
            if done:
                assert super().shutdown(timeout_sec=0)
                self._thread_pool.shutdown()
                self._is_shutdown = True
        if done:
            with self._spin_lock:
                # rclpy.executors.Executor base implementation leaves tasks
                # unawaited upon shutdown. Do the housekeepng.
                for task, entity, _ in self._tasks:
                    task = AutoScalingMultiThreadedExecutor.Task(task, entity)
                    task.cancel()
        return done


@contextlib.contextmanager
def background(executor: rclpy.executors.Executor) -> typing.Iterator[rclpy.executors.Executor]:
    """
    Pushes an executor to a background thread.

    Upon context entry, the executor starts spinning in a background thread.
    Upon context exit, the executor is shutdown and the background thread is joined.

    Args:
        executor: executor to be managed.

    Returns:
        a context manager.
    """
    background_thread = threading.Thread(target=executor.spin)
    executor.spin = bind_to_thread(executor.spin, background_thread)
    executor.spin_once = bind_to_thread(executor.spin_once, background_thread)
    executor.spin_until_future_complete = bind_to_thread(executor.spin_until_future_complete, background_thread)
    executor.spin_once_until_future_complete = bind_to_thread(
        executor.spin_once_until_future_complete, background_thread
    )
    background_thread.start()
    try:
        yield executor
    finally:
        executor.shutdown()
        background_thread.join()


@contextlib.contextmanager
def foreground(executor: rclpy.executors.Executor) -> typing.Iterator[rclpy.executors.Executor]:
    """
    Manages an executor in the current thread.

    Upon context exit, the executor is shutdown.

    Args:
        executor: executor to be managed.

    Returns:
        a context manager.
    """
    try:
        yield executor
    finally:
        executor.shutdown()
