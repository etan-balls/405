Scheduler & Shared Variables
============================

cotask.py
---------

``cotask.py`` is a cooperative multitasking library originally written by JR Ridgely
for Cal Poly ME405. Tasks are Python generators registered in a ``TaskList`` and run
by a priority scheduler.

Creating and registering a task:

.. code-block:: python

   from cotask import Task, task_list

   task_list.append(Task(
       my_obj.run,       # generator function (no arguments)
       name="MyTask",
       priority=3,       # higher number = higher priority
       period=20,        # milliseconds between runs
   ))

   while True:
       task_list.pri_sched()   # call in the main loop

**Scheduling algorithm** (``pri_sched``):

Each call scans from the highest-priority group downward. Within each priority group
tasks are scheduled round-robin. The first task found to be *ready* (its period timer
has expired) is run for one generator step, then the scan restarts from the top.

This guarantees that a high-priority task can never be starved by a long sequence of
low-priority tasks.

**``Task.ready()``** checks the hardware microsecond timer against the next scheduled
run time. If the task is past due, ``go_flag`` is set and the method returns ``True``.

**Profiling**: pass ``profile=True`` to ``Task()`` to collect average and worst-case
execution times (in microseconds). Call ``print(task_list)`` to see the results.

task_share.py
-------------

Provides two interrupt-safe container classes for inter-task data exchange.

Share
~~~~~

A single-value buffer. Any task can write at any time; the latest value is always
available to readers. Used for continuously updated signals (sensor readings, effort
commands, state estimates).

.. code-block:: python

   from task_share import Share

   my_share = Share('f', thread_protect=False, name="my_signal")
   my_share.put(3.14)
   val = my_share.get()   # → 3.14

Type codes follow Python's ``array`` module: ``'b'`` int8, ``'f'`` float32, etc.
Set ``thread_protect=True`` if the share is written from an interrupt service routine.

Queue
~~~~~

A circular FIFO buffer. Used when a producer needs to pass a sequence of values to
a consumer without dropping data.

.. code-block:: python

   from task_share import Queue

   q = Queue('f', 200, name="data_log")
   q.put(value)      # blocks if full (unless overwrite=True)
   val = q.get()     # blocks if empty
   q.any()           # True if items available
   q.full()          # True if no room left

.. warning::
   ``Queue.get()`` and ``Queue.put()`` block (busy-wait) if the queue is empty or
   full respectively. Always check ``q.any()`` / ``q.full()`` before calling in a
   time-critical task, or set ``overwrite=True`` at construction.
