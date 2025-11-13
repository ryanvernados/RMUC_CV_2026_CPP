## 1. Logging Module
Supports both stream-style and printf-style logging.

Customizable log formats, log levels, and log separation.

Stream-style usage：CALIBUR_LOG_INFO(g_logger) << "this is a log".

Formatted usage：CALIBUR_LOG_FMT_INFO(g_logger, "%s", "this is a log").

Configurable fields: timestamp, thread ID, thread name, log level, logger name, filename, line number

## 2. Configuration Module
Convention-over-configuration design
YAML-based configuration with change notifications
Supports hierarchical data types and STL containers (vector, list, set, map)),
Custom type support through serialization/deserialization
```cpp
static calibur::ConfigVar<int>::ptr g_tcp_connect_timeout =
	calibur::Config::Lookup("tcp.connect.timeout", 5000, "tcp connect timeout");
```
A TCP connection timeout parameter has been defined. You can directly use g_tcp_connect_timeout->getValue()
to obtain the parameter's value. When the configuration is modified and reloaded, 
this value will be automatically updated.

The configuration format is as follows:
```YAML
tcp:
    connect:
            timeout: 10000
```
## 3. Thread Module
The thread module encapsulates some commonly used functionalities from pthread, 
including objects such as Thread, Semaphore, Mutex, RWMutex,
and Spinlock, facilitating daily thread operations in development.

Why not use C++11's std::thread?

Although this framework is developed using C++11, 
we chose not to use std::thread because it is ultimately implemented based on pthread. 
Additionally, C++11 does not provide certain synchronization primitives like read-write mutexes 
(RWMutex) and Spinlock, which are essential in high-concurrency scenarios. 
Therefore, we opted to encapsulate pthread ourselves to meet these requirements.

## 4. Coroutine
Coroutines are user-mode threads, akin to "threads within threads," 
but much more lightweight. By integrating with socket hooking, 
complex asynchronous calls can be encapsulated into synchronous operations,
significantly reducing the complexity of business logic implementation.

## 5. Coroutine Scheduler Module

A coroutine scheduler that manages coroutine execution, implemented as a thread pool with N:M scheduling model:
- **N** threads
- **M** coroutines
Efficiently reuses all threads for optimal performance.

### Key Features
- **Flexible Scheduling**:
  - Automatic load balancing across threads
  - Option to pin coroutines to specific threads
- **Efficient Resource Utilization**:
  ```math
  \text{Throughput} = \frac{\text{Coroutines Completed}}{\text{Thread Seconds}}

## 6. IO Coroutine Scheduler Module
Inherits from the coroutine scheduler and encapsulates epoll (Linux), supporting timer functionality 
(implemented using epoll with millisecond precision). 
Provides capabilities to add, remove, and cancel socket read/write events. 
Supports one-shot timers, recurring timers, and conditional timers.

## SETTING UP DETECTION 

## 1. Install OpenCV 
from Windows: https://opencv.org/releases/


