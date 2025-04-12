Bitbot Note
===

> Bitbot, developed by [God Yuan](https://lmy.name), is a modern C++ framework for building robot control systems.
> 
> Reference: [Bitbot](https://bitbot.lmy.name)

# 1 Usage

To use Bitbot, you should fill a `.xml` and a `.json` file, then modify the `main.cpp`, `user_func.cpp` and `user_func.h` to fit your needs.

`my_config.xml` example:

```xml
<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<bitbot>
  <!-- Available log level: trace, debug, info, warn, error, critical, off -->
  <logger path="./log" level="info"/>
  <backend port="12888" settings_file="./backend.json"/>
  <bus>
    <device id="1" type="MujocoJoint" name="joint_x"
            mode="position"
            pos_kp='28000' pos_kd='50' pos_ki='0' vel_kp='10' vel_kd='0'
    />
    <device id="2" type="MujocoJoint" name="joint_y"
            mode="position" initial_pos='30'
            pos_kp='28000' pos_kd='50' pos_ki='0' vel_kp='10' vel_kd='0'
    />
  </bus>
</bitbot>
```

`backend.json` example:

```json
{
  "control": [
    // enable_record, stop and start are built-in events
    {
      "event": "enable_record",
      "kb_key": "m"
    },
    {
      "event": "stop",
      "kb_key": " "
    },
    {
      "event": "start",
      "kb_key": "8"
    },
    // User defined events
    {
      "event": "test",
      "kb_key": "t"
    }
    // You can add joystick events
  ]
}
```

Then you should write the event and state function.

**API cheat sheet**

```cpp
/// Event function
/// @brief Event function type
/// @return The next state to transfer
using EventFunc = std::function<std::optional<StateId>(bitbot::EventValue value, UserData& user_data)>;
/// @brief Register event function in kernel
/// @param always_enabled Flag to enable event for all states
void RegisterEvent(std::string name, bitbot::EventId id, bitbot::EventFunc func, bool always_enabled = false);

/// State function
// State function type
using StateFunc = std::function<void(const bitbot::KernelInterface& kernel, bitbot::ExtraData& extra_data, UserData& user_data)>;
/// @brief Register state function in kernel
/// @param events Declear events responsed in this state, alongside events enabled for all states.
void RegisterState(std::string name, bitbot::StateId id, bitbot::StateFunc func, std::vector<uint32_t> events);

/// Config function will be run once the kernel is initialized
// Config function type
using ConfigFunc = std::function<void(const BusManagerT & bus, UserData& user_data)>;
// Register config function in kernel
void RegisterConfigFunc(ConfigFunc func);

/// Finish function will be run once the kernel is finished
// Finish function type
using FinishFunc = std::function<void(UserData&)>;
// Register finish function in kernel
void RegisterFinishFunc(FinishFunc func);

// Set the initial state
void SetFirstState(StateId id);

// To emit an event
void EmitEvent(EventId id, EventValue value);
```

There are 3 built-in events in Bitbot: `STOP`, `START` and `ENABLE_RECORD`; and one built-in state: `IDLE`.

The Bitbot kernel is a template. We could define the ExtraData (number type only, see `bitbot::Number`) and UserData, which can be accessed in functions registered above.

```cpp
// User data sample
struct UserData
{
  double sin = 10;
};

// "time" and "sin" 
using Kernel = bitbot::MujocoKernel<UserData, "time", "sin">;

// Use extra_data in state function to sync data to frontend
extra_data.Set<0>(2);  // Set time by index
extra_data.Set<"sin">(user_data.sin);  // Set sin by name
```

# 2.2 Code Basics

## 2.1 Modern C++ Basics in Bitbot

Refer to [cpp/modern_cpp.md](cpp/modern_cpp.md).

## 2.2 WebSockets

How to play with Bitbot backend in terminal:

```shell
# sudo apt install websocat
websocat -v ws://127.0.0.1:12888/console
```

or,

```shell
# npm install -g wscat
wscat -c ws://127.0.0.1:12888/console
```

Supported message types:

```json
// Get the current state value
{"type":"request_data","data":""}
// Send keyboard pressing event
{"type":"events","data":"{\"events\":[{\"name\":\"stop\",\"value\":1}]}"}
// Send keyboard releasing event
{"type":"events","data":"{\"events\":[{\"name\":\"stop\",\"value\":2}]}"}
```

Have fun!

# 3 Code structure

## 3.1 `bitbot_kernel`

CPU core allocation:

1. The main thread, i.e. `Kernel::doRun()`, is sticked to core 4.
2. Kernel monitor data writer is sticked to core 5.

### 3.1.1 `kernel`

`KernelTpl` uses CRTP pattern. The derived class should implement the following functions: `doRun()` and `doStart()`.

`KernelTpl::Run()` is user interface to start the kernel. It calls `doRun()` in the derived class.

`doStart()` in the derived class is called in the `Start` event callback.

Events should be registered before states.

### 3.1.2 `backend`

`bitbot::Backend` is a class to handle the HTTP and WebSocket servers. It is written with library `uWebSockets` as HTTP and WebSocket server. JSON library `glaze`, which requires a really modern C++ version, is used in backend to save a class instance to a JSON string.

There are 3 HTTP servers:

1. `/monitor/headers` get monitor headers. JSON class is `KernelTpl::MonitorHeader`. JSON string is set in kernel.
2. `/monitor/statelist` get state id and name. Relative json class is `StatesType`.
3. `/setting/control/get` get event name and its corresponding key. Relative json class is `BackendSettings`.

All message sent between the WebSocket server and client should be `WebsocketMessageType`. The WebSocket server handles the following messages:

1. `request_data`. The server then fills the monitor data and sends it back.
2. `events`. 

The frontend should send `request_data` message periodically, and send `events` message the corresponding event is triggered.

The received events are saved in a `readwritequeue` in the backend. Where are these events processed?

### 3.1.3 `bus_manager`

`BusManagerTpl` uses CRTP pattern. It creates devices from XML file.

`ReadBus()` and `WriteBus()` are called in `Kernel::KernelLoopTask()`. These two function are main interfaces during processing.

### 3.1.4 `device`

In device factory file `DeviceFactory` and `DeviceRegistrar: DeviceRegistrarBase` are defined. They are all base classes.

`DeviceFactory` uses singleton pattern. `DeviceRegistrar` of different device types are inserted in factory, then user can use the factory to create device registered in factory, parsing various device config from XML file.

`DeviceRegistrar` is used to create devices with the same type based on XML file.

### 3.1.5 Realtime Configuration

> Test these functions.

Set priority of the process.

Set deadline policy of the process.

Set CPU affinity of the process. Stick the thread to a specific core.

> In real-time systems or applications that require low-latency, controlling CPU affinity can ensure that critical threads are given consistent CPU time on a specific core, which reduces unpredictability and latency.

### 3.1.6 Others

* `config_parser`

`ConfigParser` is used to parse the xml file.

`DeviceConfigs` is not used further, since the reading function can read nothing. Parsing is done in Kernel and BudManager directly.

* `extra_data`

`ExtraDataImpl` is a template class to store extra data. It takes a `CTString` pack as the template argument. Extra datas can be set by index or name string, since `CTSTuple` can convert `CTString` to index at compile time.

* `compile_time_string`

`CTString` and `CTSTuple` are defined.

* `logger`

`spdlog` is used as the logger. The code is written in a singleton pattern.

* `time_func`

Delay functions.

* `file_writer`

`FileWriter` keeps a `readwritequeue` to storage data, and crates a thread to write data to file periodically.

## 3.2 `cifx_kernel`

### 3.2.1 EtherCAT Bacis

EtherCAT uses a "processing-on-the-fly" mechanism, where data frames pass through each node (slave device) in a daisy-chained topology. Each node reads or writes data in real time as the frame travels through the network, minimizing latency and maximizing efficiency. A typical EtherCAT network has one master and multiple slaves. **Master card** (a PCIe card) is a EtherCAT master interface installed in the PC.

**SDO** (Service Data Object) is a communication mechanism inherited from the CANopen protocol, which EtherCAT adopts for configuration and parameterization. SDOs are used for non-real-time, asynchronous data exchange, such as: configuration, parameterization, and diagnostics.

PDOs (Process Data Objects) handle cyclic real-time data.

<mark>How to display the received status word?</mark>

## 3.3 `mujoco_kernel`

# 4 RL Deploy

The code provided by God Z. uses `libtorch` as inference engine. We may use `onnxruntime` later.

Torque compensate?

Filter?

Noise generator?

Action interpolation buffer. `[policy_frequency+1][num_action]`

In `CoinfigFunc()`, joint handle, IMU handle and force sensor handle are obtained via `bitbot::CifxBus`, then the action interpolation buffer and inference net are created in `InitPolicy()`.

Then `EventInitPose` should be emitted. The kernel then drumps to the `StateJointInitPose()` state, a PD position controller, in witch `InitPose()` is called. In `InitPose()` function `realtime_1D_interpolation_5()` is used to compute a target joint position, then the angles are saved in `target_angle`. Then `TorqueController()` is called, in which a PD controller is execuated. The `StateJointInitPose()` will be called periodically, thus the robot will move to the target position controlled by a PD controller.

After that `PolicyRun` event should be emitted. The kernel then drumps to the `StatePolicyRun()` state, in which `PolicyController()` is called.

`TorqueController()` is a PD controller.


