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
// Event function type
using EventFunc = std::function<std::optional<StateId>(bitbot::EventValue value, UserData& user_data)>;
// Register event function in kernel
void RegisterEvent(std::string name, bitbot::EventId id, bitbot::EventFunc func, bool always_enabled = false);

/// State function
// State function type
using StateFunc = std::function<void(const bitbot::KernelInterface& kernel, bitbot::ExtraData& extra_data, UserData& user_data)>;
// Register state function in kernel
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
```

# 2 Code Structure

## 2.1 Modern C++ Basis in Bitbot

### `std::variant` (C++17)

`std::variant` is a C++17 feature that allows you to create a type-safe union.

Example:

```cpp
#include <variant>
#include <iostream>
std::variant<uint32_t, double> var;
var = uint32_t(10);
if (std::holds_alternative<uint32_t>(myVariant)) {
  std::cout << std::get<uint32_t>(var) << std::endl;
}
var = double(3.14);
if (std::holds_alternative<double>(myVariant)) {
  std::cout << std::get<double>(var) << std::endl;
}
```

The traditional union is not type-safe, and we don't know which type is stored currently. Moreover, traditional union does not support complex type, such as `std::string`.

### Designated Initializer (C++20)

Designated Initializer is a C++20 feature that allows you to initialize a struct with designated fields.

Sample code:

```cpp
class MyClass {
public:
  int a;
  double b;
};

MyClass my_class = {.a = 10, .b = 3.14};
```

## 2.2 WebSockets

How to play with Bitbot backend in terminal:

```shell
websocat -v ws://127.0.0.1:12888/console
```

or,

```shell
wscat -c ws://127.0.0.1:12888/console
```

Supported message types:

```json
// Get the current state value
{"type":"request_data","data":""}
// Send keyboard press event
{"type":"events","data":"{\"events\":[{\"name\":\"stop\",\"value\":1}]}"}
// Send keyboard release event
{"type":"events","data":"{\"events\":[{\"name\":\"stop\",\"value\":2}]}"}
```

Have fun!
