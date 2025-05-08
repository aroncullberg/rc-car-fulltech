# Style guide (C/C++)

## Naming converions
- **Namespaces**: Use lowercase_snake_case for namespace names
```c++
namespace sensor_utils
{
// No indentation
}
```
- **Classes & Structs**: Use UpperCamelCase, may use `_` to increase readability (`GPIO_Output` vs `GPIOOutput` ).
```c++
class MotorController_ 
{
    // ...
};
```
```c++
struct Motor 
{
    // ...
}
```
- 

## Include order
```c++
// C/C++ standard library
#include <stdio.h>

// C/C++ standard library
#include <string>   

// Common ESP-IDF headers
#include "esp_log.h"

// Headers from other components or libraries (e.g. FreeRTOS or third-party component headers).
#include "your_component/sensor_reader.hpp"

// Public headers of the current component (the API headers in the include/ directory).
#include "sensor_reader.hpp"  // public header

// Private headers of the current component (the API headers in the include/ directory).
#include "sensor_reader_internal.hpp"  // private header
```

## namespaces
- This serves a similar purpose as the prefix convention Espressif uses in C (e.g. `esp_console_*` functions)