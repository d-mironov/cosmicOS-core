# cosmicHAL

The **cosmicHAL** is a small and easy to use Hardware Abstraction Layer to improve development speed and memory safety.  
This started as a personal project aimed on learning how to program low level hardware and quickly changed to a bigger and more ambitious project.  
The next step right now is to develop a good, quick and stable build system to speed up project generation, structure and management and also write an install script for such.

## Future Plans

### General System
- Developing a good, stable and quick build system
- Developing a software architecture/system to provide all the necessary files, tools and binaries
- Developing a lightweight package manager for drivers and libraries.

### HAL features
- support for more boards and vendors of ARM Cortex-M microcontrollers
- More peripherals to cover
- "ready for production" device drivers for sensors, etc.
- memory safety
- general headers for different drivers (IMU's, Motor controllers, display's, etc.)
- In house developed RTOS (maybe a little bit too ambitious, but we'll see)

## Installing
At the moment there is no easy install procedure but it is already work in progress.  
Right now you just clone the repository and put all of the `.h` and `.c` files into your project source where you can include them.  

## Usage
Example to blink LED:  
```c
#include "stm32/f4/gpio/gpio.h"
#include "stm32/f4/delay/delay.h"

int main(void) {
    // Initialize PC13 to output mode
    GPIO_enable(PC13, GPIO_OUTPUT);

    while(1) {
        // toggle PC13 LED
        GPIO_toggle(PC13);
        // sleep for 1s
        delayMs(1000);
    }
}
```
More examples to follow


## Contributing
You are very, very welcome to contribute since I work on this alone for now.  
Just contact me here:
- **Discord:** moonraccoon#1337
- **Mail:** [mail@moonraccoon.sh](mailto:mail@moonraccoon.sh)

##License
Copyright (c) 2020-2021 Daniel M. <mail@moonraccoon.sh>
It is licensed under an MIT license. So feel free to use it, just credit me in some way ^^.
