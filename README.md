# EmbeddedTask

This is a test project that implements fan speed control for a fan with build-in tachometer. The fan is controlled by a [MAX6650 IC](https://datasheets.maximintegrated.com/en/ds/MAX6650-MAX6651.pdf), which is connected to the microcontroller using an I2C interface. The fan operation parameters are controlled using commands transmitted from the terminal to the microcontroller via the UART interface. The following commands are implemented:
- *set_fan_speed X* - sets fan speed, X%. X should be a number in [0, 100] range. If X == 0, fan will be stopped.
- *get_fan_speed* - prints current fan speed in % and RPM.
- *self_erase* - erases microcontroller's FLASH memory. Requests additional confirmation before the FLASH memory erasing. After the FLASH memory erasing, microcontroller reacts to any commands with "No functional" message untill the power off.

During the execution of commands, the following errors may be detected:
- *Error: wrong value* - wrong value in *set_fan_speed* command parameter.
- *Error: unknown command* - unknown command was received.
- *Error: library error* - initialization or hardware error.

At the moment, the project is working with the following parameters:
- 12V fan (can be changed in MAX6650 initialization function call, 12V and 5V fans are available)
- Minimal (1%) fan speed is 500 RPM (can be changed in MAX6650 initialization function call, [240, 30000] RPM range is available)
- Maximal (100%) fan speed is 10500 RPM (can be changed in MAX6650 initialization function call, [240, 30000] RPM range is available)
- MAX6650 I2C address pin is unconnected (I2C slave address is 0x36)
- MAX6650 static library compiled for ARM Cortex M4 core
- Project compiled for STM32F401CB MCU

## How to build

- Ensure that [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) installed and available:
  `make -version`
- Go to the MAX6650 static library directory and build the library:
  ```
  cd libs/MAX6650
  make
  ```
- Return to the project directory and build the project
  ```
  cd ../..
  make
  ```