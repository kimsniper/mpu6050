# MPU5060 - ESP-IDF

MPU5060 i2c library for ESP-IDF.
ESP-IDF template used for this project: https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_simple

## Overview

This example demonstrates usage of MPU5060 for reading linear accelaration and angular velocity.

### Hardware Required

The MPU-60X0 is the world’s first integrated 6-axis MotionTracking device that combines a 3-axis
gyroscope, 3-axis accelerometer, and a Digital Motion Processor™ (DMP) all in a small 4x4x0.9mm
package. With its dedicated I C sensor bus, it directly accepts inputs from an external 3-axis compass to
provide a complete 9-axis MotionFusion™ output. The MPU-60X0 MotionTracking device, with its 6-axis
integration, on-board MotionFusion™, and run-time calibration firmware, enables manufacturers to eliminate
the costly and complex selection, qualification, and system level integration of discrete devices, guaranteeing
optimal motion performance for consumers. The MPU-60X0 is also designed to interface with multiple non-inertial digital sensors, such as pressure sensors, on its auxiliary I C port. The MPU-60X0 is footprintcompatible with the MPU-30X0 family.
Datasheet can be found [here](https://www.analog.com/media/en/technical-documentation/data-sheets/MPU5060.pdf).

#### Pin Assignment:

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| MPU5060          | SDA            | SCL            |


For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `Example Configuration` in `menuconfig`.

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash

```
