# BMP280 - Gecko SDK 4.3.1

BMP280 i2c and SPI library for Silicon Labs Gecko SDK 4.3.1.

Hardware: https://www.silabs.com/development-tools/wireless/bluetooth/bgm220-explorer-kit

## Overview

This example demonstrates usage of BMP280 for reading barometeric pressure.

### Hardware Required

To run this example, you should have BGM220P as well as a BMP280. The BMP280 is an absolute barometric pressure sensor especially designed for mobile applications. The sensor module is housed in an extremely compact 8-pin metal-lid LGA package with a footprint of only 2.0 × 2.5 mm2 and 0.95 mm package height. Its small dimensions and its low power consumption of 2.7 µA @1Hz allow the implementation in battery driven devices such as mobile phones, GPS modules or watches.. It is easy to operate via a simple I2C command, you can read the datasheet [here](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf).

#### Pin Assignment:

**Note:** The following pin assignments are used by default for BGM220P explorer kit mikro bus pins.

|                      | SDA            | SCL            |
| -------------------- | -------------- | -------------- |
| BGM220P I2C Master   | Port D, Pin 2  | Port D, Pin 3  |
| BMP280               | SDA            | SCL            |

**Note: ** There’s no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.
**Note: ** Driver is tested with Simplicity Studio 5 IDE with Gecko SDK 4.3.1. Hardware use: BGM220P Explorer kit

### Build and Flash

1. Open Simplicity Studio 5.
2. Select File > Import, and locate the folder with the project or solution to be imported.
Ex: (Folder Path)\BMP280\examples\bgm220p_implementation
3. Select the project and click Next. If anything about the project is unresolved you can resolve it.
4. Click Next. Name the project or solution and click Finish.
5. Right click project and select Run As and select the appropriate program.
6. Open any UART terminal application and set the baudrate to 115200 with the right COM port.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://www.silabs.com/development-tools/wireless/bluetooth/bgm220-explorer-kit?tab=getting-started) for full steps to start with BGM220P explorer kit.

## Example Output

```bash
[I] Part number: 0x58
[I] Calibration data setting: Successful
[I] Setting to normal mode: Successful
[I] BMP280 initialization successful
[I] Pressure: 100734 Pa
[I] Temperature: 32 °C
[I] Pressure: 100734 Pa
[I] Temperature: 32 °C
[I] Pressure: 100735 Pa
[I] Temperature: 32 °C
[I] Pressure: 100735 Pa
[I] Temperature: 32 °C
[I] Pressure: 100735 Pa
[I] Temperature: 32 °C
[I] Pressure: 100734 Pa
[I] Temperature: 32 °C
[I] Pressure: 100733 Pa
[I] Temperature: 32 °C
[I] Pressure: 100736 Pa
[I] Temperature: 32 °C
[I] Pressure: 100735 Pa
```
