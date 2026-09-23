# MPU-6050 driver usage examples

These examples were developed and tested on a **NUCLEO-F767ZI**. They are not **standalone projects** - each is a `main()` 
meant to modify `Core/Src/main.c` in your project.

If you're targeting a different STM32 microcontroller, you'll need to:
- regenerate `Core/` for your MCU/board in CubeMX (peripherals, clocks, pins),
- update `mpu6050.h` to include the matching HAL header (e.g. `stm32f4xx_hal.h` instead of `stm32f7xx_hal.h`) if 
  you're on a different STM32 family,
- adjust the I2C/UART handle names used in each example (`hi2c1`, `huart3`) to whatever your CubeMX config generates,
- re-check I2C timing values / clock speed for your MCU's clock tree.

All examples assume `hi2c1` (sensor bus) and `huart3` (debug UART, ST-Link  VCP, 115200 8N1) — adjust to your 
board's config if different.

## Building an example
To build one, copy its contents  into `Core/Src/main.c` (replacing the current one) and rebuild. Adjust parameters 
defined in `/* Private define --- ... */` if needed.

## basic_read.c
Minimal usage of the MPU_6050 driver. Contains sensor initialization, DLPF configuration, calibration, reading data and
sending it over UART.