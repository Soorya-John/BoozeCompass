# Wiring

## Pin map in firmware

The exact ESP32-C3 development board and revision is unknown. The following assignments are provisional and use the ESP32-C3-DevKitM-1 reference pinout. Verify the actual board schematic and header labels before wiring. All assignments are configurable through idf.py menuconfig → BoozeCompass.

| ESP32-C3 signal | Default GPIO | Connect to | Notes |
|---|---:|---|---|
| I2C SDA | 4 | BZ251 SDA and MPU6050 SDA | Shared bus, open-drain |
| I2C SCL | 5 | BZ251 SCL and MPU6050 SCL | Shared bus, 100 kHz default |
| UART1 RX | 6 | BZ251 TX | Cross TX to RX |
| UART1 TX | 7 | BZ251 RX | Cross RX to TX; receiver input is optional if only parsing output |
| ULN2003 IN1 | 0 | Driver IN1 | Only active when motor build option is enabled |
| ULN2003 IN2 | 1 | Driver IN2 | Only active when motor build option is enabled |
| ULN2003 IN3 | 3 | Driver IN3 | Only active when motor build option is enabled |
| ULN2003 IN4 | 10 | Driver IN4 | Only active when motor build option is enabled |
| GND | GND | All module and supply grounds | Common reference required |

The firmware does not use GPIO18/19 for peripherals because they may serve native USB Serial/JTAG on ESP32-C3. It leaves UART0 GPIO20/21 for the serial console. Recheck board-specific pin routing and boot behavior.

## BZ251 GNSS and compass

| BZ251 six-pin connector label | Connect to |
|---|---|
| TX | ESP32-C3 UART1 RX, default GPIO6 |
| RX | ESP32-C3 UART1 TX, default GPIO7 |
| GND | Common ground |
| VCC | Module supply within the exact module's specified range (documented variant: 3.0–5.5 V) |
| SCL | Shared I2C SCL, default GPIO5 |
| SDA | Shared I2C SDA, default GPIO4 |

The documented module variant specifies 3.3 V digital I/O. Confirm the actual board pull-ups are not tied to 5 V. GNSS defaults to 38400 baud in this project's Kconfig, but it can be changed. The driver accepts NMEA and UBX NAV-PVT; it does not reconfigure the module. Compass default address is 0x0C per the BZ251 manual; 0x0D is a common QMC5883L variant address.

## MPU6050 breakout

| MPU6050 label | Connect to |
|---|---|
| SDA | Shared I2C SDA |
| SCL | Shared I2C SCL |
| GND | Common ground |
| VCC | 3.3 V unless the specific breakout documentation confirms a regulated input |
| AD0 | GND/low for default address 0x68; high selects 0x69 on common MPU6050 implementations |

0x68 is the default driver address. The breakout/AD0 level must be checked. If your module uses 0x69, set MPU6050 I2C address in menuconfig, rebuild, and verify the scan. I2C addresses are 7-bit.

## 28BYJ-48 and ULN2003

Plug the five-wire motor connector into the matching five-pin motor socket on a 28BYJ-48 ULN2003 board. Connect ULN2003 IN1..IN4 to GPIOs listed above, VCC to the separate 5 V motor rail, and GND to common ground. Do not connect the motor's five-wire connector to the A4988.

The firmware uses the four input pins in the order IN1, IN2, IN3, IN4 and an eight-state half-step sequence. If the motor turns in the wrong direction, change Reverse motor rotation in menuconfig. If the physical shaft does not move smoothly, stop and verify the motor/board pairing and sequence; do not assume the estimated position is correct.

## Electrical checks

- Pull SDA/SCL up to 3.3 V, with suitable external pull-ups if absent from the sensor boards. Firmware internal pull-ups are disabled.
- Keep motor current off the ESP32-C3 GPIO and sensor rails.
- Share ground across every connected module and supply.
- Do not apply power until each module's VCC and I/O levels are confirmed.
