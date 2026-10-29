# Hardware compatibility and safety

## Findings from available documentation

The BZGNSS product listing identifies its BZ251 as u-blox M10050 with a QMC5883 compass. The BZ251 manual identifies UART output, I2C compass pins, NMEA 0183 and UBX protocols, and a 38400 baud default for the documented module variant. It lists a 3.0–5.5 V module supply and 3.3 V digital I/O. The same manual records QCM5883_MS_ADDRESS 0x0C, so firmware defaults to 0x0C. The QMC5883L datasheet gives a 7-bit I2C default of 0x0D. This discrepancy may reflect a manual typo or a different board/sensor revision. Firmware exposes the address in Kconfig; inspect board markings and confirm with the I2C scan before selecting it.

The available manual does not establish the physical revision of the user's module, its exact magnetometer die, or whether this unit's pull-ups are present and tied to 3.3 V. The driver is an implemented QMC5883L-compatible register driver, but no board ID register can prove the exact chip: the common QMC5883L ID register returns a fixed value that is not unique. A responding address and plausible measurements are diagnostics, not proof of BOM identity.

The MPU6050 provides 3-axis acceleration and gyroscope measurements; it does not include a magnetometer and cannot provide absolute heading on its own. The compass remains the absolute magnetic reference. The MPU6050 accelerometer supplies gravity direction for tilt compensation; gyro bias calibration supports motion diagnostics but yaw integration is never treated as absolute heading.

The five-wire 28BYJ-48 is a unipolar motor. Allegro's A4988 is designed to drive bipolar steppers using two full-bridge winding outputs. The unmodified five-wire motor's common winding connection is incompatible with that output topology. Use the ULN2003 backend and matching unipolar board. This project does not implement or recommend cutting motor traces or modifying the motor.

## Power and signal checks

- ESP32-C3 GPIO is 3.3 V logic; do not feed 5 V into GPIO.
- BZ251 manual variant: 3.0–5.5 V module VCC and 3.3 V digital I/O. Use an appropriately rated supply and verify the exact module manual/label.
- MPU6050 IC digital/analog rails are 3.3 V class. Breakout boards differ; inspect the exact board regulator and I2C pull-ups before applying 5 V.
- ULN2003 board and 28BYJ-48 motor use a 5 V motor rail for the specified version. The ULN2003 is a low-side transistor array, not a current-regulated stepper driver. Check motor/board winding current and temperature.
- Join grounds between ESP32-C3, GNSS, IMU, driver, and motor supply.
- I2C pull-ups must terminate at 3.3 V. If a breakout pulls the bus to 5 V, remove/alter those pull-ups or use a suitable bidirectional level shifter.
- ESP32-C3 to ULN2003 inputs: 3.3 V logic output; verify the exact driver board recognizes that level.
- Keep magnetometer and GNSS patch antenna away from motor, motor wiring, steel, magnets, speaker magnets, and high-current conductors.

No battery management or power-distribution design is included.

## GPIO selection status

The default pin map is based on the official ESP32-C3-DevKitM-1 reference board. That board model has not been confirmed as the user's board, so these are configuration defaults, not a wiring authorization. Confirm the board silkscreen and schematic, including any on-board USB/JTAG, UART bridge, LEDs, boot straps, and other peripherals. The defaults avoid GPIO2/8/9 strap pins and GPIO18/19 USB Serial/JTAG, but third-party boards can route GPIO differently.

## First-power precautions

1. Keep the motor supply disconnected while checking logic power, grounds, I2C voltage, UART direction, and I2C scan results.
2. Use CONFIG_BC_MOTOR_DRY_RUN=y and leave CONFIG_BC_MOTOR_ENABLE=n.
3. Confirm the pointer can rotate freely and cannot catch cables.
4. Calibrate and validate compass direction away from the motor.
5. Manually align the pointer before motor_zero confirm. This command changes only the firmware's estimate.
6. Arm only after checking output order and motor/driver connections. Connect motor power last.
7. Stop if the driver, motor, regulator, wiring, or board becomes hot or behaves unexpectedly.

References: [BZ251 manual](https://manuals.plus/bzgnss/bz251-gps-module-manual.pdf), [QMC5883L datasheet](https://www.qstcorp.com/upload/pdf/202202/%EF%BC%88%E5%B7%B2%E4%BC%A0%EF%BC%8913-52-04%20QMC5883L%20Datasheet%20Rev.%20A%281%29.pdf), [MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf), [A4988 datasheet](https://www.allegromicro.com/-/media/files/datasheets/a4988-datasheet.pdf), [ESP32-C3-DevKitM-1 pin guide](https://docs.espressif.com/projects/esp-idf/en/v5.0/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html).
