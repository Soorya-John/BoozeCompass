# Testing and validation

## Host tests

From the repository root in PowerShell:

    powershell.exe -ExecutionPolicy Bypass -NoProfile -File .\tools\run_host_tests.ps1

The host harness compiles the actual portable C navigation and motor-math components with MinGW GCC and runs CTest. Coverage includes:

- Haversine distance, antimeridian behavior, initial bearing, and invalid coordinates.
- Angle normalization, signed shortest difference, and wraparound.
- Relative bearing, east-positive declination, pointer-zero sign, stale position/heading/target, and destination proximity.
- Nearest store selection, duplicate removal, missing-name preservation, and invalid input.
- Motor angle-to-step rounding, direction, output-angle conversion, and wraparound.

## Firmware build

With ESP-IDF v5.2.8 activated:

    idf.py set-target esp32c3
    idf.py build

No physical flash or monitor test is implied by a successful build.

## Hardware tests still required

- Verify the exact ESP32-C3 board/revision, GPIO headers, boot behavior, USB/JTAG, and console port.
- Verify BZ251 board revision, UART baud, NMEA/UBX output, compass chip, I2C address, and pull-up voltage.
- Verify the MPU6050 breakout supply, AD0 address, axis orientation, and I2C electrical levels.
- Confirm sensor values and magnetic calibration away from the stepper.
- Validate heading/declination against known directions.
- Measure 28BYJ-48 output steps/revolution and confirm ULN2003 phase order/direction.
- Confirm hotspot connection and certificate-verified Overpass requests on the device.
- Test stale GPS, Wi-Fi loss, empty Overpass responses, API timeout/rate-limit response, and sensor disconnect/reconnect.
- Confirm motor remains disabled before manual zero/arm and remains still when sensor data is invalid.

Never describe these as passed until hardware is connected and the tests are actually performed.
