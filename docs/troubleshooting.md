# Troubleshooting

| Symptom | Checks |
|---|---|
| No I2C devices | Check SDA/SCL pins, common ground, 3.3 V pull-ups, module power, and board GPIO routing. |
| MPU at 0x69, not 0x68 | AD0 is high; set MPU6050 I2C address in menuconfig and rebuild. |
| Compass scan shows 0x0C but firmware expects 0x0D | The available BZ251 manual and QMC5883L datasheet disagree. Set Compass I2C address in menuconfig and rebuild; verify raw values before accepting compass identity. |
| Compass answers but heading is wrong | Check sensor axes/mounting, hard/soft-iron calibration, pointer correction, local magnetic field, and declination. The raw QMC ID value is not a unique identity proof. |
| Heading unavailable when tilted | Check MPU6050 reads and accelerometer axis orientation. Tilt compensation requires a gravity magnitude near 1 g. |
| GNSS never fixes | Check TX to UART RX, common ground, module power, baud setting, antenna sky view, and complete NMEA/UBX output. Indoor GNSS lock may fail. |
| GPS data is stale | Confirm UART bytes are arriving and checksum/protocol output is valid. Firmware parses NMEA GGA/RMC and UBX NAV-PVT; it does not configure the GNSS receiver. |
| Wi-Fi remains disconnected | Check hotspot 2.4 GHz compatibility, local SSID/password in menuconfig, WPA settings, and IP assignment. Credentials are not printed. |
| TLS/Overpass fails before a request | Check hotspot internet access and SNTP time synchronization. Certificate validation stays enabled and search waits for a valid system clock. |
| Overpass search fails or returns no stores | Confirm Wi-Fi and GPS; check hotspot internet access, SNTP time sync for TLS, endpoint HTTP status, radius, and mapped OSM coverage. The API is public and can be overloaded. Empty results clear the active destination. |
| Search command does not immediately query | Public Overpass requests are limited to at most one every 15 minutes by this firmware, even when manual refresh is requested. |
| Motor does not move | Dry-run is default. Check build options, explicit motor_zero confirm, motor_arm, ULN2003 power/ground, and GPIO mapping. |
| Motor turns opposite direction or pointer estimate drifts | Adjust reverse/direction and calibrated steps/revolution. There is no encoder or homing switch; missed steps cannot be detected. |
| Motor or driver heats | Disconnect motor power and verify 5 V motor, winding, ULN2003 board, phase sequence, and mechanical load. The ULN2003 backend does not provide adjustable current regulation. |

Use status, scan_i2c, gps, compass, and imu to collect diagnostics. Do not share logs if they include location coordinates.
