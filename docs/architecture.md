# Architecture and runtime states

## Component flow

main owns the periodic control loop and explicit state machine. Hardware-independent navigation and motor-angle math live in separate, host-testable components.

- gps: UART1 input; byte-stream NMEA checksum/parser and UBX NAV-PVT parser.
- compass: QMC5883 register access, raw data, tilt-compensated magnetic heading, and min/max calibration.
- mpu6050: I2C detection/configuration, accel/gyro sampling, low-pass filtering, tilt, and stationary gyro-bias estimate.
- navigation: spherical distance, initial bearing, angle wrap, input freshness, candidate de-duplication, and nearest selection.
- wifi_manager: station association, IP event state, SNTP time sync for TLS certificate checks, and exponential reconnect delay.
- store_finder: queued HTTPS request on its own task, bounded response buffer, JSON parsing, and result queue.
- motor: logical absolute-angle interface and separate ULN2003 half-step backend task.
- configuration: local Kconfig defaults plus NVS magnetometer calibration.

Network operations run outside the sensor/control loop. A timeout or API outage therefore does not stop UART parsing, sensor sampling, or Wi-Fi reconnection. The motor has a separate task and latest-target queue.

The Wi-Fi IP event starts SNTP. The Overpass worker waits for a valid system clock before opening HTTPS, so certificate dates are checked against synchronized time. If NTP cannot sync, the search fails safely and retries later.

## State transitions

| State | Entry condition | Exit condition / action |
|---|---|---|
| BOOT | Reset | Initialize NVS, event loop, and runtime configuration |
| HARDWARE_INIT | Startup | Create I2C bus, start UART, drivers, Wi-Fi, search, and motor interface |
| SENSOR_DIAGNOSTICS | Initializing complete | Scan I2C and publish startup status |
| ERROR_RECOVERY | I2C, GNSS UART, Wi-Fi manager, or search task unavailable | Retry sensor init every 30 s where possible; app loop remains alive |
| DEGRADED_MODE | IMU/compass unavailable, stale heading, or cached target expired | Inhibit new motor updates; retain explicit stale diagnostics |
| CALIBRATING_HEADING | Sensors have fresh data but no accepted magnetometer calibration | Run a user-started calibration and save result in NVS |
| WAITING_FOR_GPS | Sensor readings are valid but position is missing/stale | Wait for fresh, valid coordinates; do not search or move |
| CONNECTING_WIFI | Fresh position/calibrated heading but no IP connection | Wi-Fi task retries with exponential backoff; no HTTP request while offline |
| SEARCHING_FOR_TARGET | Fresh fix/network and search is due, pending, or no candidate exists | Queued HTTPS query; empty result clears target; API error retains cache only until stale |
| TRACKING_TARGET | Fresh GPS, calibrated fresh heading, connected network, and current mapped candidate | Compute distance/bearing; pass filtered relative angle to motor only if explicitly zeroed and armed |

Any loss of GPS, heading, compass calibration, or target freshness prevents automatic pointer updates. An API outage never upgrades the last cached candidate into a current result.

## Navigation conventions

- Geographic bearings are true bearings, measured clockwise from true north.
- The compass supplies magnetic heading. East-positive declination is added once: true_heading = magnetic_heading + declination.
- Relative angle is normalize_360(target_true_bearing - device_true_heading + pointer_zero_offset).
- Positive pointer-zero correction is clockwise from the sensor/device forward axis to the pointer's calibrated zero axis.
- The motor is open-loop. Reported position is a step-count estimate, not a measured shaft angle.
- A circular low-pass filter and configurable angular deadband suppress small heading/GPS jitter. They do not correct magnetic interference, local map errors, or lost motor steps.

## Data retention

Candidate results are cached in RAM only. No coordinate history or destination is persisted. Wi-Fi settings are local Kconfig values; magnetometer calibration is stored in NVS.
