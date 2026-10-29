# Calibration

## Magnetometer calibration

The QMC5883 driver reads raw X/Y/Z magnetic counts. calibrate_mag start begins a 3-axis min/max collection; slowly rotate and tilt the device through multiple orientations for 30–60 seconds, then use calibrate_mag finish. The code estimates hard-iron offsets as each axis midpoint and diagonal scale from the average half-range. It saves the result in NVS.

This is a practical min/max calibration, not a full ellipsoid fit. Use it away from the stepper, motor wiring, steel, magnets, and current-carrying cables. If a calibration axis spans too little field or the resulting scale is implausible, the routine rejects it. Validate heading against a known reference at several orientations after installation.

The default QMC register setup is continuous measurement at 50 Hz, ±8 gauss, 512x oversampling. The QMC5883L has no unique ID value that certifies the board's actual chip. The firmware's I2C scan and raw readings are diagnostics.

## Tilt and mounting

The MPU6050 supplies the gravity vector used for roll/pitch compensation. Firmware assumes the MPU6050 and magnetometer axes are mounted consistently, with the device's forward axis represented by the expected horizontal sensor axis. Sensor mounting yaw correction adjusts compass mounting rotation in the horizontal plane. Validate this mapping physically; the source module's orientation is not known.

calibrate_gyro averages stationary gyro readings to estimate angular-rate bias. Keep the device motionless for the half-second collection. This correction does not produce absolute heading. The MPU6050 gyro is never integrated as the compass reference.

## Declination and pointer zero

Magnetic declination is configured in degrees with east positive. Firmware converts magnetic heading to true heading exactly once:

    true_heading = normalize_360(magnetic_heading + east_positive_declination)

The pointer command is:

    relative = normalize_360(true_target_bearing - true_heading + pointer_zero_offset)

Pointer-zero offset is positive clockwise. If the pointer's calibrated zero position is 4 degrees clockwise from the sensor/device forward axis, configure +4. Adjust only after confirming compass alignment and motor direction.

## Motor zero and steps/revolution

There is no homing switch, encoder, or shaft feedback. After every power-up, the physical pointer position is unknown even if the firmware has a step estimate. Manually align the pointer with the device forward axis, then enter motor_zero confirm. This does not physically move the motor or prove alignment.

4096 half-steps/output revolution is an initial configurable estimate, not a guaranteed gearbox ratio. Mark the pointer and measure one full output revolution at low speed; correct the setting for your motor/gearbox. Configure direction and pointer offset only after motor zero is repeatable.

Use dry-run first. Then enable hardware outputs in menuconfig and verify low-speed motion with the motor free to rotate. Missed steps, backlash, slippage, and external pointer movement are not detected.
