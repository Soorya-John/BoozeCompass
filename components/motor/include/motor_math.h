#pragma once

#ifdef __cplusplus
extern "C" {
#endif

double bc_motor_normalize_angle(double angle_deg);
double bc_motor_shortest_delta(double target_deg, double current_deg);
long bc_motor_angle_delta_steps(double target_deg, double current_deg,
                                long steps_per_revolution, int direction_sign);
double bc_motor_steps_to_angle(long steps, long steps_per_revolution,
                               int direction_sign);

#ifdef __cplusplus
}
#endif
