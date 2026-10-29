#include "motor_math.h"
#include "navigation.h"

#include <math.h>

double bc_motor_normalize_angle(double angle_deg)
{
    return bc_normalize_360(angle_deg);
}

double bc_motor_shortest_delta(double target_deg, double current_deg)
{
    return bc_signed_angle_difference(target_deg, current_deg);
}

long bc_motor_angle_delta_steps(double target_deg, double current_deg,
                                long steps_per_revolution, int direction_sign)
{
    if (!isfinite(target_deg) || !isfinite(current_deg) ||
        steps_per_revolution <= 0 || (direction_sign != 1 && direction_sign != -1)) {
        return 0;
    }
    const double signed_steps =
        bc_motor_shortest_delta(target_deg, current_deg) *
        (double)steps_per_revolution / 360.0 * (double)direction_sign;
    return signed_steps >= 0.0 ? (long)floor(signed_steps + 0.5)
                               : (long)ceil(signed_steps - 0.5);
}

double bc_motor_steps_to_angle(long steps, long steps_per_revolution,
                               int direction_sign)
{
    if (steps_per_revolution <= 0 || (direction_sign != 1 && direction_sign != -1)) {
        return NAN;
    }
    return bc_motor_normalize_angle((double)steps * 360.0 /
                                    (double)steps_per_revolution *
                                    (double)direction_sign);
}
