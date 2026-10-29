#include "motor_math.h"
#include "navigation.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned tests_run;

#define CHECK(condition) do { \
    ++tests_run; \
    if (!(condition)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static int near(double actual, double expected, double tolerance)
{
    return isfinite(actual) && fabs(actual - expected) <= tolerance;
}

int main(void)
{
    CHECK(near(bc_haversine_distance_m(0, 0, 0, 1), 111195.08, 2.0));
    CHECK(near(bc_haversine_distance_m(0, 179.9, 0, -179.9), 22239.0, 30.0));
    CHECK(isnan(bc_haversine_distance_m(91, 0, 0, 0)));
    CHECK(near(bc_initial_bearing_deg(0, 0, 1, 0), 0, 1e-9));
    CHECK(near(bc_initial_bearing_deg(0, 0, 0, 1), 90, 1e-9));
    CHECK(near(bc_initial_bearing_deg(0, 0, -1, 0), 180, 1e-9));
    CHECK(near(bc_initial_bearing_deg(0, 0, 0, -1), 270, 1e-9));
    CHECK(near(bc_normalize_360(-10), 350, 1e-9));
    CHECK(near(bc_normalize_360(720), 0, 1e-9));
    CHECK(near(bc_signed_angle_difference(1, 359), 2, 1e-9));
    CHECK(near(bc_signed_angle_difference(359, 1), -2, 1e-9));
    CHECK(near(bc_signed_angle_difference(180, 0), -180, 1e-9));
    CHECK(!bc_coordinates_valid(90.01, 0));
    CHECK(!bc_coordinates_valid(0, -180.01));

    bc_position_t position = {
        .latitude_deg = 0, .longitude_deg = 0, .observed_ms = 9000, .valid = true
    };
    bc_heading_t heading = {
        .magnetic_heading_deg = 350, .observed_ms = 9900,
        .valid = true, .calibrated = true
    };
    bc_store_t target = {
        .latitude_deg = 0, .longitude_deg = 1, .osm_id = 1,
        .observed_ms = 9900, .name = "East"
    };
    bc_navigation_solution_t solution;
    CHECK(bc_navigation_solve(&position, &heading, &target, 10000,
                              2000, 1000, 20000, 10, 5, &solution) == BC_NAV_OK);
    CHECK(near(solution.true_bearing_deg, 90, 1e-9));
    CHECK(near(solution.relative_pointer_deg, 95, 1e-9));
    heading.observed_ms = 100;
    CHECK(bc_navigation_solve(&position, &heading, &target, 10000,
                              2000, 1000, 20000, 0, 0, &solution) ==
          BC_NAV_STALE_HEADING);
    heading.observed_ms = 9900;
    position.observed_ms = 1;
    CHECK(bc_navigation_solve(&position, &heading, &target, 10000,
                              2000, 1000, 20000, 0, 0, &solution) ==
          BC_NAV_STALE_POSITION);
    position.observed_ms = 9000;
    target.observed_ms = 1;
    CHECK(bc_navigation_solve(&position, &heading, &target, 10000,
                              2000, 1000, 5000, 0, 0, &solution) ==
          BC_NAV_STALE_TARGET);
    target.observed_ms = 9900;
    target.latitude_deg = 0;
    target.longitude_deg = 0;
    CHECK(bc_navigation_solve(&position, &heading, &target, 10000,
                              2000, 1000, 20000, 0, 0, &solution) ==
          BC_NAV_AT_DESTINATION);

    bc_store_t stores[3] = {
        {.latitude_deg = 0, .longitude_deg = 1, .osm_id = 1, .name = "far"},
        {.latitude_deg = 0, .longitude_deg = 0.1, .osm_id = 2, .name = ""},
        {.latitude_deg = 0, .longitude_deg = 0.10001, .osm_id = 3, .name = "near duplicate"}
    };
    size_t unique = bc_store_deduplicate(stores, 3, 5.0);
    CHECK(unique == 2);
    CHECK(strcmp(stores[0].name, "far") == 0);
    CHECK(strcmp(stores[1].name, "near duplicate") == 0);
    bc_store_t selected;
    double distance;
    CHECK(bc_store_select_nearest(stores, unique, 0, 0, &selected, &distance));
    CHECK(selected.osm_id == 2);
    CHECK(distance > 11000 && distance < 11200);
    CHECK(!bc_store_select_nearest(NULL, 0, 0, 0, &selected, &distance));

    CHECK(bc_motor_angle_delta_steps(2, 358, 4096, 1) == 46);
    CHECK(bc_motor_angle_delta_steps(358, 2, 4096, 1) == -46);
    CHECK(bc_motor_angle_delta_steps(90, 0, 200, -1) == -50);
    CHECK(near(bc_motor_steps_to_angle(2048, 4096, 1), 180, 1e-9));
    CHECK(near(bc_motor_steps_to_angle(-1, 4096, 1), 359.912109375, 1e-9));

    printf("PASS: %u navigation and motor-math assertions\n", tests_run);
    return 0;
}
