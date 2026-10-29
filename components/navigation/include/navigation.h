#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BC_STORE_NAME_MAX 64
#define BC_EARTH_RADIUS_M 6371008.8

typedef struct {
    double latitude_deg;
    double longitude_deg;
    int64_t observed_ms;
    bool valid;
    unsigned satellites;
    unsigned fix_quality;
    double altitude_m;
    double speed_mps;
} bc_position_t;

typedef struct {
    double latitude_deg;
    double longitude_deg;
    uint64_t osm_id;
    int64_t observed_ms;
    char name[BC_STORE_NAME_MAX];
} bc_store_t;

typedef struct {
    double magnetic_heading_deg;
    int64_t observed_ms;
    bool valid;
    bool calibrated;
} bc_heading_t;

typedef struct {
    double distance_m;
    double true_bearing_deg;
    double relative_pointer_deg;
} bc_navigation_solution_t;

typedef enum {
    BC_NAV_OK = 0,
    BC_NAV_INVALID_POSITION,
    BC_NAV_STALE_POSITION,
    BC_NAV_INVALID_HEADING,
    BC_NAV_STALE_HEADING,
    BC_NAV_INVALID_TARGET,
    BC_NAV_STALE_TARGET,
    BC_NAV_AT_DESTINATION
} bc_nav_status_t;

double bc_normalize_360(double degrees);
double bc_signed_angle_difference(double target_deg, double current_deg);
bool bc_coordinates_valid(double latitude_deg, double longitude_deg);
double bc_haversine_distance_m(double lat1_deg, double lon1_deg,
                               double lat2_deg, double lon2_deg);
double bc_initial_bearing_deg(double lat1_deg, double lon1_deg,
                              double lat2_deg, double lon2_deg);
bc_nav_status_t bc_navigation_solve(const bc_position_t *position,
                                    const bc_heading_t *heading,
                                    const bc_store_t *target,
                                    int64_t now_ms,
                                    int64_t max_position_age_ms,
                                    int64_t max_heading_age_ms,
                                    int64_t max_target_age_ms,
                                    double east_positive_declination_deg,
                                    double pointer_zero_offset_deg,
                                    bc_navigation_solution_t *solution);
bool bc_store_select_nearest(const bc_store_t *stores, size_t count,
                             double latitude_deg, double longitude_deg,
                             bc_store_t *selected, double *distance_m);
size_t bc_store_deduplicate(bc_store_t *stores, size_t count,
                            double duplicate_distance_m);

#ifdef __cplusplus
}
#endif
