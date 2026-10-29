#include "navigation.h"

#include <math.h>
#include <string.h>

#define BC_DEG_TO_RAD (M_PI / 180.0)
#define BC_RAD_TO_DEG (180.0 / M_PI)

double bc_normalize_360(double degrees)
{
    if (!isfinite(degrees)) {
        return NAN;
    }
    double result = fmod(degrees, 360.0);
    if (result < 0.0) {
        result += 360.0;
    }
    return result >= 360.0 ? 0.0 : result;
}

double bc_signed_angle_difference(double target_deg, double current_deg)
{
    return bc_normalize_360(target_deg - current_deg + 180.0) - 180.0;
}

bool bc_coordinates_valid(double latitude_deg, double longitude_deg)
{
    return isfinite(latitude_deg) && isfinite(longitude_deg) &&
           latitude_deg >= -90.0 && latitude_deg <= 90.0 &&
           longitude_deg >= -180.0 && longitude_deg <= 180.0;
}

double bc_haversine_distance_m(double lat1_deg, double lon1_deg,
                               double lat2_deg, double lon2_deg)
{
    if (!bc_coordinates_valid(lat1_deg, lon1_deg) ||
        !bc_coordinates_valid(lat2_deg, lon2_deg)) {
        return NAN;
    }
    const double lat1 = lat1_deg * BC_DEG_TO_RAD;
    const double lat2 = lat2_deg * BC_DEG_TO_RAD;
    const double dlat = (lat2_deg - lat1_deg) * BC_DEG_TO_RAD;
    const double dlon = (lon2_deg - lon1_deg) * BC_DEG_TO_RAD;
    const double sin_lat = sin(dlat * 0.5);
    const double sin_lon = sin(dlon * 0.5);
    const double a = sin_lat * sin_lat +
                     cos(lat1) * cos(lat2) * sin_lon * sin_lon;
    const double bounded_a = fmin(1.0, fmax(0.0, a));
    return BC_EARTH_RADIUS_M * 2.0 *
           atan2(sqrt(bounded_a), sqrt(1.0 - bounded_a));
}

double bc_initial_bearing_deg(double lat1_deg, double lon1_deg,
                              double lat2_deg, double lon2_deg)
{
    if (!bc_coordinates_valid(lat1_deg, lon1_deg) ||
        !bc_coordinates_valid(lat2_deg, lon2_deg)) {
        return NAN;
    }
    const double lat1 = lat1_deg * BC_DEG_TO_RAD;
    const double lat2 = lat2_deg * BC_DEG_TO_RAD;
    const double dlon = (lon2_deg - lon1_deg) * BC_DEG_TO_RAD;
    const double y = sin(dlon) * cos(lat2);
    const double x = cos(lat1) * sin(lat2) -
                     sin(lat1) * cos(lat2) * cos(dlon);
    return bc_normalize_360(atan2(y, x) * BC_RAD_TO_DEG);
}

static bool age_is_valid(int64_t observed_ms, int64_t now_ms,
                         int64_t max_age_ms)
{
    return observed_ms >= 0 && now_ms >= observed_ms && max_age_ms >= 0 &&
           now_ms - observed_ms <= max_age_ms;
}

bc_nav_status_t bc_navigation_solve(const bc_position_t *position,
                                    const bc_heading_t *heading,
                                    const bc_store_t *target,
                                    int64_t now_ms,
                                    int64_t max_position_age_ms,
                                    int64_t max_heading_age_ms,
                                    int64_t max_target_age_ms,
                                    double east_positive_declination_deg,
                                    double pointer_zero_offset_deg,
                                    bc_navigation_solution_t *solution)
{
    if (position == NULL || heading == NULL || target == NULL ||
        solution == NULL || !position->valid ||
        !bc_coordinates_valid(position->latitude_deg, position->longitude_deg)) {
        return BC_NAV_INVALID_POSITION;
    }
    if (!age_is_valid(position->observed_ms, now_ms, max_position_age_ms)) {
        return BC_NAV_STALE_POSITION;
    }
    if (!heading->valid || !isfinite(heading->magnetic_heading_deg)) {
        return BC_NAV_INVALID_HEADING;
    }
    if (!age_is_valid(heading->observed_ms, now_ms, max_heading_age_ms)) {
        return BC_NAV_STALE_HEADING;
    }
    if (!bc_coordinates_valid(target->latitude_deg, target->longitude_deg)) {
        return BC_NAV_INVALID_TARGET;
    }
    if (!age_is_valid(target->observed_ms, now_ms, max_target_age_ms)) {
        return BC_NAV_STALE_TARGET;
    }

    solution->distance_m = bc_haversine_distance_m(
        position->latitude_deg, position->longitude_deg,
        target->latitude_deg, target->longitude_deg);
    if (!isfinite(solution->distance_m)) {
        return BC_NAV_INVALID_TARGET;
    }
    if (solution->distance_m < 0.5) {
        return BC_NAV_AT_DESTINATION;
    }

    solution->true_bearing_deg = bc_initial_bearing_deg(
        position->latitude_deg, position->longitude_deg,
        target->latitude_deg, target->longitude_deg);
    const double true_heading = bc_normalize_360(
        heading->magnetic_heading_deg + east_positive_declination_deg);
    solution->relative_pointer_deg = bc_normalize_360(
        solution->true_bearing_deg - true_heading + pointer_zero_offset_deg);
    return BC_NAV_OK;
}

size_t bc_store_deduplicate(bc_store_t *stores, size_t count,
                            double duplicate_distance_m)
{
    if (stores == NULL || !isfinite(duplicate_distance_m) ||
        duplicate_distance_m < 0.0) {
        return 0;
    }
    size_t kept = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!bc_coordinates_valid(stores[i].latitude_deg,
                                  stores[i].longitude_deg)) {
            continue;
        }
        bool duplicate = false;
        for (size_t j = 0; j < kept; ++j) {
            if ((stores[i].osm_id != 0 && stores[i].osm_id == stores[j].osm_id) ||
                bc_haversine_distance_m(stores[i].latitude_deg,
                                        stores[i].longitude_deg,
                                        stores[j].latitude_deg,
                                        stores[j].longitude_deg) <=
                    duplicate_distance_m) {
                if (stores[j].name[0] == '\0' && stores[i].name[0] != '\0') {
                    memcpy(stores[j].name, stores[i].name, sizeof(stores[j].name));
                    stores[j].name[sizeof(stores[j].name) - 1] = '\0';
                }
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            if (kept != i) {
                stores[kept] = stores[i];
            }
            stores[kept].name[sizeof(stores[kept].name) - 1] = '\0';
            ++kept;
        }
    }
    return kept;
}

bool bc_store_select_nearest(const bc_store_t *stores, size_t count,
                             double latitude_deg, double longitude_deg,
                             bc_store_t *selected, double *distance_m)
{
    if (stores == NULL || selected == NULL ||
        !bc_coordinates_valid(latitude_deg, longitude_deg)) {
        return false;
    }
    double nearest = INFINITY;
    size_t nearest_index = count;
    for (size_t i = 0; i < count; ++i) {
        const double distance = bc_haversine_distance_m(
            latitude_deg, longitude_deg,
            stores[i].latitude_deg, stores[i].longitude_deg);
        if (isfinite(distance) && distance < nearest) {
            nearest = distance;
            nearest_index = i;
        }
    }
    if (nearest_index == count) {
        return false;
    }
    *selected = stores[nearest_index];
    if (distance_m != NULL) {
        *distance_m = nearest;
    }
    return true;
}
