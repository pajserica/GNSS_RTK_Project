#include "navigation.h"
#include "imu_module.h"
#include "esp_log.h"
#include "math.h"

static const char *TAG = "NAVIGATION";

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)
#define EARTH_RADIUS_M 6371000.0

double normalize_angle(double angle) {
    while (angle < 0) angle += 360.0;
    while (angle >= 360.0) angle -= 360.0;
    return angle;
}

double angle_difference(double angle1, double angle2) {
    double diff = angle2 - angle1;
    while (diff < -180.0) diff += 360.0;
    while (diff > 180.0) diff -= 360.0;
    return diff;
}

double calculate_distance(const coordinate_t* from, const coordinate_t* to) {
    // Haversine formula for distance calculation
    double lat1_rad = from->latitude * DEG_TO_RAD;
    double lon1_rad = from->longitude * DEG_TO_RAD;
    double lat2_rad = to->latitude * DEG_TO_RAD;
    double lon2_rad = to->longitude * DEG_TO_RAD;
    
    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;
    
    double a = sin(dlat/2) * sin(dlat/2) + 
               cos(lat1_rad) * cos(lat2_rad) * 
               sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return EARTH_RADIUS_M * c;
}

double calculate_bearing(const coordinate_t* from, const coordinate_t* to) {
    // Calculate initial bearing (forward azimuth)
    double lat1_rad = from->latitude * DEG_TO_RAD;
    double lat2_rad = to->latitude * DEG_TO_RAD;
    double dlon_rad = (to->longitude - from->longitude) * DEG_TO_RAD;
    
    double y = sin(dlon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - 
               sin(lat1_rad) * cos(lat2_rad) * cos(dlon_rad);
    
    double bearing_rad = atan2(y, x);
    double bearing_deg = bearing_rad * RAD_TO_DEG;
    
    return normalize_angle(bearing_deg);
}

esp_err_t navigate_to_target(double target_bearing, double distance) {
    double current_heading = get_current_heading();
    double heading_error = angle_difference(current_heading, target_bearing);
    
    ESP_LOGD(TAG, "Current heading: %.1f°, Target bearing: %.1f°, Error: %.1f°", 
             current_heading, target_bearing, heading_error);
    
    // Determine navigation strategy based on heading error and distance
    if (fabs(heading_error) > HEADING_TOLERANCE_DEG) {
        // Need to turn first
        float turn_speed = TURN_SPEED;
        
        // Reduce turn speed for small corrections
        if (fabs(heading_error) < 45.0) {
            turn_speed *= (fabs(heading_error) / 45.0);
        }
        
        if (turn_speed < 0.2) turn_speed = 0.2; // Minimum turn speed
        
        if (heading_error > 0) {
            // Turn right
            ESP_LOGD(TAG, "Turning right at speed %.2f", turn_speed);
            return motor_turn_right(turn_speed);
        } else {
            // Turn left
            ESP_LOGD(TAG, "Turning left at speed %.2f", turn_speed);
            return motor_turn_left(turn_speed);
        }
    } else {
        // Heading is acceptable, move forward
        float forward_speed = MAX_SPEED;
        
        // Reduce speed when close to target
        if (distance < 10.0) {
            forward_speed *= (distance / 10.0);
            if (forward_speed < MIN_SPEED) {
                forward_speed = MIN_SPEED;
            }
        }
        
        // Apply slight steering correction while moving forward
        float steering_correction = 0.0;
        if (fabs(heading_error) > 5.0) {
            steering_correction = heading_error / 90.0; // Normalize to -1.0 to 1.0 range
            if (steering_correction > 0.3) steering_correction = 0.3;
            if (steering_correction < -0.3) steering_correction = -0.3;
        }
        
        ESP_LOGD(TAG, "Moving forward at speed %.2f with steering %.2f", 
                 forward_speed, steering_correction);
        
        return motor_differential_drive(forward_speed, steering_correction);
    }
}

esp_err_t execute_navigation_step(const coordinate_t* current, const coordinate_t* target, double current_heading) {
    double distance = calculate_distance(current, target);
    double bearing = calculate_bearing(current, target);
    
    ESP_LOGD(TAG, "Navigation step - Distance: %.2fm, Bearing: %.1f°, Heading: %.1f°", 
             distance, bearing, current_heading);
    
    if (distance < WAYPOINT_TOLERANCE_M) {
        // Target reached
        motor_stop();
        ESP_LOGI(TAG, "Target reached within tolerance (%.2fm)", distance);
        return ESP_OK;
    }
    
    return navigate_to_target(bearing, distance);
}