#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "gnss_rover.h"
#include "motor_control.h"

// Navigation configuration
#define WAYPOINT_TOLERANCE_M 2.0      // Distance tolerance to consider waypoint reached
#define MAX_SPEED 0.7                 // Maximum motor speed (0.0 to 1.0)
#define MIN_SPEED 0.3                 // Minimum motor speed for movement
#define TURN_SPEED 0.5                // Speed for turning maneuvers
#define HEADING_TOLERANCE_DEG 15.0    // Heading tolerance in degrees

// Navigation commands
typedef enum {
    NAV_CMD_START,
    NAV_CMD_STOP,
    NAV_CMD_NEXT_WAYPOINT,
    NAV_CMD_PAUSE,
    NAV_CMD_RESUME
} navigation_command_type_t;

typedef struct {
    navigation_command_type_t command;
    float param1;  // Optional parameter
    float param2;  // Optional parameter
} navigation_command_t;

// Function declarations
double calculate_distance(const coordinate_t* from, const coordinate_t* to);
double calculate_bearing(const coordinate_t* from, const coordinate_t* to);
esp_err_t navigate_to_target(double target_bearing, double distance);
double normalize_angle(double angle);
double angle_difference(double angle1, double angle2);
esp_err_t execute_navigation_step(const coordinate_t* current, const coordinate_t* target, double current_heading);

#endif // NAVIGATION_H