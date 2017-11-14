#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <vector>

using namespace std;



// --- Bounds ---
// Speed
const double MAX_VELOCITY                   = 49.5;
// Distances
const int DIST_TO_CAR_IN_FRONT              = 30;
const int EMERGENCY_DIST                    = 10;
const int LEFT_LANE_MAX                     = 4;
const int RIGHT_LANE_MIN                    = 8;
const int FRONT_MOST_DISTANCE_SAFETY        = 30;
const int BACK_MOST_DISTANCE_SAFETY         = -60;
const int FRONT_MOST_DISTANCE_RELEVANT      = 100;
const int BACK_MOST_DISTANCE_RELEVANT       = -5;
const int SAFE_TO_CROSS_DISTANCE            = 8;
const int SAFE_FRONT_CHANGE_INTO            = 13;
const int SAFE_FRONT_DISTANCE               = 10;
const int DOUBLE_LANE_CLOSE_DIST            = 30;
// Bean Counters
const int LANE_CHANGE_BEAN_COUNT_UPPER      = 10;
const int SAFETY_CHECK_BEAN_COUNT_UPPER     = 2;

// Choose Speed Costs
const double COLLISION_COST                 = 200.0;
const double MAX_VELOCITY_COST              = 1.0;
const double MATCH_LANE_VELOCTIY_COST       = 2.0;

// Choose Lane Costs
const double LANE_VELOCITY_COST             = 1.0;
const double LANE_VELOCITY_DIFF_COST        = 2.5;
const double LANE_DISTANCE_COST             = 4.0;
const double LANE_NUM_VEHICLES_COST         = 0.5;
const double LANE_CHANGE_COST               = 1.5;
const double LANE_NUM_CLOSE_VEHICLES_COST   = 1.0;

// Constants for cost functions
const double COLLISION_MAX_COST_DISTANCE    = -20.0;
const double COLLISION_MIN_COST_DISTANCE    = 10.0;
const double BUFFER_MAX_COST_DISTANCE       = 5.0;
const double BUFFER_MIN_COST_DISTANCE       = 15.0;
const double SPEED_LIMIT_UPPER              = 50.0;

#endif //__CONSTANTS_H__
