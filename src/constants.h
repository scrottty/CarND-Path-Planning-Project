#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <vector>

using namespace std;

const double MAX_VELOCITY                   = 49.5;
const int DIST_TO_CAR_IN_FRONT              = 30;
const int DIST_FOR_SLOW_DOWN                = 20;
const int EMERGENCY_DIST                    = 5;
const int LEFT_LANE_MAX                     = 4;
const int RIGHT_LANE_MIN                    = 8;
const int FRONT_MOST_DISTANCE_SAFETY        = 30;
const int BACK_MOST_DISTANCE_SAFETY         = -60;
const int FRONT_MOST_DISTANCE_RELEVANT      = 100;
const int BACK_MOST_DISTANCE_RELEVANT       = -5;
const int SAFE_TO_CROSS_DISTANCE            = 5;
const int SAFE_FRONT_CHANGE_INTO            = 10;
const int SAFE_FRONT_DISTANCE               = 5;
const int DOUBLE_LANE_CLOSE_DIST            = 30;
const int MIN_GAP_SIZE                      = 5;
const int LANE_CHANGE_BEAN_COUNT_UPPER      = 10;
const int SAFETY_CHECK_BEAN_COUNT_UPPER     = 2;

const double DOUBLE_LANE_CHANGE_ADJUST      = 0.8;

// Costs
const double COLLISION_COST                 = 200.0;
const double MAX_VELOCITY_COST              = 1.0;
const double VEL_DIST_COMBO_COST            = 2.0;
const double MATCH_LANE_VELOCTIY_COST       = 0.5;
// const double BUFFER_COST                    = 2.0;
// const double DESIRED_LANE_COST              = 10.0;
// const double CURRENT_LANE_COST              = 5.0;
// const double SPEED_TRAFFIC_COST             = 1.0;
// const double SPEED_LIMIT_COST               = 100.0;
// const double SPEED_FAST_COST                = 5.0;
// const double SPEED_DISTANCE_COST            = 5.0;
// const double NUM_CARS_COST                  = 1.0;
// const double LANE_SPEED_COST                = 2.0;

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
