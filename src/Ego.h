#ifndef __EGO_H__
#define __EGO_H__

#include <vector>

enum vehicle_state
{
    keep_lane = 0,
    prepare_lane_change,
    lane_change
    // prepare_lane_change_left,
    // prepare_lane_change_right,
    // lane_change_left,
    // lane_change_right
};

class Ego
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double velocity;

    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;

    vehicle_state current_state = keep_lane;
};



#endif //__EGO_H__
