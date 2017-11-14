#ifndef __EGO_H__
#define __EGO_H__

#include <vector>

enum vehicle_state
{
    keep_lane = 0,
    prepare_lane_change,
    lane_change
};

class Ego
{
public:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double velocity;
    double current_s;

    vehicle_state current_state = keep_lane;
    std::vector<std::vector<double>> choosen_trajectory;
};



#endif //__EGO_H__
