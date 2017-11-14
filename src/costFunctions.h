#ifndef __COSTFUNCTIONS_H__
#define __COSTFUNCTIONS_H__

#include <vector>
#include <iostream>
#include <cmath>
#include "providedFunctions.h"
#include "constants.h"
#include "Vehicle.h"

using namespace std;

double LaneVelocityCost(double speed_of_vehicle)
{
    return 1/(1+exp(-2*((MAX_VELOCITY - speed_of_vehicle)-2)));
}

double LaneVelocityDifferenceCost(double speed_of_vehicle, double speed_current_lane)
{
    return 1/(1+exp(-(speed_current_lane-speed_of_vehicle)));
}

double LaneDistanceCost(double distance_to_vehicle)
{
    return 1/(1+exp(-0.1*((FRONT_MOST_DISTANCE_RELEVANT - distance_to_vehicle)-70)));
    // return 1/(1+exp(-0.05*((FRONT_MOST_DISTANCE_RELEVANT - distance_to_vehicle)-80)));
}

double LaneNumVehiclesCost(double num_cars_in_lane, double total_num_cars)
{
    return num_cars_in_lane/total_num_cars;
}

double LaneNumVehiclesCrossedCost(vector<int> num_close_cars, int new_lane, int current_lane)
{
    if (abs(new_lane-current_lane) > 1)
        return (double)num_close_cars[1];
    return 0;
}

double LaneChangeCost(int new_lane, int current_lane)
{
    return abs((double)current_lane-(double)new_lane)/2;
}

double CollisionCost(const vector<vector<double>>& trajectory, const vector<vector<Vehicle>>& relevant_vehicles,
                        const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y, double& minimum_distance)
{
    // Get the position at the end of the trajectory and see where itll be relative to the other vehicles
    double x_position = trajectory[0][trajectory[0].size()-1];
    double y_position = trajectory[1][trajectory[1].size()-1];

    double x_position_prev = trajectory[0][trajectory[0].size()-2];
    double y_position_prev = trajectory[1][trajectory[1].size()-2];

    double theta = atan2(y_position-y_position_prev , x_position-x_position_prev);

    // Get Frenet, check how close to vehicles in the lane
    vector<double> frenet = getFrenet(x_position, y_position, theta, map_waypoints_x, map_waypoints_y);
    double car_s = frenet[0];
    int lane = GetLane(frenet[1]);

    for  (int i=0; i<relevant_vehicles[lane].size(); i++)
    {
        double check_car_s = relevant_vehicles[lane][i].CalcPositionForward(100);
        // += ((double)100*0.02*check_speed);
        double relative_distance = check_car_s - car_s;
        // cout << "rel dist: " << relative_distance << "| Check_car_s: " << check_car_s << " | check_speed: " << check_speed << " | car_s: " << car_s << endl;
        // cout << "vx: " << vx << " | vy: " << vy << " | x_pos: " << x_position << " | x_pos_prev: " << x_position_prev <<
        //         " | y_pos: " << y_position << " | y_pos_prev: " << y_position_prev << " | theta: " << theta << " | lane" << lane << endl;
        if (relative_distance < minimum_distance)
            minimum_distance = relative_distance;
    }

    double cost = 1/(1+exp((minimum_distance-10)/3));
    return cost;
}

double MaxVelocityCost(double velocity)
{
    return 1/(1+exp(-0.5*((MAX_VELOCITY - velocity)-10)));
}

double MatchLaneChangeVelocityCost(double velocity, double lane_velocity, int lane_change)
{
    if (lane_change > 1)
        lane_change = 1;
    return ((abs(lane_velocity-velocity)/10) * (double)lane_change);
}

#endif //__COSTFUNCTIONS_H__
