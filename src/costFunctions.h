#ifndef __COSTFUNCTIONS_H__
#define __COSTFUNCTIONS_H__

#include <vector>
#include <iostream>
#include "providedFunctions.h"
#include "constants.h"

using namespace std;

double LaneVelocityCost(double speed_of_vehicle)
{
    return 1/(1+exp(-2*((MAX_VELOCITY - speed_of_vehicle)-2)));
}

double LaneDistanceCost(double distance_to_vehicle)
{
    return 1/(1+exp(-0.05*((FRONT_MOST_DISTANCE_RELEVANT - distance_to_vehicle)-50)));
}

double LaneNumVehiclesCost(double num_cars_in_lane, double total_num_cars)
{
    return num_cars_in_lane/total_num_cars;
}

double LaneChangeCost(int new_lane, int current_lane)
{
    return abs((double)current_lane-(double)new_lane)/2;
}

// double AvoidCollision(vector<vector<double>> trajectory, vector<vector<vector<double>>> relevant_vehicles,
//                         const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
// {
//     double cost = 0.0;
//     for (int i=1; i<trajectory[0].size(); i++) // Step through each coordinate
//     {
//         double x_position = trajectory[0][i];
//         double y_position = trajectory[1][i];
//
//         double x_position_prev = trajectory[0][i-1];
//         double y_position_prev = trajectory[1][i-1];
//
//         double theta = atan2(y_position-y_position_prev , x_position-x_position_prev);
//
//         // Get Frenet, if in same lane, check how close
//         vector<double> frenet = getFrenet(x_position, y_position, theta, map_waypoints_x, map_waypoints_y);
//         double lane = GetLane(frenet[1]);
//         // cout << " S: " << frenet[0] << endl;
//
//         // Loop through the other vehicles in the same lane
//         for (int j=0; j<relevant_vehicles[lane].size(); j++)
//         {
//             // CAN USE THIS TO CALC THEIR POSITION IN THE FUTURE
//             double vx = relevant_vehicles[lane][j][3];
//             double vy = relevant_vehicles[lane][j][4];
//             double check_speed = sqrt(vx*vx+vy*vy);
//             double check_car_s = relevant_vehicles[lane][j][5];
//
//             check_car_s += ((double)i*0.02*check_speed);
//
//             double relative_distance = check_car_s - frenet[0];
//             if (relative_distance <= COLLISION_MAX_COST_DISTANCE)
//             {
//                 cost = 1.0;
//             }
//             else if (relative_distance < COLLISION_MIN_COST_DISTANCE)
//             {
//                 double new_cost = -(relative_distance-COLLISION_MAX_COST_DISTANCE)/(COLLISION_MIN_COST_DISTANCE-COLLISION_MAX_COST_DISTANCE) + 1;
//                 if (new_cost > cost)
//                     cost = new_cost;
//             }
//         }
//     }
//     return cost;
// }
//
// double DistanceBuffer(vector<vector<double>> trajectory, vector<vector<vector<double>>> relevant_vehicles,
//                         const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
// {
//     double cost = 0.0;
//     for (int i=1; i<trajectory[0].size(); i++) // Step through each coordinate
//     {
//         double x_position = trajectory[0][i];
//         double y_position = trajectory[1][i];
//
//         double x_position_prev = trajectory[0][i-1];
//         double y_position_prev = trajectory[1][i-1];
//
//         double theta = atan2(y_position-y_position_prev , x_position-x_position_prev);
//
//         // Get Frenet, if in same lane, check how close
//         vector<double> frenet = getFrenet(x_position, y_position, theta, map_waypoints_x, map_waypoints_y);
//         double lane = GetLane(frenet[1]);
//
//         // Loop through the other vehicles in the same lane
//         for (int j=0; j<relevant_vehicles[lane].size(); j++)
//         {
//             // CAN USE THIS TO CALC THEIR POSITION IN THE FUTURE
//             double vx = relevant_vehicles[lane][j][3];
//             double vy = relevant_vehicles[lane][j][4];
//             double check_speed = sqrt(vx*vx+vy*vy);
//             double check_car_s = relevant_vehicles[lane][j][5];
//
//             check_car_s += ((double)i*0.02*check_speed);
//
//             double relative_distance = check_car_s - frenet[0];
//             if (relative_distance <= BUFFER_MAX_COST_DISTANCE)
//             {
//                 cost = 1.0;
//             }
//             else if (relative_distance < BUFFER_MIN_COST_DISTANCE)
//             {
//                 double new_cost = -(relative_distance-BUFFER_MAX_COST_DISTANCE)/(BUFFER_MIN_COST_DISTANCE-BUFFER_MAX_COST_DISTANCE) + 1;
//                 if (new_cost > cost)
//                     cost = new_cost;
//             }
//         }
//     }
//     return cost;
// }
//
// double DesiredLane(vector<int> lanes, int desired_lane)
// {
//     int lane = lanes[2];
//
//     return lane == desired_lane ? 0.0 : 1.0;
// }
//
// double CurrentLane(vector<int> lanes, int current_lane)
// {
//     int lane = lanes[0];
//
//     return lane == current_lane ? 0.0 : 1.0;
// }
//
// double SpeedTraffic(vector<vector<double>> trajectory, vector<double> mean_lane_speeds,
//                     const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
// {
//     // Cost will be the max, maybe should be a mean???????
//     double cost = 0.0;
//     // Step through the trajectory and check the speed, compare to mean lane speed
//     for (int i=1; i<trajectory[0].size(); i++) // Step through each coordinate
//     {
//         double x_position = trajectory[0][i];
//         double y_position = trajectory[1][i];
//
//         double x_position_prev = trajectory[0][i-1];
//         double y_position_prev = trajectory[1][i-1];
//
//         double theta = atan2(y_position-y_position_prev , x_position-x_position_prev);
//
//         // Get Frenet, if in same lane, check how close
//         vector<double> frenet = getFrenet(x_position, y_position, theta, map_waypoints_x, map_waypoints_y);
//         double lane = GetLane(frenet[1]);
//
//         double x = x_position - x_position_prev;
//         double y = y_position - y_position_prev;
//         double distance = sqrt(x*x+y*y);
//         double speed = distance/0.02;
//
//         double speed_diff = mean_lane_speeds[lane] - speed;
//         double new_cost = (speed_diff*speed_diff)/10;
//         // Limit tege cost to 1
//         if (new_cost > 1.0)
//             new_cost = 1.0;
//         // Choose the maximum cost
//         if (new_cost>cost)
//             cost = new_cost;
//     }
//     return cost;
// }
//
// double SpeedLimit(vector<vector<double>> trajectory)
// {
//     double cost = 0.0;
//     // Step through the trajectory and check the speed, compare to speed limit
//     for (int i=1; i<trajectory[0].size(); i++) // Step through each coordinate
//     {
//         double x_position = trajectory[0][i];
//         double y_position = trajectory[1][i];
//
//         double x_position_prev = trajectory[0][i-1];
//         double y_position_prev = trajectory[1][i-1];
//
//         double x = x_position - x_position_prev;
//         double y = y_position - y_position_prev;
//         double distance = sqrt(x*x+y*y);
//         double speed = distance/0.02 * 2.24;
//         // cout << "Speed: " << speed;
//
//         if (speed >= SPEED_LIMIT_UPPER)
//             cost = 1.0;
//     }
//     return cost;
// }
//
// double SpeedFast(vector<vector<double>> trajectory)
// {
//     // Cost will be the mean
//     double cost = 0.0;
//     // Step through the trajectory and check the speed, compare to max speed
//     for (int i=1; i<trajectory[0].size(); i++) // Step through each coordinate
//     {
//         double x_position = trajectory[0][i];
//         double y_position = trajectory[1][i];
//
//         double x_position_prev = trajectory[0][i-1];
//         double y_position_prev = trajectory[1][i-1];
//
//         double x = x_position - x_position_prev;
//         double y = y_position - y_position_prev;
//         double distance = sqrt(x*x+y*y);
//         double speed = distance/0.02 * 2.24;
//
//         double speed_diff = MAX_VELOCITY - speed;
//
//         double new_cost = (speed_diff*speed_diff)/10;
//         if (new_cost > 1.0)
//             new_cost = 1.0;
//
//         cost += new_cost;
//     }
//     cost = cost/trajectory[0].size();
//     return cost;
// }
//
// double SpeedDistance(vector<vector<double>> trajectory, vector<vector<vector<double>>> relevant_vehicles,
//                         const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
// {
//     // Cost will be the max
//     double cost = 0.0;
//     // Step through the trajectory
//     for (int i=1; i<trajectory[0].size(); i++)
//     {
//         double x_position = trajectory[0][i];
//         double y_position = trajectory[1][i];
//
//         double x_position_prev = trajectory[0][i-1];
//         double y_position_prev = trajectory[1][i-1];
//
//         double theta = atan2(y_position-y_position_prev , x_position-x_position_prev);
//
//         // Get Frenet, if in same lane, check how close
//         vector<double> frenet = getFrenet(x_position, y_position, theta, map_waypoints_x, map_waypoints_y);
//         double lane = GetLane(frenet[1]);
//
//         // Loop through the other vehicles in the same lane, get the smallest relative distance
//         double smallest_relative_distance = 9999999999.0;
//         for (int j=0; j<relevant_vehicles[lane].size(); j++)
//         {
//             // CAN USE THIS TO CALC THEIR POSITION IN THE FUTURE
//             double vx = relevant_vehicles[lane][j][3];
//             double vy = relevant_vehicles[lane][j][4];
//             double check_speed = sqrt(vx*vx+vy*vy);
//             double check_car_s = relevant_vehicles[lane][j][5];
//
//             check_car_s += ((double)i*0.02*check_speed);
//
//             double relative_distance = check_car_s - frenet[0];
//             if (relative_distance < smallest_relative_distance)
//                 smallest_relative_distance = relative_distance;
//         }
//
//         double x = x_position - x_position_prev;
//         double y = y_position - y_position_prev;
//         double distance = sqrt(x*x+y*y);
//         double speed = distance/0.02 * 2.24;
//
//
//         double new_cost = 1/(1+exp(-1*((speed/smallest_relative_distance)-5)));
//         // cout << "Speed Dist Ratio: " << (speed/smallest_relative_distance) << endl;
//         if (new_cost > 1.0)
//             new_cost = 1.0;
//
//         if (new_cost > cost)
//             cost = new_cost;
//     }
//     return cost;
// }


#endif //__COSTFUNCTIONS_H__
