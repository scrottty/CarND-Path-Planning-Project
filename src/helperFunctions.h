#ifndef __HELPERFUNCTIONS_H__
#define __HELPERFUNCTIONS_H__

#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "providedFunctions.h"
#include "constants.h"
#include "spline.h"
#include "costFunctions.h"
#include "Vehicle.h"
#include "Ego.h"

using namespace std;


bool CheckFrontCollision(vector<Vehicle> lane_vehicles, Ego ego)
{
    // Loop through all the vehicles in my lane, if within a distance slow down. This is done
    // against the current positions rather than the estimated forward
    for (int i=0; i < lane_vehicles.size(); i++)
    {
        double check_car_s = lane_vehicles[i].s;

        if ((check_car_s > ego.current_s) && (check_car_s-ego.current_s) < EMERGENCY_DIST)
        {
            cout << "Im behind: " << lane_vehicles[i].ID << " - GETTING TOO CLOSE" << endl;
            return true;
        }
    }
    return false;
}

void SplitVehicelsIntoLanes( vector<vector<double>> sensor_fusion, vector<vector<Vehicle>>& lane_vehicles, double car_s, int prev_size)
{
    vector<Vehicle> left_lane_vehicles;
    vector<Vehicle> middle_lane_vehicles;
    vector<Vehicle> right_lane_vehicles;

    for (int i=0; i<sensor_fusion.size(); i++)
    {
        Vehicle new_vehicle (sensor_fusion[i][0],sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],
                                sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6]);

        double check_car_s = new_vehicle.CalcPositionForward(prev_size);
        new_vehicle.relative_dist = check_car_s - car_s;

        // Print out the vehicles
        // cout << new_vehicle.ID << " | " << new_vehicle.x << " | " << new_vehicle.y << " | " << new_vehicle.velocity_mph << " | " <<
        //         new_vehicle.relative_dist << " | " << new_vehicle.d << endl;

        // Store each vehicle based upon their lane positions
        if (new_vehicle.d < LEFT_LANE_MAX)
            left_lane_vehicles.push_back(new_vehicle);
        else if (new_vehicle.d > RIGHT_LANE_MIN)
            right_lane_vehicles.push_back(new_vehicle);
        else
            middle_lane_vehicles.push_back(new_vehicle);
    }

    lane_vehicles.push_back(left_lane_vehicles);
    lane_vehicles.push_back(middle_lane_vehicles);
    lane_vehicles.push_back(right_lane_vehicles);
}

void FilterVehicles(const vector<vector<Vehicle>>& lane_vehicles, vector<vector<Vehicle>>& relevant_vehicles,
                                vector<vector<Vehicle>>& safety_vehicles)
{
    // Filter cars on the lanes by position to find only the relavent cars for safety
    // This being the cars the are just in front of us and most of those behind us (in case of fast moving vehicles)
    for (int i=0; i<3; i++)
    {
        vector<Vehicle> close_lane_vehicles;
        vector<Vehicle> relevant_lane_vehicles;
        for (int j=0; j<lane_vehicles[i].size(); j++)
        {
            if (lane_vehicles[i][j].relative_dist > BACK_MOST_DISTANCE_SAFETY+1 &&
                    lane_vehicles[i][j].relative_dist < FRONT_MOST_DISTANCE_SAFETY)
                close_lane_vehicles.push_back(lane_vehicles[i][j]);

            if (lane_vehicles[i][j].relative_dist > BACK_MOST_DISTANCE_RELEVANT+1 &&
                    lane_vehicles[i][j].relative_dist < FRONT_MOST_DISTANCE_RELEVANT)
                relevant_lane_vehicles.push_back(lane_vehicles[i][j]);

        }
        safety_vehicles.push_back(close_lane_vehicles);
        relevant_vehicles.push_back(relevant_lane_vehicles);
    }
}

vector<vector<double>> CreateTrajectory(vector<int> lanes, double target_vel, double& ref_vel,
    double steps_forwards, bool avoid_vehicle, const vector<double>& previous_path_x, const vector<double>& previous_path_y,
    const Ego ego, const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
{
    double car_x = ego.x;
    double car_y = ego.y;
    double car_s = ego.s;
    double car_yaw = ego.yaw;

    // Create a list of widely spaced waypoint to interpolate between
    vector<double> ptsx;
    vector<double> ptsy;

    // Reference x,y,yaw state. These are saved for the coordinate transformations
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
    // If the previous path is almost empty, use the car as the starting reference
    int prev_size = previous_path_x.size();
    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else // Use the previous paths end point as starting ref
    {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
        // Use the two points to make the path tangent to the previous path's ebd point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // In Frenet add evenly spaced points ahead of starting position
    vector<double> next_wp0 = getXY(car_s+30, (2+4*lanes[0]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s+60, (2+4*lanes[1]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s+90, (2+4*lanes[2]), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // We now have 5 point in the vector, the last representing the end of the current path, then 3 plotted forward

    // Shift the points to the different coordinates
    for (int i=0; i < ptsx.size(); i++)
    {
        // Shift car reference angle to 0 degress
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    }

    // Create Spline
    tk::spline s;

    // Set (x,y) ppints to the spline
    s.set_points(ptsx, ptsy);

    // Define the actual path point we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with all of the remaining previous path points
    // Note: points travelled by the vehicle are removed from the previous path
    for (int i=0; i<previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calc how to break up the spline points so that we travel at the desired velocity
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

    double x_add_on = 0;
    // Set the ref velocity to the cars current speed and inc and dec from there
    // Fill in the points for the spline
    for (int i=1; i<(steps_forwards-previous_path_x.size()); i++)
    {
        if (avoid_vehicle && ref_vel > target_vel-10) // Too close to vehilce do maximum de accleration until 10 kph less
        {
            ref_vel -= 0.4; // Not the full deacceleration but close
        }
        else if (ref_vel > target_vel)
        {
            // Can slow down slower if dotn need to avoid
            ref_vel -= 0.112;
        }
        else if (ref_vel < target_vel)
        {
            ref_vel += 0.224;
        }

        // Calc the size of the step
        double N = (target_dist/(0.02*ref_vel/2.24));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);

        // Save the start point for the next loop
        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to the normal coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double>> xy;
    xy.push_back(next_x_vals);
    xy.push_back(next_y_vals);

    return xy;
}

// Returns the lane to be in 30m forward
int ChooseLane( vector<vector<Vehicle>> relevant_vehicles, double& target_vel, double ref_vel, double current_lane,
    const vector<double>& previous_path_x, const vector<double>& previous_path_y,
    const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
{
    // For each lane calculate the following
    // Number of cars in the lane
    // Distance to the closest car in front
    // Velocity of the closest car
    vector<int> num_cars;
    double total_num_cars = 0;
    // Number of cars 'close'
    vector<int> num_close_cars;
    num_close_cars.push_back(0);
    num_close_cars.push_back(0);
    num_close_cars.push_back(0);
    // Preset the distance to the max in case there are no vehicles
    vector<double> distance_to_vehicles;
    distance_to_vehicles.push_back(FRONT_MOST_DISTANCE_RELEVANT);
    distance_to_vehicles.push_back(FRONT_MOST_DISTANCE_RELEVANT);
    distance_to_vehicles.push_back(FRONT_MOST_DISTANCE_RELEVANT);

    // Preset the speed to max in case of no cars
    vector<double> speed_of_vehicles;
    speed_of_vehicles.push_back(MAX_VELOCITY);
    speed_of_vehicles.push_back(MAX_VELOCITY);
    speed_of_vehicles.push_back(MAX_VELOCITY);

    // Go through each lane
    for (int i=0; i<relevant_vehicles.size(); i++)
    {
        num_cars.push_back(relevant_vehicles[i].size());
        total_num_cars += relevant_vehicles[i].size();

        // Loop through the cars in the lane
        double minimum_distance = 99999999.0;
        for (int j=0; j<relevant_vehicles[i].size(); j++)
        {
            if (relevant_vehicles[i][j].relative_dist < DOUBLE_LANE_CLOSE_DIST)
                num_close_cars[i]++;

            if (relevant_vehicles[i][j].relative_dist < minimum_distance)
            {
                minimum_distance = relevant_vehicles[i][j].relative_dist;
                distance_to_vehicles[i] = relevant_vehicles[i][j].relative_dist;
                speed_of_vehicles[i] = relevant_vehicles[i][j].velocity_mph;
            }
        }
    }

    double cost = 9999999999;
    int choosen_lane = current_lane;
    // Loop through each lane and calc the cost
    for (int i=0; i<3; i++)
    {
        double new_cost = 0.0;
        new_cost += LaneVelocityCost(speed_of_vehicles[i]) * LANE_VELOCITY_COST;
        new_cost += LaneVelocityDifferenceCost(speed_of_vehicles[i], speed_of_vehicles[current_lane]) * LANE_VELOCITY_DIFF_COST;
        new_cost += LaneDistanceCost(distance_to_vehicles[i]) * LANE_DISTANCE_COST;
        new_cost += LaneNumVehiclesCost(num_cars[i], total_num_cars) * LANE_NUM_VEHICLES_COST;
        new_cost += LaneNumVehiclesCrossedCost(num_close_cars, i, current_lane) * LANE_NUM_CLOSE_VEHICLES_COST;
        new_cost += LaneChangeCost(i, current_lane) * LANE_CHANGE_COST;

        cout << "Lane: " << i << " | Vel: " << speed_of_vehicles[i] << " cost: " << LaneVelocityCost(speed_of_vehicles[i]) * LANE_VELOCITY_COST <<
                " | Vel Diff: " << speed_of_vehicles[current_lane]-speed_of_vehicles[i] << " cost: " << LaneVelocityDifferenceCost(speed_of_vehicles[i],speed_of_vehicles[current_lane]) * LANE_VELOCITY_DIFF_COST <<
                " | Dist: " << distance_to_vehicles[i] << " cost: " << LaneDistanceCost(distance_to_vehicles[i]) * LANE_DISTANCE_COST <<
                " | Num: " << num_cars[i] << " cost: " << LaneNumVehiclesCost(num_cars[i], total_num_cars) * LANE_NUM_VEHICLES_COST <<
                " | Num Crossed cost: " << LaneNumVehiclesCrossedCost(num_close_cars, i, current_lane) * LANE_NUM_CLOSE_VEHICLES_COST <<
                " | LC: " << abs(current_lane-i) << " cost: " << LaneChangeCost(i, current_lane) * LANE_CHANGE_COST <<
                " | Total: " << new_cost << endl;

        if (new_cost < cost)
        {
            cost = new_cost;
            choosen_lane = i;
        }

    }

    cout << "Goal - Lane: " << choosen_lane << " | num_cars: " << num_cars[choosen_lane] << " | Speed: " << speed_of_vehicles[choosen_lane] <<
            " | Distance: " << distance_to_vehicles[choosen_lane] << " | Lane Change: " << abs(current_lane-choosen_lane) << endl;

    return choosen_lane;
}

// Will return the lane to change
int ConfirmLaneChange(int new_lane, int current_lane, int& prev_desired_lane, int& bean_count)
{
    // increment the bean count if the new lane is the same as previously requested
    if (new_lane == prev_desired_lane)
        bean_count++;
    else // else reset the bean_count
        bean_count = 0;

    // Reset the prev desired lane to the new lane for next round
    prev_desired_lane = new_lane;

    // If the bean count has accumulate enough return the new_lane, reset the bean_count
    if (bean_count > LANE_CHANGE_BEAN_COUNT_UPPER)
    {
        bean_count = 0;
        return new_lane;
    }
    else // else return the current lane until the other has enough
        return  current_lane;
}

double ChooseSpeed(vector<vector<Vehicle>> relevant_vehicles, double ref_vel, double new_lane, double current_lane,
    const vector<double>& previous_path_x, const vector<double>& previous_path_y, const Ego& ego,
    const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y,
    vector<vector<double>>& choosen_trajectory)
{
    vector<double> speed_options;
    speed_options.push_back(MAX_VELOCITY);
    if (ego.velocity < MAX_VELOCITY-0.5) // Add current speed if sufficiently less
        speed_options.push_back(ego.velocity);


    // Get the speeds of the closest cars in each lane
    // Preset the speed to max in case of no cars
    vector<double> speed_of_vehicles;
    speed_of_vehicles.push_back(MAX_VELOCITY);
    speed_of_vehicles.push_back(MAX_VELOCITY);
    speed_of_vehicles.push_back(MAX_VELOCITY);

    for (int i=0; i<relevant_vehicles.size(); i++)
    {
        // Loop through the cars in the lane
        double minimum_distance = 99999999.0;
        for (int j=0; j<relevant_vehicles[i].size(); j++)
        {
            if (relevant_vehicles[i][j].relative_dist < minimum_distance)
            {
                minimum_distance = relevant_vehicles[i][j].relative_dist;
                speed_of_vehicles[i] = relevant_vehicles[i][j].velocity_mph;
            }
        }
    }

    // Add the relevant lane speeds
    speed_options.push_back(speed_of_vehicles[current_lane]);
    if (new_lane != current_lane)
    {
        speed_options.push_back(speed_of_vehicles[new_lane]);
        if (abs(new_lane - current_lane) > 1)
        {
            speed_options.push_back(speed_of_vehicles[current_lane + (new_lane-current_lane)/2]);
        }
    }

    // Choose the lanes
    vector<int> lanes;
    lanes.push_back(new_lane);
    lanes.push_back(new_lane);
    lanes.push_back(new_lane);
    if ( abs(current_lane-new_lane) > 1)
        lanes[0] = current_lane + (new_lane-current_lane)/2;

    // Generate Trajectories for each speed
    vector<vector<vector<double>>> potential_trajectories;
    for (int i=0; i<speed_options.size(); i++)
    {
        double ref_vel_temp = ref_vel;
        vector<vector<double>> xy = CreateTrajectory(lanes, speed_options[i], ref_vel_temp, 100, false,
                            previous_path_x, previous_path_y, ego, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        potential_trajectories.push_back(xy);
    }

    // Calc the cost for each trajectory
    double cost = 99999999999.9;
    int choosen_idx = 99999;
    for (int i=0; i<potential_trajectories.size(); i++)
    {
        double new_cost = 0.0;
        double minimum_distance = FRONT_MOST_DISTANCE_RELEVANT; //Done here for debugging
        double collision_cost = CollisionCost(potential_trajectories[i], relevant_vehicles, map_waypoints_x, map_waypoints_y, minimum_distance) * COLLISION_COST;
        double max_velocity_cost = MaxVelocityCost(speed_options[i]) * MAX_VELOCITY_COST;
        double new_lane_vel_cost = MatchLaneChangeVelocityCost(speed_options[i], speed_of_vehicles[new_lane], abs(new_lane-current_lane)) * MATCH_LANE_VELOCTIY_COST;
        new_cost = collision_cost + max_velocity_cost + new_lane_vel_cost;

        cout << "Speed: " << speed_options[i] << " | Distance: " << minimum_distance << " cost: " << collision_cost <<
                 " | Velocity: " << (MAX_VELOCITY - speed_options[i]) << " cost " << max_velocity_cost <<
                 " | Lane Vel: " << speed_of_vehicles[new_lane] << " lane diff: " << abs(new_lane-current_lane) << " cost " << new_lane_vel_cost <<
                 " | Total: " << new_cost << endl;

        if (new_cost < cost)
        {
            cost = new_cost;
            choosen_idx = i;
        }
    }
    cout << "CHOOSEN SPEED: " << speed_options[choosen_idx] << endl;

    choosen_trajectory = potential_trajectories[choosen_idx];
    return speed_options[choosen_idx];
}

bool CheckLaneIsSafe(bool& slow_down, vector<vector<Vehicle>>& safety_vehicles, const vector<vector<double>>& trajectory,
                        const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y,
                        const int& current_lane, const int& new_lane)
{
    // Want to check that i dont get too close to any vehicles during the trajectory
    // Also want to check that the vehicle in front isn't too close - if slow ill slow down
    int assess_idx = 50;

    int lane = new_lane;
    if (abs(new_lane-current_lane) > 1)
        lane = current_lane + (new_lane-current_lane)/2;

    // Get the position at the end of the trajectory and see where itll be relative to the other vehicles
    double x_position = trajectory[0][assess_idx];
    double y_position = trajectory[1][assess_idx];

    double x_position_prev = trajectory[0][assess_idx-1];
    double y_position_prev = trajectory[1][assess_idx-1];

    double theta = atan2(y_position-y_position_prev , x_position-x_position_prev);

    // Get Frenet, check how close to vehicles in the lane
    vector<double> frenet = getFrenet(x_position, y_position, theta, map_waypoints_x, map_waypoints_y);

    double car_s = frenet[0];

    // loop through the vehicles at this time and check their estimated positions
    // only check for the lane moving into
    for (int j=0; j<safety_vehicles[lane].size(); j++)
    {
        // Predict the cars in the future (tamely)
        double check_car_s = safety_vehicles[lane][j].CalcPositionForward(assess_idx);
        double relative_distance = check_car_s - car_s;

        cout << "Relative Dist: " << relative_distance << " lane: " << lane << endl;
        // check if the relative distances are too close, if so return false and bail on the change
        if ((relative_distance > -SAFE_TO_CROSS_DISTANCE) && (relative_distance < SAFE_FRONT_CHANGE_INTO))
        {
            // cout << " Relative Dist: " << relative_distance << " Lane: " << lane << endl;
            return false;
        }
    }

    // Check the car in front of me and make sure it isn't too close. If so, slow down
    // Loop through the cars in the lane
    double minimum_distance = 999999999.0;
    for (int i=0; i<safety_vehicles[current_lane].size(); i++)
    {
        if (safety_vehicles[current_lane][i].relative_dist < minimum_distance &&
                safety_vehicles[current_lane][i].relative_dist > -SAFE_TO_CROSS_DISTANCE)
        {
            minimum_distance = safety_vehicles[current_lane][i].relative_dist;
        }
    }

    cout << "Minimum Distance: " << minimum_distance << endl;
    if (minimum_distance < SAFE_FRONT_DISTANCE)
        slow_down = true;

    return true;
}

// Will return the lane to change
bool ConfirmSafe(bool decision, int& bean_count)
{
    // increment the bean count if it is safe
    if (decision == true)
        bean_count++;
    else // else reset the bean_count
        bean_count = 0;

    // If the bean count has accumulate enough return true, reset bean count
    if (bean_count > SAFETY_CHECK_BEAN_COUNT_UPPER)
    {
        bean_count = 0;
        return decision;
    }
    else // else return not safe
        return  false;
}

#endif //__HELPERFUNCTIONS_H__
