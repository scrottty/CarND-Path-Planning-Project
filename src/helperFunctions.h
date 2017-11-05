#ifndef __HELPERFUNCTIONS_H__
#define __HELPERFUNCTIONS_H__

#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include "providedFunctions.h"
#include "constants.h"
#include "spline.h"
#include "costFunctions.h"

using namespace std;


bool CheckFrontCollision(vector<vector<double>> lane_vehicles, double car_s, int prev_size, double &new_speed)
{
    // Loop through all the vehicles in my lane, if within a distance match speed and flag for a lane change
    for (int i=0; i < lane_vehicles.size(); i++)
    {
        double vx = lane_vehicles[i][3];
        double vy = lane_vehicles[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = lane_vehicles[i][5];

        check_car_s += ((double)prev_size*0.02*check_speed);
        if ((check_car_s > car_s) && (check_car_s-car_s) < EMERGENCY_DIST)
        {
            cout << "Im behind: " << lane_vehicles[i][0] << " - GETTING TOO CLOSE" << endl;
            new_speed = check_speed*2.24 - 10;
            return true;
        }
    }
    return false;
}

bool CheckSlowVehicle(vector<vector<double>> safety_vehicles, double car_s, int prev_size, double &new_speed)
{
    // Loop through the vehicles in my lane, if within a distance match speed and flag for a lane change
    for (int i=0; i < safety_vehicles.size(); i++)
    {
        double vx = safety_vehicles[i][3];
        double vy = safety_vehicles[i][4];
        double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = safety_vehicles[i][5];

        check_car_s += ((double)prev_size*0.02*check_speed);
        if ((check_car_s > car_s) && (check_car_s-car_s) < DIST_TO_CAR_IN_FRONT)
        {
            cout << "Im behind: " << safety_vehicles[i][0] << endl;
            // Match their speed if not too close
            if ((check_car_s-car_s) > DIST_FOR_SLOW_DOWN)
                new_speed = check_speed*2.24;
            else
                new_speed = check_speed*2.24-2;
            return true;
        }
    }
    return false;
}

void SplitVehicelsIntoLanes( vector<vector<double>> sensor_fusion, vector<vector<vector<double>>>& lane_vehicles, double car_s)
{
    vector<vector<double>> left_lane_vehicles;
    vector<vector<double>> middle_lane_vehicles;
    vector<vector<double>> right_lane_vehicles;

    for (int i=0; i<sensor_fusion.size(); i++)
    {
        // Calc and store the relative distance
        double relative_distance = sensor_fusion[i][5] - car_s;
        sensor_fusion[i].push_back(relative_distance);
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        sensor_fusion[i].push_back(sqrt(vx*vx+vy*vy)*2.24);

        float d = sensor_fusion[i][6];
        // cout << sensor_fusion[i][0] << "," << d << "," << relative_distance << endl;

        // Store each vehicle based upon their lane positions
        if (d < LEFT_LANE_MAX)
            left_lane_vehicles.push_back(sensor_fusion[i]);
        else if (d > RIGHT_LANE_MIN)
            right_lane_vehicles.push_back(sensor_fusion[i]);
        else
            middle_lane_vehicles.push_back(sensor_fusion[i]);
    }

    lane_vehicles.push_back(left_lane_vehicles);
    lane_vehicles.push_back(middle_lane_vehicles);
    lane_vehicles.push_back(right_lane_vehicles);
}

void FilterVehiclesForSafety( vector<vector<vector<double>>> lane_vehicles, vector<vector<vector<double>>>& safety_vehicles)
{
    // Filter cars on the lanes by position to find only the relavent cars (safety)
    vector<vector<double>> close_left_lane_vehicles;
    vector<vector<double>> close_middle_lane_vehicles;
    vector<vector<double>> close_right_lane_vehicles;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<lane_vehicles[i].size(); j++)
        {
            double relative_distance = lane_vehicles[i][j][7];
            if (relative_distance > BACK_MOST_DISTANCE_SAFETY+1 && relative_distance < FRONT_MOST_DISTANCE_SAFETY)
            {
                if (i==0)
                    close_left_lane_vehicles.push_back(lane_vehicles[i][j]);
                else if (i==1)
                    close_middle_lane_vehicles.push_back(lane_vehicles[i][j]);
                else
                    close_right_lane_vehicles.push_back(lane_vehicles[i][j]);
            }
        }
    }
    safety_vehicles.push_back(close_left_lane_vehicles);
    safety_vehicles.push_back(close_middle_lane_vehicles);
    safety_vehicles.push_back(close_right_lane_vehicles);
}

void FilterForRelaventVehicles( vector<vector<vector<double>>> lane_vehicles, vector<vector<vector<double>>>& relevant_vehicles)
{
    // Filter cars on the lanes by position to find only the relavent cars (safety)
    vector<vector<double>> relavant_left_lane_vehicles;
    vector<vector<double>> relavant_middle_lane_vehicles;
    vector<vector<double>> relavant_right_lane_vehicles;
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<lane_vehicles[i].size(); j++)
        {
            double relative_distance = lane_vehicles[i][j][7];
            if (relative_distance > BACK_MOST_DISTANCE_RELEVANT+1 && relative_distance < FRONT_MOST_DISTANCE_RELEVANT)
            {
                if (i==0)
                    relavant_left_lane_vehicles.push_back(lane_vehicles[i][j]);
                else if (i==1)
                    relavant_middle_lane_vehicles.push_back(lane_vehicles[i][j]);
                else
                    relavant_right_lane_vehicles.push_back(lane_vehicles[i][j]);
            }
        }
    }
    relevant_vehicles.push_back(relavant_left_lane_vehicles);
    relevant_vehicles.push_back(relavant_middle_lane_vehicles);
    relevant_vehicles.push_back(relavant_right_lane_vehicles);
}

vector<vector<double>> CreateTrajectory(vector<int> lanes, double target_vel, double& ref_vel,
    double steps_forwards, bool avoid_vehicle, const vector<double>& previous_path_x, const vector<double>& previous_path_y,
    const vector<double>& car_values, const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
{
    double car_x = car_values[0];
    double car_y = car_values[1];
    double car_s = car_values[2];
    double car_yaw = car_values[4];

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
    else // Use the previous parths end point as starting ref
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
    for (int i=1; i<steps_forwards-previous_path_x.size(); i++)
    {
        if (avoid_vehicle) // Too close to vehilce do maximum de accleration
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

// Returns -1 for move left, 0 for stay, 1 for move right
// int ChooseLane( vector<vector<vector<double>>> safety_vehicles, int lane, double car_s)
// {
//     // Return if in the edge lanes
//     if (lane == 0)
//         return 1;
//     else if (lane == 2)
//         return -1;
//
//     // // Choose lane based upon the number of cars in it that either behind me or
//     // // within 10m in front
//     // int left_lane_cost = 0;
//     // int right_lane_cost = 0;
//     //
//     // // Cost weightings
//     // double num_vechiles_weight = 1.0;
//
//     // If there are no vehicles in either lane take the left lane else take the empty lane
//     if (safety_vehicles[0].size() == 0 && safety_vehicles[2].size() == 0)
//     {
//         return -1;
//     }
//     else if (safety_vehicles[0].size() == 0)
//     {
//         return -1;
//     }
//     else if (safety_vehicles[2].size() == 0)
//     {
//         return 1;
//     }
//
//     // If vehicles in the lane stay in the lane
//     return 0;
// }


// Returns the lane to be in 30m forward
int ChooseLane( vector<vector<vector<double>>> relevant_vehicles, double& target_vel, double ref_vel, double current_lane,
    const vector<double>& previous_path_x, const vector<double>& previous_path_y, const vector<double>& car_values,
    const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
{
    // Find which lane we want to be in 90m forward
    // It is chosen on the number of cars in the lane and speed of the lane
    vector<int> num_cars;
    for (int i=0; i<relevant_vehicles.size(); i++)
        num_cars.push_back(relevant_vehicles[i].size());

    // Calculate average speed of each lane
    vector<double> max_lane_speed;
    max_lane_speed.push_back(MAX_VELOCITY);
    max_lane_speed.push_back(MAX_VELOCITY);
    max_lane_speed.push_back(MAX_VELOCITY);
    vector<double> current_lane_speed;
    current_lane_speed.push_back(target_vel);
    current_lane_speed.push_back(target_vel);
    current_lane_speed.push_back(target_vel);
    vector<double> slow_lane_speed;
    slow_lane_speed.push_back(target_vel-2);
    slow_lane_speed.push_back(target_vel-2);
    slow_lane_speed.push_back(target_vel-2);
    vector<double> mean_lane_speed;
    mean_lane_speed.push_back(MAX_VELOCITY);
    mean_lane_speed.push_back(MAX_VELOCITY);
    mean_lane_speed.push_back(MAX_VELOCITY);
    for (int i=0; i<3; i++)
    {
        double mean_speed = 0;
        for (int j=0; j<relevant_vehicles[i].size(); j++)
        {
            mean_speed += relevant_vehicles[i][j][8];
        }
        if (mean_speed != 0) // If there where no vehicles
            mean_lane_speed[i] = (mean_speed/relevant_vehicles[i].size());
    }
    int goal_lane = ChooseGoalLane(num_cars, mean_lane_speed, current_lane);
    // Generate possible paths
    // Step through the different lane options and speeds to produce the various options
    vector<int> possible_lanes;
    possible_lanes.push_back(0);
    possible_lanes.push_back(1);
    possible_lanes.push_back(2);
    vector<vector<double>> speeds;
    speeds.push_back(max_lane_speed);
    speeds.push_back(current_lane_speed);
    speeds.push_back(mean_lane_speed);
    speeds.push_back(slow_lane_speed);
    vector<vector<vector<double>>> potential_trajectories;
    vector<vector<int>> lanes_stored;
    vector<double> choosen_speed;
    for (int i=0; i<possible_lanes.size(); i++) // Lane option for the 30m point
    {
        for (int j=0; j<possible_lanes.size(); j++) // Lane option for the 60m point
        {
            for (int k=0; k<possible_lanes.size(); k++) // Lane option for the 90m point
            {
                for (int m=0; m<speeds.size(); m++) // different speed options
                {
                    // Miss the trajectories that will cause the vehicle to do a double lane change
                    if (possible_lanes[i] > current_lane+1 || possible_lanes[i] < current_lane-1)
                        continue;

                    vector<int> lanes;
                    lanes.push_back(possible_lanes[i]);
                    lanes.push_back(possible_lanes[j]);
                    lanes.push_back(possible_lanes[k]);
                    // lanes.push_back(possible_lanes[i]);
                    // lanes.push_back(possible_lanes[j]);
                    // lanes.push_back(possible_lanes[k]);
                    double ref_vel_temp = ref_vel; // Need to reset the value of ref vel to what it is now
                    // The selected speed is for the lane of the 30m mark
                    vector<vector<double>> xy = CreateTrajectory(lanes, speeds[m][possible_lanes[i]], ref_vel_temp, 200, false,
                        previous_path_x, previous_path_y, car_values, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    potential_trajectories.push_back(xy);
                    lanes_stored.push_back(lanes); // Store this for later
                    choosen_speed.push_back(speeds[m][possible_lanes[i]]);
                }
            }
        }
    }

    // cout << " Actual S: " << car_values[2] << endl;

    // Go through each trajectory and calc the cost
    vector<vector<double>> all_costs;
    vector<double> all_total_costs;
    for (int i=0; i<potential_trajectories.size(); i++)
    {
        double total_cost = 0;
        double avoid_cost = AvoidCollision(potential_trajectories[i],relevant_vehicles, map_waypoints_x, map_waypoints_y) * COLLISION_COST;
        double Buffer_cost = DistanceBuffer(potential_trajectories[i],relevant_vehicles, map_waypoints_x, map_waypoints_y) * BUFFER_COST;
        double DesiredLane_cost = DesiredLane(lanes_stored[i] ,goal_lane) * DESIRED_LANE_COST;
        double CurrentLane_cost = CurrentLane(lanes_stored[i], current_lane) * CURRENT_LANE_COST;
        double SpeedTraffic_cost = SpeedTraffic(potential_trajectories[i],mean_lane_speed, map_waypoints_x, map_waypoints_y) * SPEED_TRAFFIC_COST;
        double SpeedLimit_cost = SpeedLimit(potential_trajectories[i]) * SPEED_LIMIT_COST;
        double SpeedFast_cost = SpeedFast(potential_trajectories[i]) * SPEED_FAST_COST;

        vector<double> cost;
        cost.push_back(avoid_cost);
        cost.push_back(Buffer_cost);
        cost.push_back(DesiredLane_cost);
        cost.push_back(CurrentLane_cost);
        cost.push_back(SpeedTraffic_cost);
        cost.push_back(SpeedLimit_cost);
        cost.push_back(SpeedFast_cost);
        all_costs.push_back(cost);
        total_cost = avoid_cost + Buffer_cost + DesiredLane_cost + CurrentLane_cost + SpeedTraffic_cost + SpeedLimit_cost + SpeedFast_cost;
        all_total_costs.push_back(total_cost);
    }

    // Find the minimum cost and the trajectory
    double minimum_cost = 99999999;
    int minimum_idx = 99999999;
    for (int i=0; i<all_costs.size(); i++)
    {
        if (all_total_costs[i] < minimum_cost)
        {
            minimum_cost = all_total_costs[i];
            minimum_idx = i;
        }
    }

    // for (int i=0; i<potential_trajectories.size(); i++)
    // {
    //     cout << "Trajectory: [" << lanes_stored[i][0] << ","
    //             << lanes_stored[i][1] << "," << lanes_stored[i][2]
    //             << "] | Speed: " << choosen_speed[i] << " | Cost: " << all_total_costs[i] << "| Costs: [" << all_costs[i][0] << ","
    //                 << all_costs[i][1] << "," << all_costs[i][2] << "," << all_costs[i][3] << "," << all_costs[i][4]
    //                 << "," << all_costs[i][5] << "," << all_costs[i][6] << "]"<< endl;
    // }

    cout << "Choosen Trajectory: [" << lanes_stored[minimum_idx][0] << ","
            << lanes_stored[minimum_idx][1] << "," << lanes_stored[minimum_idx][2]
            << "] | Speed: " << choosen_speed[minimum_idx] << " | Cost: " << minimum_cost << endl;

    target_vel = choosen_speed[minimum_idx];
    return lanes_stored[minimum_idx][0];


    // DO I JUST RETURN THE POINTS OR RECALCULATE WITH THE SAFETY STUFF INCLUDED????


    /*
    // Return if in the edge lanes
    if (lane == 0)
        return 1;
    else if (lane == 2)
        return -1;

    // // Choose lane based upon the number of cars in it that either behind me or
    // // within 10m in front
    // int left_lane_cost = 0;
    // int right_lane_cost = 0;
    //
    // // Cost weightings
    // double num_vechiles_weight = 1.0;

    // If there are no vehicles in either lane take the left lane else take the empty lane
    // if (relevant_vehicles[0].size() == 0 && relevant_vehicles[2].size() == 0)
    // {
    //     return -1;
    // }

    // If there are no vehicles in either lane take the left lane else take the empty lane
    if (safety_vehicles[0].size() == 0 && safety_vehicles[2].size() == 0)
    {
        return -1;
    }
    else if (safety_vehicles[0].size() == 0)
    {
        return -1;
    }
    else if (safety_vehicles[2].size() == 0)
    {
        return 1;
    }

    // If vehicles in the lane stay in the lane
    return 0; */

    /*
    cout << "left lane vechiles" << endl;
    // Sort the list by the distance to my car
    sort(left_lane_vehicles.begin(), left_lane_vehicles.end(), [](const vector<double>& a, const vector<double>& b) {return a[7]<b[7];} );
    // loop through each list and get the metrics
    vector<double> distances;
    distances.push_back(BACK_MOST_DISTANCE);
    vector<double> gaps;
    vector<double> distance_to_gaps;
    vector<double> speeds;
    for (int i=0; i<left_lane_vehicles.size(); i++)
    {
        // double vx = left_lane_vehicles[i][3];
        // double vy = left_lane_vehicles[i][4];
        // double vehicle_speed = sqrt(vx*vx+vy*vy);
        // speeds.push_back(vehicle_speed);

        double distance = left_lane_vehicles[i][7];
        distances.push_back(distance);
        double gap = distance - distances[i];
        double gap_distance = distance;

        cout << "-- " << left_lane_vehicles[i][0] << "," << left_lane_vehicles[i][6] << "," << distance << ","  << distances[i] << "," << gap << endl;
    }

    // // Calc the gaps from the relative distances
    // distance.push_back(BACK_MOST_DISTANCE); // Put a fake position at the furthest back so the distance to the back most car gets included
    //
    // for (int i=0; i<distance.size()-1; i++)
    // {
    //     double gap = distance[i+1] - distance[i];
    //     double dist_to_gap = distance[i+1]; // the back of the front vehicle
    //     if (gap > MIN_GAP_SIZE)
    //     {
    //         gaps.push_back(gap);
    //         distance_to_gaps.push_back(dist_to_gap);
    //     }
    // }

    return false;*/
}

bool CheckLaneIsSafe(vector<vector<double>> vehicles_in_new_lane)
{
    if (vehicles_in_new_lane.size() == 0)
        return true;
    else
        return false;
}






#endif //__HELPERFUNCTIONS_H__
