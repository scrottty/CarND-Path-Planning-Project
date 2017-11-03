#ifndef __HELPERFUNCTIONS_H__
#define __HELPERFUNCTIONS_H__

#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include "constants.h"

using namespace std;

const int DIST_TO_CAR_IN_FRONT              = 30;
const int DIST_FOR_SLOW_DOWN                = 20;
const int EMERGENCY_DIST                    = 10;
const int LEFT_LANE_MAX                     = 4;
const int RIGHT_LANE_MIN                    = 8;
const int FRONT_MOST_DISTANCE_SAFETY        = 10;
const int BACK_MOST_DISTANCE_SAFETY         = -20;
const int FRONT_MOST_DISTANCE_PERFORMANCE   = 100;
const int BACK_MOST_DISTANCE_PERFORAMCE     = -20;
const int MIN_GAP_SIZE                      = 5;

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
        
        float d = sensor_fusion[i][6];
        cout << sensor_fusion[i][0] << "," << d << "," << relative_distance << endl;
        
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

void FilterVehiclesForPerformance( vector<vector<vector<double>>> lane_vehicles, vector<vector<vector<double>>>& performance_vehicles)
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
            if (relative_distance > BACK_MOST_DISTANCE_PERFORAMCE+1 && relative_distance < FRONT_MOST_DISTANCE_PERFORMANCE)
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
    performance_vehicles.push_back(relavant_left_lane_vehicles);
    performance_vehicles.push_back(relavant_middle_lane_vehicles);
    performance_vehicles.push_back(relavant_right_lane_vehicles);
}

// Returns -1 for move left, 0 for stay, 1 for move right
int ChooseLane( vector<vector<vector<double>>> safety_vehicles, int lane, double car_s)
{
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
    return 0;
    
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