#ifndef __HELPERFUNCTIONS_H__
#define __HELPERFUNCTIONS_H__

#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

bool CheckFrontCollision(vector<vector<double>> sensor_fusion, double car_s, int lane_middle, int prev_size, double &new_speed)
{
    // Loop through each vehicle, if it is in front and going slower, flag
    bool too_close = false;
    for (int i=0; i < sensor_fusion.size(); i++)
    {
        // Car is my lane
        float d = sensor_fusion[i][6];
        if ( d < (lane_middle+2) && d > (lane_middle-2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            check_car_s += ((double)prev_size*0.02*check_speed);
            if ((check_car_s > car_s) && (check_car_s-car_s) < 20)
            {
                too_close = true;
                new_speed = check_speed*2.24;
                return too_close;
            }
            else if ((check_car_s > car_s) && (check_car_s-car_s) < 30) 
            {
                new_speed = check_speed*2.24;
            }
        }
    }
    return too_close;
}






#endif //__HELPERFUNCTIONS_H__