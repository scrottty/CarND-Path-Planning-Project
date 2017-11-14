#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include <math.h>

class Vehicle
{
public:
    double ID;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

    double velocity;
    double velocity_mph;
    double relative_dist;

    Vehicle(double ID, double x, double y, double vx, double vy, double s, double d);
    double CalcPositionForward (int stepsForward) const;
    void CalcVelocity();
};

Vehicle::Vehicle(double ID, double x, double y, double vx, double vy, double s, double d)
{
    this->ID = ID;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;

    CalcVelocity();
}

void Vehicle::CalcVelocity()
{
    velocity = sqrt(vx*vx+vy*vy);
    velocity_mph = velocity*2.24;
}

double Vehicle::CalcPositionForward(int stepsForward) const
{
    return s+(double)stepsForward*0.02*velocity;
}

#endif //__VEHICLE_H__
