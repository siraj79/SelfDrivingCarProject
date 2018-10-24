#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

#include "utils.h"
#include "Constants.h"
#include "Types.h"
using namespace std;


class Vehicle {

  private:
    int updates = 0;  // update "lane" at (updates == 6) // used in update_params_and_lane
    void Update_accel(double vx, double vy, double diff);

  public:

    int id;
    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double yaw;
    double s;
    double d;
    int lane = 1;


    Vehicle(int id);
    Vehicle(int id, double x, double y, double dx, double dy, double s, double d);
    virtual ~Vehicle();


    void Update_params(double x, double y, double yaw, double s, double d, double speed, double diff);
    void Update_params_and_lane(double x, double y, double vx, double vy, double s, double d, double diff);
    void increment(double t, const vector<double> &maps_x, const vector<double> &maps_y);


    bool is_in_front_of(Prediction pred, int checked_lane);
    bool is_behind_of(Prediction pred, int lane);
    bool is_close_to(Prediction pred, int lane);

    vector<Prediction> Generate_Predictions(double interval, int horizon, const vector<double> &maps_x, const vector<double> &maps_y);
};

#endif