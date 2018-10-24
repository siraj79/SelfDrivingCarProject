#ifndef FSM_H
#define FSM_H

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "Constants.h"
#include "Types.h"
#include "Vehicle.h"
#include "Estimator.h"

class FSM {

  public:
    FSM(Vehicle& car);
    ~FSM();

    Vehicle& ego_car;
    
    double Get_expected_velocity();
    void Update_state(double car_s, map<int, vector<Prediction>> predictions, const vector<double> &maps_x, const vector<double> &maps_y);
    void Realize_state(map<int, vector<Prediction> > predictions);

  //private:
    Estimator estimator = Estimator();
    double ref_vel = 0;
    int proposed_lane;
    CarState state = CarState::CS;
    



    void realize_constant_speed();
    void realize_keep_lane(map<int, vector<Prediction> > predictions);
    void realize_lane_change(map<int, vector<Prediction> > predictions, string direction);
    void realize_prep_lane_change(map<int, vector<Prediction> > predictions, string direction);


    void _update_ref_speed_for_lane(map<int, vector<Prediction> > predictions, int lane);
    //vector<Snapshot> trajectory_for_state(CarState state, map<int, vector<Prediction>> predictions, int horizon);
    vector<Snapshot> trajectory_for_state(CarState proposed_state, map<int, vector<Prediction>> predictions, int horizon, const vector<double> &maps_x, const vector<double> &maps_y); 
    void restore_state_from_snapshot(Snapshot snapshot);
    Snapshot get_snapshot();
  };

#endif