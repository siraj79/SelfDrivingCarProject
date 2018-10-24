#ifndef Estimator_H
#define Estimator_H

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include <iostream>
#include <algorithm>
#include <functional>
#include <numeric>
#include <map>

#include "json.hpp"
#include "spline.h"
#include "Constants.h"
#include "Types.h"
#include "Vehicle.h"

using namespace std;


class Estimator {

	public:
	    Estimator();
		double Calculate_Cost(double car_s, double ref_vel, vector<Snapshot> trajectory, map<int, vector<Prediction>>predictions, CarState state);
    
    private:
    	double Cost_Inefficiency(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const;
    	double Cost_Collision(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const;
    	double Cost_Buffer(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const;
    	double Cost_ChangeLane(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const;
    	double Cost_FreeLine(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const;

    	map<int, vector<Prediction>> Filter_Predictions_By_Lane(map<int, vector<Prediction>> predictions, int lane);
        bool Check_Collision(double car_s, double ref_speed, Snapshot snap, Prediction s_now, CarState checkstate, bool lack_of_space);
        TrajectoryData Get_TrajectoryData(double car_s, double ref_s, vector<Snapshot> trajectory, map<int, vector<Prediction>>predictions, CarState checkstate);
};


#endif