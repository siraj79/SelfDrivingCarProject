#ifndef PathAgent_hpp
#define PathAgent_hpp

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
#include "utils.h"
#include "Constants.h"
#include "Types.h"
#include "Trajectory.hpp"
#include "FSM.h"
#include "Vehicle.h"
using namespace std;
using namespace std::chrono;



class PathAgent {
private: 
	map<int, Vehicle*> vehicles;
    map<int, vector<Prediction>> predictions;
	
	//Time Steps
	double diff;
    milliseconds ms;

	Trajectory trajectory = Trajectory();
	Vehicle ego_car = Vehicle(-1);
    FSM fsm = FSM(ego_car);


public: 
	PathAgent();
	void Update_Ego_Vehicles_State_and_Trajectory(double car_s, double x, double y, double yaw, double s, double d, double speed, vector<double> previous_path_x, vector<double> previous_path_y);
    void Update_NonEgo_Vehicles_State(json sensor_fusion);
    void Update_time_step();

    vector<double> Get_Next_X_values();
    vector<double> Get_Next_Y_values();
};

#endif
