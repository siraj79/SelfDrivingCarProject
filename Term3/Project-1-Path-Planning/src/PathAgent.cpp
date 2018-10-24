#include "PathAgent.hpp"
#include <iostream>
#include <vector>
#include "spline.h"
#include "utils.h"
#include "Constants.h"
#include "Types.h"
#include "Trajectory.hpp"
#include "FSM.h"
#include "Vehicle.h"
#include "PathAgent.hpp"
using namespace std;
using namespace std::chrono;


PathAgent::PathAgent()
{
	fsm.ego_car = ego_car;
	fsm.proposed_lane = ego_car.lane;

	ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()); // time steps
}

void PathAgent::Update_time_step() {
    milliseconds new_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    diff = (double)(new_time - ms).count() / 1000;
    ms = new_time;
}


#pragma Get trajectory
	vector<double> PathAgent::Get_Next_X_values()
	{
		return trajectory.next_x_vals;
	}

	vector<double> PathAgent::Get_Next_Y_values()
	{
		return trajectory.next_y_vals;
	}
#pragma endregion


#pragma Update_Ego_Vehicles_State_and_Trajectory
	void PathAgent::Update_Ego_Vehicles_State_and_Trajectory(double car_s, double x, double y, double yaw, double s, double d, double speed, vector<double> previous_path_x, vector<double> previous_path_y)
	{
	    fsm.ego_car.Update_params(x, y, yaw, s, d, speed, diff); // diff is updated in update_time_step()
	 	fsm.Update_state(car_s, predictions, trajectory.map_x, trajectory.map_y);
	    fsm.Realize_state(predictions);

	    trajectory.Generate_Trajectory(car_s, ego_car.x, ego_car.y, yaw, ego_car.lane, fsm.Get_expected_velocity(), previous_path_x, previous_path_y);
	}
#pragma endregion


#pragma Update_NonEgo_Vehicles_State
	void PathAgent::Update_NonEgo_Vehicles_State(json sensor_fusion){
	    predictions.clear();

	    for (auto data : sensor_fusion) {
	      // data[id, x, y, dx, dy, s, d]
	      Vehicle* vehicle = NULL;
	      if (((double)data[5] <= MAX_S) && ((double)data[6] >= 0)) 
	      {// check if car is visible
	        if (vehicles.find(data[0]) == vehicles.end()) 
	        {
	          vehicle = new Vehicle(data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
	          vehicles[data[0]] = vehicle;
	        }
	        else 
	        {
	          vehicle = vehicles[data[0]];
	          (*vehicle).Update_params_and_lane(data[1], data[2], data[3], data[4], data[5], data[6], diff);

	          vector<Prediction> car_preds = (*vehicle).Generate_Predictions(PREDICTION_INTERVAL,10, trajectory.map_x, trajectory.map_y);
	          predictions[(*vehicle).id] = car_preds;		          
	        }
	      }
	      else 
	      {
	        auto it = vehicles.find(data[0]);
	        if (it != vehicles.end()) {
	          //remove vehicle
	          delete (*it).second;
	          vehicles.erase((int)data[0]);
	        }
	      }
	    }
	}
#pragma endregion