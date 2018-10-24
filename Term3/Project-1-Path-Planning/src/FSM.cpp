#include "PathAgent.hpp"
#include <iostream>
#include <vector>
#include "spline.h"
#include "utils.h"
#include "Constants.h"
#include "Types.h"
#include "FSM.h"
#include "Vehicle.h"
#include "Estimator.h"
using namespace std;



FSM::FSM(Vehicle& car) : ego_car(car) {
	proposed_lane = car.lane;
}

FSM::~FSM() {}



#pragma Update_state
	void FSM::Update_state(double car_s, map<int, vector<Prediction>> predictions, const vector<double> &maps_x, const vector<double> &maps_y) {
		/*

		Updates the "state" of the vehicle by assigning one of the following values to 'self.state':

		"KL" - Keep Lane
		"LCL" or "LCR" - Lane Change Left / Right
		"PLCL" or "PLCR" - Prepare for Lane Change Left / Right

		*/
		vector<CarState> states;
		if (state == CarState::PLCR) {
		  states = vector<CarState>{ CarState::KL, CarState::PLCR, CarState::LCR };
		}
		else if (state == CarState::PLCL) {
		  states = vector<CarState>{ CarState::KL, CarState::PLCL, CarState::LCL };
		}
		else if (state == CarState::LCR) {
		  states = vector<CarState>{ CarState::KL };
		}
		else if (state == CarState::LCL) {
		  states = vector<CarState>{ CarState::KL };
		}
		else {
		  states = vector<CarState>{ CarState::KL };
		  if (ego_car.lane > 0) {
		    states.push_back(CarState::PLCL);
		  }
		  if (ego_car.lane < lanes_available - 1) {
		    states.push_back(CarState::PLCR);
		  }
		}

		if (states.size() == 1) 
		{
			state = states[0];
		}
		else 
		{
			auto costs = vector<Estimate>();
			for (auto state : states) {
			  Estimate estimate;
			  estimate.state = state;
			  auto trajectory = trajectory_for_state(state, predictions, PREDICTIONS_COUNT, maps_x, maps_y);
			  estimate.cost = estimator.Calculate_Cost(car_s, ref_vel, trajectory, predictions, state);
			  costs.push_back(estimate);
			}

			auto best = min_element(std::begin(costs), std::end(costs), [](Estimate est1, Estimate est2) {   return est1.cost < est2.cost;	});
			state = (*best).state;
		}
	}

	vector<Snapshot> FSM::trajectory_for_state(CarState proposed_state, map<int, vector<Prediction>> predictions, int horizon, const vector<double> &maps_x, const vector<double> &maps_y) {
		// remember current state
		auto initial_snapshot = get_snapshot();

		// pretend to be in new proposed state
		//state = proposed_state;
		vector<Snapshot> trajectory = { initial_snapshot };
		for (int i = 0; i < horizon; ++i) {
		  restore_state_from_snapshot(initial_snapshot);
		  state = proposed_state;
		  Realize_state(predictions);
		  ego_car.increment(i*PREDICTION_INTERVAL, maps_x, maps_y);
		  trajectory.push_back(get_snapshot());

		  // need to remove first prediction for each vehicle.
		  for (auto pair : predictions) {
		    auto vehicle_pred = pair.second;
		    vehicle_pred.erase(vehicle_pred.begin());
		  }
		}

		// restore state from snapshot
		restore_state_from_snapshot(initial_snapshot);
		return trajectory;
	}
#pragma endregion


#pragma Realize_state and support functions
	void FSM::Realize_state(map<int, vector<Prediction>> predictions) {
		//Given a state, realize it by adjusting velocity and lane.
		if (state == CarState::CS)
		{
		  realize_constant_speed();
		}
		else if (state == CarState::KL)
		{
		  realize_keep_lane(predictions);
		}
		else if (state == CarState::LCL)
		{
		  realize_lane_change(predictions, "L");
		}
		else if (state == CarState::LCR)
		{
		  realize_lane_change(predictions, "R");
		}
		else if (state == CarState::PLCL)
		{
		  realize_prep_lane_change(predictions, "L");
		}
		else if (state == CarState::PLCR)
		{
		  realize_prep_lane_change(predictions, "R");
		}
	}

	void FSM::realize_constant_speed() { }

	void FSM::realize_keep_lane(map<int, vector<Prediction>> predictions) {
		proposed_lane = ego_car.lane;
		_update_ref_speed_for_lane(predictions, ego_car.lane);
	}

	void FSM::realize_lane_change(map<int, vector<Prediction>> predictions, string direction) {
		int delta = -1;
		if (direction.compare("R") == 0)
		{
		  delta = 1;
		}
		ego_car.lane += delta;
		proposed_lane = ego_car.lane;
		_update_ref_speed_for_lane(predictions, proposed_lane);
	}

	void FSM::realize_prep_lane_change(map<int, vector<Prediction>> predictions, string direction) {
	    int delta = -1;
	    bool close = false;
	    if (direction.compare("R") == 0)
	    {
	      delta = 1;
	    }
	    proposed_lane = ego_car.lane + delta;

	    vector<vector<Prediction>> at_behind;
	    for (auto pair : predictions) {
	      int v_id = pair.first;
	      vector<Prediction> v = pair.second;
	      if (ego_car.is_in_front_of(v[0], proposed_lane)) {
	        at_behind.push_back(v);
	      }
	      if (ego_car.is_close_to(v[0], ego_car.lane)) {
	        if (v[0].get_distance(ego_car.x, ego_car.y, ego_car.s) < 4 ) {
	          close = true;
	        }
	      }
	    }
	    if (at_behind.size() > 0)
	    {
	      double velocity = ref_vel;
	      if (close) {
	        if (velocity > 40.0) {
	          velocity -= 2 * SPEED_INCREMENT;
	        }
	        else {
	          velocity -= SPEED_INCREMENT;
	        }
	      }
	      else {
	        velocity += SPEED_INCREMENT;
	      }
	      if (velocity > MAX_SPEED) {
	        velocity = MAX_SPEED;
	      }
	      ref_vel = velocity;
	    }
	}

	void FSM::_update_ref_speed_for_lane(map<int, vector<Prediction>> predictions, int checked_lane) {
	    bool too_close = false, keep_speed = false, danger = false;
	    double max_speed = MAX_SPEED;

	    for (auto pair : predictions) {
	      Prediction pred = pair.second[0];
	      double target_speed = pred.get_velocity();
	      if (ego_car.is_behind_of(pred, checked_lane) && target_speed < max_speed) {
	        // follow the car behavior
	        max_speed = target_speed - SPEED_INCREMENT;
	        keep_speed = true;
	      }
	      if (ego_car.is_close_to(pred, checked_lane)) {
     
	        if (pred.s < ego_car.s + 5) {
	          danger = true;
	        }
	        too_close = true;
	      }
	    }
	    double velocity = ref_vel;
	    if (too_close) {
	      if (danger) {
	        if (velocity > 40.0) {
	          velocity -= 2 * SPEED_INCREMENT;
	        }
	        else {
	          velocity -= SPEED_INCREMENT;
	        }
	      }
	      else {
	        if (velocity < max_speed) {
	          velocity += SPEED_INCREMENT;
	        }
	        else if (velocity > max_speed) {
	          if (velocity > 40.0) {
	            velocity -= 2 * SPEED_INCREMENT;
	          }
	          else {
	            velocity -= SPEED_INCREMENT;
	          }
	        }
	      }
	    }
	    else {
	      if (keep_speed && velocity > 25 && velocity > max_speed*2.7) {
	        velocity -= SPEED_INCREMENT;
	      }
	      else {
	        velocity += SPEED_INCREMENT;
	      }
	      if (velocity > MAX_SPEED) {
	        velocity = MAX_SPEED;
	      }
	    }

	    if (velocity < 0) {
	      velocity = 0;
	    }
	    ref_vel = velocity;
	}
#pragma endregion


#pragma Get and Set functions 
	double FSM::Get_expected_velocity() {
		////////////////////////////
		return ref_vel;
	}

	void FSM::restore_state_from_snapshot(Snapshot snapshot) {
		this->ego_car.s = snapshot.s;
		this->ego_car.d = snapshot.d;
		this->ego_car.x = snapshot.x;
		this->ego_car.y = snapshot.y;
		this->ego_car.dx = snapshot.dx;
		this->ego_car.dy = snapshot.dy;
		this->ego_car.ddx = snapshot.ddx;
		this->ego_car.ddy = snapshot.ddy;
		this->ego_car.yaw = snapshot.yaw;
		this->ego_car.lane = snapshot.lane;
		this->state = snapshot.state;
		this->ref_vel = snapshot.ref_vel;
		this->proposed_lane = snapshot.proposed_lane;
	}

	Snapshot FSM::get_snapshot() {
		Snapshot snapshot_temp;
		snapshot_temp.x = this->ego_car.x;
		snapshot_temp.y = this->ego_car.y;
		snapshot_temp.dx = this->ego_car.dx;
		snapshot_temp.dy = this->ego_car.dy;
		snapshot_temp.s = this->ego_car.s;
		snapshot_temp.d = this->ego_car.d;
		snapshot_temp.ddx = this->ego_car.ddx;
		snapshot_temp.ddy = this->ego_car.ddy;
		snapshot_temp.yaw = this->ego_car.yaw;
		snapshot_temp.lane = this->ego_car.lane;
		snapshot_temp.state = this->state;
		snapshot_temp.ref_vel = this->ref_vel;
		snapshot_temp.proposed_lane = this->proposed_lane;

		return snapshot_temp;
	}
#pragma endregion


