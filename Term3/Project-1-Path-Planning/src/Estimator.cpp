#include "Estimator.h"
#include "Types.h"
#include "Constants.h"
using namespace std;


Estimator::Estimator() 
{
}

#pragma Cost Functions
	double Estimator::Cost_Inefficiency(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const {
		double speed = data.avg_speed;
		double target_speed = MAX_SPEED;
		double diff = target_speed - speed;
		double pct = diff / target_speed;
		double multiplier = pow(pct, 2);
		return 8*multiplier * EFFICIENCY;
	}

	double Estimator::Cost_Collision(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const {
		if (data.collides.hasCollision) 
		{
		  double time_til_collision = 0;
		  double exponent = time_til_collision*time_til_collision;
		  double mult = exp(-exponent);
		  return mult * COLLISION;
		}
		return 0;
	}

	double Estimator::Cost_Buffer(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const {
		double closest = data.actual_closest_approach;

		if (closest < SAFE_DISTANCE/2) {
		  return 3*DANGER;
		}

		if (closest > DESIRED_BUFFER) {
		  return 0.0;
		}

		double multiplier = 1.0 - pow((closest / DESIRED_BUFFER), 2);
		return 3*multiplier * DANGER;
	}

	double Estimator::Cost_ChangeLane(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const {
		if (data.proposed_lane != data.current_lane) {
		  if (data.proposed_lane == 1) {
		    return 0;
		  }
		  return COMFORT;
		}
		return 0;
	}

	double Estimator::Cost_FreeLine(vector<Snapshot> trajectory, map<int, vector<Prediction>> predictions, TrajectoryData data) const {
		double closest = data.prop_closest_approach;

		if (closest > OBSERVED_DISTANCE) {
		  double multiplier = (MAX_DISTANCE - closest) / MAX_DISTANCE;
		  return 20*multiplier*multiplier;
		}
		double multiplier = (OBSERVED_DISTANCE - closest) / OBSERVED_DISTANCE;
		return 5*multiplier * multiplier * COMFORT;
	}

	double Estimator::Calculate_Cost(double car_s, double ref_vel, vector<Snapshot> trajectory, map<int, vector<Prediction>>predictions, CarState state) {
		
		TrajectoryData data = Get_TrajectoryData(car_s, ref_vel, trajectory, predictions, state);

		double cost = 0.0;
		cost += Cost_Inefficiency(trajectory, predictions, data);
		cost += Cost_Collision(trajectory, predictions, data);
		cost += Cost_Buffer(trajectory, predictions, data);
		cost += Cost_ChangeLane(trajectory, predictions, data);
		cost += Cost_FreeLine(trajectory, predictions, data);

		return cost;
	}
#pragma endregion


#pragma Private Support Functions
	map<int, vector<Prediction>> Estimator::Filter_Predictions_By_Lane(map<int, vector<Prediction>> predictions, int lane) {
		map<int, vector<Prediction>> filtered = {};	
		for (auto pair: predictions) {
		  if (pair.second[0].lane == lane) {
		    filtered[pair.first] = pair.second;
		  }
		}
		return filtered;
	}

	bool Estimator::Check_Collision(double car_s, double ref_speed, Snapshot snap, Prediction s_now, CarState checkstate, bool lack_of_space) {

	    double diff = s_now.get_distance(snap.x, snap.y, snap.s);
	    double prediction_time = 4 / snap.dx;

	    if (car_s > s_now.s) 
	    {
	      double predicted_distance1v = diff + prediction_time*(snap.Get_Speed() - s_now.get_velocity());
	      double predicted_distance2v = diff + 10* PREDICTION_INTERVAL*(ref_speed - s_now.get_velocity());
	      if ((predicted_distance2v < MANOEUVRE || predicted_distance1v < MANOEUVRE || lack_of_space || diff < -1.0)) 
	      {
	        return true;
	      }
	    }
	    else 
	    {
	      double predicted_distance1v = -diff + 3*PREDICTION_INTERVAL*(s_now.get_velocity() - snap.Get_Speed());
	      if (predicted_distance1v < 0 || -diff < -MANOEUVRE) 
	      {
	        return true;
	      }
	    }
	    return false;
	}
#pragma endregion


#pragma Get_TrajectoryData Functions
	TrajectoryData Estimator::Get_TrajectoryData(double car_s, double ref_s, vector<Snapshot> trajectory, map<int, vector<Prediction>>predictions, CarState checkstate) 
	{
	    TrajectoryData data = TrajectoryData();

	    vector<Snapshot> t = trajectory;
	    // actual state
	    Snapshot current_snapshot = t[0];
	    Snapshot first = t[1];
	    Snapshot last = t[t.size() - 1];

	    double dt = trajectory.size()*PREDICTION_INTERVAL;
	    // for lane change we see actual line after current state only
	    data.current_lane = first.lane;
	    data.proposed_lane = last.proposed_lane;
	    data.avg_speed = (last.Get_Speed()*dt - current_snapshot.Get_Speed()) / dt; // (v2*dt-v1*1)/dt

	    // initialize a bunch of variables
	    data.prop_closest_approach = MAX_DISTANCE;
	    data.actual_closest_approach = MAX_DISTANCE;

	    data.collides = Collision();
	    data.collides.hasCollision = false;
	    bool checkCollisions = current_snapshot.lane != data.proposed_lane;

	    map<int, vector<Prediction>> cars_in_proposed_lane = Filter_Predictions_By_Lane(predictions, data.proposed_lane);
	    map<int, vector<Prediction>> cars_in_actual_lane = Filter_Predictions_By_Lane(predictions, data.current_lane);

	    for (int i = 0; i < PLANNING_HORIZON; ++i) {
	      Snapshot snap = trajectory[i];

	      for (auto pair : cars_in_actual_lane) {
	        Prediction state = pair.second[i];
	        double dist = -state.get_distance(snap.x, snap.y, snap.s);
	        if (dist >= 0 && dist < data.actual_closest_approach) {
	          data.actual_closest_approach = dist;
	        }
	      }
	    }

	    for (int i = 0; i < PLANNING_HORIZON; ++i) {
	      Snapshot snap = trajectory[i];

	      for (auto pair : cars_in_proposed_lane) {
	        Prediction state = pair.second[i];
	        //double pred_car_s = car_s + i*0.15*ref_s;
	        double dist = -state.get_distance(snap.x, snap.y, snap.s);
	        if (checkCollisions) {
	          bool vehicle_collides = Check_Collision(car_s, ref_s, snap, state, checkstate,
	            data.actual_closest_approach < MANOEUVRE);
	          if (vehicle_collides) {
	            data.collides.hasCollision = true;
	            data.collides.step = i;
	          }
	          else if (car_s > state.s) {
	            dist = MAX_DISTANCE;
	          }
	        }
	        if (dist >= 0 && dist < data.prop_closest_approach) {
	          data.prop_closest_approach = dist;
	          if (data.proposed_lane == data.current_lane) {
	            data.actual_closest_approach = data.prop_closest_approach;
	          }
	        }
	      }
	    }

	    return data;
	}
#pragma endregion


