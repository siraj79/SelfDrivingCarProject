#include "Trajectory.hpp"
#include <vector>
#include <fstream>
#include <iostream>
#include <math.h>

#include "json.hpp"
#include "spline.h"
#include "utils.h"
#include "Constants.h"
#include "Types.h"

using namespace std;


#pragma Get_TrajectoryData Functions
  Trajectory::Trajectory() 
  { 
    string map_file = "../data/highway_map.csv";
  	ifstream in_map(map_file.c_str(), ifstream::in);
      
      ///////////// Add Waypoints ///////////////////////////////
  	string line;
  	while (getline(in_map, line)) {
  		istringstream iss(line);
  	    double x;
  	    double y;
  	    float s;
  	    float d_x;
  	    float d_y;
  	    iss >> x;
  	    iss >> y;
  	    iss >> s;
  	    iss >> d_x;
  	    iss >> d_y;
  	    
  	    map_x.push_back(x);
      	map_y.push_back(y);
      	map_s.push_back(s);
      	map_dx.push_back(d_x);
      	map_dy.push_back(d_y);
  	}

  	/////////////////// init /////////////////////////////////
  	map_x.push_back(map_x[0]);
    map_y.push_back(map_y[0]);
    map_s.push_back(MAX_S);
    map_dx.push_back(map_dx[0]);
    map_dy.push_back(map_dy[0]);

    spline_x.set_points(map_s, map_x);
    spline_y.set_points(map_s, map_y);
    spline_dx.set_points(map_s, map_dx);
    spline_dy.set_points(map_s, map_dy);
    /////////////////////////////////////////////////////////
  }
#pragma endregion


#pragma Get_TrajectoryData Functions
  void Trajectory::Generate_Trajectory(double car_s, double original_x, double original_y, double original_yaw, int lane, double ref_vel, vector<double> previous_path_x, vector<double> previous_path_y)
  {
      this->previous_path_x = previous_path_x;
      this->previous_path_y = previous_path_y;
  	  // Create a list of widly spaced waypoints (x,y), evenly spaced at 30 m
      // Later we will interpolate these waypoints with a spline and fill it in with more points tha control speed
      next_x_vals.clear();
      next_y_vals.clear();

      vector<double> ptsx;
      vector<double> ptsy;

      if (abs(ref_vel) < 0.1) {
        cout << "car stopped" << endl;
        return;
      }

      double ref_x = original_x;
      double ref_y = original_y;
      double ref_yaw = deg2rad(original_yaw);

      int prev_size = previous_path_x.size();

      if (prev_size < 2) {
        ref_yaw = deg2rad(original_yaw);
        // Use two points that make the path tangent to the car
        double prev_car_x = original_x - cos(original_yaw);
        double prev_car_y = original_y - sin(original_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(original_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(original_y);
      }
      else 
      {
      	    // use the previous path's end point as starting reference
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
      }

      // In Frenet add evenly 30m spaced points ahead of the starting reference
      double d = MIDDLE_LANE + LANE_WIDTH * lane;
      Coord next_wp0 = getXY(car_s +   DISTANCE, d , spline_x, spline_y, spline_dx, spline_dy);
      Coord next_wp1 = getXY(car_s + 2*DISTANCE, d , spline_x, spline_y, spline_dx, spline_dy);
      Coord next_wp2 = getXY(car_s + 3*DISTANCE, d , spline_x, spline_y, spline_dx, spline_dy);

      ptsx.push_back(next_wp0.x);
      ptsx.push_back(next_wp1.x);
      ptsx.push_back(next_wp2.x);

      ptsy.push_back(next_wp0.y);
      ptsy.push_back(next_wp1.y);
      ptsy.push_back(next_wp2.y);

      Update_Trajectory(ptsx, ptsy, ref_vel,ref_x,ref_y, ref_yaw);
  }
#pragma endregion


#pragma Update_Trajectory
  void Trajectory::Update_Trajectory(vector<double> ptsx, vector<double> ptsy, double ref_vel,double ref_x,double ref_y,double ref_yaw)
  {
  	convert2Local(ptsx, ptsy, ref_x, ref_y, ref_yaw);

  	// create a spline
  	tk::spline s;
  	// set x, y points to the spline
  	s.set_points(ptsx, ptsy);

  	double path_length = 50;

  	// fill with the previous points from the last time
  	int size = previous_path_x.size();

  	for (int i = 0; i < size; ++i) {
  	  next_x_vals.push_back(previous_path_x[i]);
  	  next_y_vals.push_back(previous_path_y[i]);
  	}


  	// calculate how to break up spline points so that we travel at our ddesired reference velocity
  	// we have local coordinates here
  	double target_x = DISTANCE;
  	double target_y = s(target_x);
  	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

  	double x_add_on = 0;

  	// fill up the rest planner after filling it with the previous points, here we will always output 50 points
  	double N = target_dist / (INTERVAL*convert2mps(ref_vel));
  	for (int i = 1; i <= path_length - size; ++i) 
  	{
  	  double x_point = x_add_on + (target_x) / N;
  	  double y_point = s(x_point);

  	  x_add_on = x_point;

  	  // rotate back to normal after rotating it earlier
  	  Coord point = convert2global(x_point, y_point, ref_x, ref_y, ref_yaw);

  	  next_x_vals.push_back(point.x);
  	  next_y_vals.push_back(point.y);
  	}
  }
#pragma endregion
