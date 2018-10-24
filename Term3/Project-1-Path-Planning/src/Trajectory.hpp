#ifndef Trajectory_hpp
#define Trajectory_hpp

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "spline.h"
#include "utils.h"
#include "Constants.h"
#include "Types.h"
using namespace std;



class Trajectory {
    public: 
    	vector<double> next_x_vals;
        vector<double> next_y_vals;
        vector<double> previous_path_x;
        vector<double> previous_path_y;

        vector<double> map_x;
        vector<double> map_y;
        vector<double> map_s;
        vector<double> map_dx;
        vector<double> map_dy;


        tk::spline spline_x;
    	tk::spline spline_y;
    	tk::spline spline_dx;
    	tk::spline spline_dy;


    public: 
        Trajectory();
    	void Generate_Trajectory(double car_s, double original_x, double original_y, double original_yaw, int lane, double ref_vel, vector<double> previous_path_x, vector<double> previous_path_y);

    private:
        void Update_Trajectory(vector<double> ptsx, vector<double> ptsy, double ref_vel,double ref_x,double ref_y,double ref_yaw);
};



#endif