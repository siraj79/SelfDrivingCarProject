#ifndef utils_h
#define utils_h

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


using namespace std;
using json = nlohmann::json;




// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }



inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

inline int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	//double heading = atan2((map_y-y),(map_x-x));
	//double angle = fabs(theta-heading);
    //angle = min(2*pi() - angle, angle);

	double heading = atan2(map_y,map_x) + pi()/2;
	double angle = std::abs(theta-heading);

	if(angle > pi()/4)
    {
  	  	closestWaypoint++;
		if (closestWaypoint == maps_x.size()-1)
		{
		   closestWaypoint = 0;
		}
	}
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline Frenet getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-2;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

    Frenet frenet;
    frenet.s = frenet_s;
    frenet.d = frenet_d;
	return frenet;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline Coord getXY(double s, double d, tk::spline& spline_x,tk::spline& spline_y,tk::spline& spline_dx,tk::spline& spline_dy)
{
	Coord coord;
	double path_x = spline_x(s);
	double path_y = spline_y(s);
	double dx = spline_dx(s);
	double dy = spline_dy(s);
	coord.x = path_x + d * dx;
	coord.y = path_y + d * dy;

	return coord;
}

inline void convert2Local(vector<double>& ptsx, vector<double>& ptsy, double ref_x,double ref_y, double ref_yaw) {

    for (int i = 0; i < ptsx.size(); ++i) {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
      ptsy[i] = (shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
    }
 }

 inline Coord convert2global(double x, double y, double ref_x,double ref_y, double ref_yaw) {

    Coord coord;
    coord.x = (x*cos(ref_yaw) - y*sin(ref_yaw));
    coord.y = (x*sin(ref_yaw) + y*cos(ref_yaw));

    coord.x += ref_x;
    coord.y += ref_y;

    return coord;
 }
 inline double convert2mps(double mph) { return mph / 2.24; }
#endif