#ifndef types_h
#define types_h


#include <iostream>
#include <random>
#include <math.h>
#include <vector>
#include <string>
#include <iterator>


struct Frenet
{
   double s;
   double d;
};

struct Coord {
   double x;
   double y;
};

enum class CarState { CS = 0, KL = 1, PLCL = 2, PLCR = 3, LCL = 4, LCR = 5 };

struct Snapshot {
    double x;
    double y;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double s;
    double d;
    double yaw;
    int lane;
    int proposed_lane;
    CarState state;
    double ref_vel;


    double Get_Speed() {
      return sqrt(dx*dx + dy*dy);
    }

    double Get_Acceleration() {
      return sqrt(ddx*ddx + ddy*ddy);
    }
};

struct Collision {
	bool hasCollision = false;
	int step = 1000;
};

struct TrajectoryData
{
    int proposed_lane;
    int current_lane;
    double avg_speed;
    double prop_closest_approach;
    double actual_closest_approach;
    Collision collides;
};


struct Prediction {
    static double LANE_WIDTH;
    double s;
    double d;
    double vx;
    double vy;
    double x;
    double y;
    int lane;

    double get_distance(double other_x, double other_y, double other_s) {
      double diff_car = sqrt((x-other_x)*(x-other_x) + (y-other_y)*(y-other_y));
      double diff_frenet = other_s - s;
      if (diff_car - std::abs(diff_frenet) < 100) { // no circle overlap
        return diff_frenet;
      }
      else {
        return copysign(diff_car, diff_frenet);
      }
    }

    double get_velocity() {
      return sqrt(vx*vx + vy*vy);
    }
};

struct Estimate {
  CarState state;
  double cost;
};



 #endif