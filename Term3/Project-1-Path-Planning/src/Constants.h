
#ifndef constants_h
#define constants_h

#include <fstream>
#include <iostream>
using namespace std;

double const INTERVAL = .02;
double const DISTANCE = 30;
double const LANE_WIDTH = 4.0;
double const MIDDLE_LANE = LANE_WIDTH/2;
double const MIN_SPEED = 0.3;

//Sensor fusion data
double const MAX_S = 6945.554;
//string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
 // double max_s = 6945.554;


double const SAFE_DISTANCE = 20.0;
double const DESIRED_BUFFER = SAFE_DISTANCE*2;

// priority levels for costs
int const COLLISION = pow(10, 6);
int const DANGER = 3*pow(10, 5);
int const COMFORT = pow(10, 4);
int const EFFICIENCY = pow(10, 3);

int const PLANNING_HORIZON = 1;
double const PREDICTION_INTERVAL = 0.15;

double const MANOEUVRE = 4.0;
double const OBSERVED_DISTANCE = 65;
double const MAX_DISTANCE = 999999;

double const SPEED_INCREMENT = .224;
double const MAX_SPEED = 49.96;
double const TIME_INTERVAL = 0.02;
double const PREDICTIONS_COUNT = 5;
int const lanes_available = 3;

#endif