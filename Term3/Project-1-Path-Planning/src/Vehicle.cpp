#include "utils.h"
#include "Constants.h"
#include "Types.h"
#include "Vehicle.h"
using namespace std;



Vehicle::Vehicle(int id, double x, double y, double dx, double dy, double s, double d) {
  this->id = id;
  this->x = x;
  this->y = y;
  this->dx = dx;
  this->dy = dy;
  this->s = s;
  this->d = d;
  this->lane = (int)d / 4;
  this->ddx = 0;
  this->ddy = 0;
  double angle = atan2(dy, dx);
  this->yaw = (abs(angle) < 0.1) ? 0 : angle;
  this->updates = 0;
}

Vehicle::Vehicle(int id) {
  this->id = id;
}

Vehicle::~Vehicle() {}




void Vehicle::Update_params(double x, double y, double yaw, double s, double d, double speed, double diff) {
  this->x = x;
  this->y = y;
  this->yaw = deg2rad(yaw);
  Update_accel(speed*cos(this->yaw), speed*sin(this->yaw), diff);
  this->s = s;
  this->d = d;
}

void Vehicle::Update_accel(double vx, double vy, double diff) {
  this->ddx = (vx - this->dx) / diff;
  if (this->ddx < 0.01) {
    this->ddx = 0;
  }
  this->ddy = (vy - this->dy) / diff;
  if (this->ddy < 0.01) {
    this->ddy = 0;

  }
  this->dx = vx;
  this->dy = vy;
}

void Vehicle::Update_params_and_lane(double x, double y, double vx, double vy, double s, double d, double diff) {
  // used for non ego car 
  double new_angle = atan2(vy, vx);
  this->yaw = (abs(new_angle) < 0.1) ? 0 : new_angle;
  this->x = x;
  this->y = y;
  Update_accel(vx, vy, diff);
  this->s = s;
  this->d = d;

  int new_lane = (int) d / 4;
  if (new_lane != this->lane){
    if (++updates > 6) {
      this->lane = new_lane;
      updates = 0;
    }
  }
  else {
    updates = 0;
  }    
}

void Vehicle::increment(double t, const vector<double> &maps_x, const vector<double> &maps_y) {
  if (abs(this->ddy) < 0.001) {
    this->y += this->dy * t;
  }
  else {
    this->y += this->dy * t + this->ddy*t*t / 2;
    this->dy += this->ddy * t;
  }
  if (abs(this->ddx) < 0.001) {
    this->x += this->dx * t;
  }
  else {
    this->x += this->dx * t + this->ddx*t*t / 2;
    this->dx += this->ddx * t;
  }
  double new_angle = atan2(dy, dx);
  this->yaw = (new_angle < 0.1) ? 0 : new_angle;
  Frenet frenet = getFrenet(this->x, this->y, this->yaw, maps_x, maps_y); // map function
  this->s = frenet.s;
  this->d = frenet.d;
}



bool Vehicle::is_in_front_of(Prediction pred, int checked_lane) {
  return (pred.lane == checked_lane) && pred.get_distance(x, y, s) >= 0;
}

bool Vehicle::is_behind_of(Prediction pred, int lane) {
  double distance = -pred.get_distance(x, y, s);
  return (pred.lane == lane) && (distance >= 0 && distance < 2*SAFE_DISTANCE);
}

bool Vehicle::is_close_to(Prediction pred, int lane) {
  double distance = -pred.get_distance(x, y, s);
  return (pred.lane == lane) && distance >= 0 && (distance < SAFE_DISTANCE);
}

vector<Prediction> Vehicle::Generate_Predictions(double interval, int horizon, const vector<double> &maps_x, const vector<double> &maps_y) {
  vector<Prediction> predictions;

  for (int i = 0; i < horizon; i++)
  {
    double t = i*interval;
    Prediction pred;
    if (std::abs(this->ddy) < 0.001) {
      pred.y = this->y + this->dy * t;
      pred.vy = this->dy;
    }
    else {
      pred.y = this->y + this->dy * t + this->ddy * t * t / 2;
      pred.vy = this->dy + this->ddy * t;
    }
    if (std::abs(this->ddy) < 0.001) {
      pred.x = this->x + this->dx * t;
      pred.vx = this->dx;
    }
    else {
      pred.x = this->x + this->dx * t + this->ddx * t * t / 2;
      pred.vx = this->dx + this->ddx * t;
    }
    double new_angle = atan2(pred.vy, pred.vx);
    double yaw = (new_angle < 0.1) ? 0 : new_angle;
    Frenet frenet = getFrenet(pred.x, pred.y, yaw, maps_x, maps_y); // map function
    pred.s = frenet.s;
    pred.d = frenet.d;
    pred.lane = this->lane;

    predictions.push_back(pred);
  }

  return predictions;
}

  