#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include "Eigen/Dense"


using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  #pragma region TODO: Initialize variable.

    double lagcut = 0.99;

    #pragma region initializing steer
      VectorXd steer_Params = VectorXd(3);
      double steer_Kp = 0.1;
      double steer_Ki = 0.0004;
      double steer_Kd = 2;
      double steer_UpperLimit = 1.0;
      double steer_LowerLimit = -1.0;
      steer_Params << steer_Kp, steer_Ki, steer_Kd;

      PID steer_pid;
      steer_pid.Init(steer_Kp, steer_Ki, steer_Kd); //PID from the Lesson

      twiddle steer_twiddle;
      steer_twiddle.Init(steer_Params);
    #pragma endregion

    #pragma region initializing speed
      VectorXd speed_Params = VectorXd(3);
      double speed_Kp = 0.2;
      double speed_Ki = 0.0;
      double speed_Kd = 0.1;
      double throttle_UpperLimit = 1.0;
      double throttle_LowerLimit = -1.0;
      speed_Params << speed_Kp, speed_Ki, speed_Kd;

      PID speed_pid;
      speed_pid.Init(speed_Kp, speed_Ki, speed_Kd); //PID from the Lesson

      twiddle speed_twiddle;
      speed_twiddle.Init(speed_Params);

      double max_speed = 75.0;
      double speed_reduction = 55.0;
    #pragma endregion
  #pragma endregion

//  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value = 0.3; // Default throttle 



          #pragma region TODO: Calcuate steering & throttle values here
            #pragma region steer
              steer_pid.UpdateError(cte);
              steer_value = steer_pid.TotalError(true, steer_LowerLimit, steer_UpperLimit, true, lagcut);
              steer_twiddle.Add_to_Total_Error(cte);

              if (steer_twiddle.number_of_error_added == 40) {

                steer_Params = steer_twiddle.Optimize_Parameter();
                steer_Kp = steer_Params[0];
                steer_Ki = steer_Params[1];
                steer_Kd = steer_Params[2];
                steer_pid.Init(steer_Kp, steer_Ki, steer_Kd);
              }
            #pragma endregion
          
            #pragma region speed / throttle

              //The CTE used for the speed controller is then the difference between the current speed and the target speed.
              //This enables the vehicle to slow down during sharp turns and accelarate during straight stretches of the roadway.

              //Set the actual speed based on the magnitude of steering angle
              double target_speed = max_speed - fabs(steer_value)*speed_reduction;

              //Calculate the CTE for speed
              double speed_cte = speed - target_speed;

              speed_pid.UpdateError(speed_cte);
              throttle_value = speed_pid.TotalError(true, throttle_LowerLimit, throttle_UpperLimit, true, lagcut);
              if (speed<35) { throttle_value = 0.3; }
              speed_twiddle.Add_to_Total_Error(speed_cte);

              if (speed_twiddle.number_of_error_added == 40) {

                speed_Params = speed_twiddle.Optimize_Parameter();
                speed_Kp = speed_Params[0];
                speed_Ki = speed_Params[1];
                speed_Kd = speed_Params[2];
                speed_pid.Init(speed_Kp, speed_Ki, speed_Kd);
              }
            #pragma endregion
          #pragma endregion
                    
          #pragma region DEBUG 
            std::cout << " Steering CTE: " << cte << " Steering Value: " << steer_value;
            std::cout << " Speed CTE: " << speed_cte << " Throttle Value: " << throttle_value;
            std::cout << std::endl;
          #pragma endregion



          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
