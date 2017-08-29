#define _USE_MATH_DEFINES

#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid_steer;
  PID pid_throttle;
  // Initialize the pid variable.
  double Kp = atof(argv[1]);
  double Kd = atof(argv[2]);
  double Ki = atof(argv[3]);
  double max_speed = atof(argv[4]);
  pid_steer.Init(Kp, Kd, Ki);
  
  int htime = 5;
  //max_speed = 100;
  //pid_steer.Init(0.1, 1.0, 0.01);
  
  pid_throttle.Init(0.2, 0.3, 0.0);

  std::string out_file_name = "Outfile.csv";
  bool write_out_file = false;
  std::ofstream out_file(out_file_name.c_str(), std::ofstream::out);
  if (write_out_file) {
    // column names for output file
    out_file << "timestep" << ";";
    out_file << "cte" << ";";
    out_file << "speed" << ";";
    out_file << "angle" << ";";
    out_file << "steer_value" << ";";
    out_file << "throttle_value" << "\n";
  }

  h.onMessage([&pid_steer, &pid_throttle, &max_speed, &htime, &out_file, &write_out_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (pid_steer.timestep == 0) {
      std::string reset_msg = "42[\"reset\",{}]";
      ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
    }

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
          double throttle_value;
          
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // Control Steering
          pid_steer.UpdateError(cte, htime);
          steer_value = -pid_steer.TotalError();
          //std::cout << pid_steer.timestep << ": cte = " << cte << "; " << steer_value << "; " << pid_steer.p_error << "; " << pid_steer.d_error << "; " << pid_steer.i_error << std::endl;

          if (steer_value > 1.0) {
            steer_value = 1.0;
          }
          else if (steer_value < -1.0) {
            steer_value = -1.0;
          }
          
          if (pid_steer.timestep > 100){
            // calculate error
            pid_steer.error += cte*cte;
          }

          // DEBUG
          std::cout << pid_steer.timestep << ";" << pid_steer.error << std::endl;
          //std::cout << "##################### " << pid_steer.timestep << " #####################" << std::endl;
          //std::cout << "Kp: " << pid_steer.Kp << " Kd: " << pid_steer.Kd << " Ki:" << pid_steer.Ki << std::endl;
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << std::endl << std::endl;

          // Control Throttle
          double throttle_cte = speed - max_speed; // constant speed 10.0
          pid_throttle.UpdateError(throttle_cte, 5);
          throttle_value = -pid_throttle.TotalError();

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          //msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (write_out_file) {
            out_file << pid_throttle.timestep << ";";
            out_file << cte << ";";
            out_file << speed << ";";
            out_file << angle << ";";
            out_file << steer_value << ";";
            out_file << throttle_value << "\n";
          }          
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
  if (h.listen("127.0.0.1", port)) // for windows
  //if (h.listen(port)) // for ubuntu
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
