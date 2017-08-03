#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = (j[1]["speed"]); 
          // v *= 0.447;// mph to m/s       

          // Transform the ptsx, ptsy to car coordinate system
          // TODO: implement in separate function
          // Convert to the vehicle coordinate system
          Eigen::VectorXd x_car(ptsx.size());
          Eigen::VectorXd y_car(ptsx.size());          
          for (int i = 0; i < ptsx.size(); i++) {
            double dtx = ptsx[i] - px;
            double dty = ptsy[i] - py;
            
            x_car[i] = dtx * cos(psi) + dty * sin(psi);
            y_car[i] = dty * cos(psi) - dtx * sin(psi);            
          }          

          // fit a polynomial to the above x and y coordinates
          // The x and y coordinates are contained in the ptsx and ptsy vectors. Since these are 2-element 
          // vectors a 1-degree polynomial (straight line) is sufficient.          
          auto coeffs = polyfit(x_car, y_car, 3);

          // Put ptsx and ptsy data into vectors
          // Eigen::VectorXd ptsxvec = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
          // Eigen::VectorXd ptsyvec = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

          // TODO: calculate the cross track error
          // Because points were transformed to vehicle coordinates, x & y equal 0 below.
          // 'y' would otherwise be subtracted from the polyeval value          
          // double cte = polyeval(coeffs, x) - y;
          double cte = polyeval(coeffs, 0);
          // TODO: calculate the orientation error
          // Recall orientation error is calculated as follows eψ=ψ−ψdes, where ψdes is can be calculated as arctan(f′(x)).
          // f(x)=a0 +a1∗x
          // f′(x)=a1
          // Because x = 0 in the vehicle coordinates, the higher orders are zero leaves only coeffs[1]
          double epsi =  -1*atan(coeffs[1]);

          // Do not send current state to solver, predict 100ms into future and use that state
          // Eigen::VectorXd state(6);
          // state << x, y, psi, v, cte, epsi;

          // predict the state 100ms into the future before you send it to the solver in order to compensate for the latency.
          // Latency of 100ms, so predict 100ms (0.1s) ahead          
          double dt = 0.1;

          // Previous steering angle and throttle
          double steering_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];
          // Predict (x, y and psi are all zero after transformation above)
          double predicted_x = v * dt; // Since psi is zero, cos(0) = 1, can leave out
          double predicted_y = 0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
          double predicted_psi = - v * steering_angle * dt/ Lf ;
          double predicted_v = v + throttle * dt;
          double predicted_cte = cte + v * sin(epsi) * dt;
          double predicted_epsi = epsi + predicted_psi;
          
          Eigen::VectorXd state(6);
          // state << predicted_x, predicted_y, predicted_psi, predicted_v, predicted_cte, predicted_epsi;
          state << 0, 0, 0, predicted_v, predicted_cte, predicted_epsi;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          */
          auto result = mpc.Solve(state, coeffs);

          // Steering must be divided by deg2rad(25) to normalize within [-1, 1].
          // Multiplying by Lf takes into account vehicle's turning ability          
          double steer_value = result[0]*1.0/ deg2rad(25);//*Lf);
          double throttle_value = result[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // steer_value = min(max(steer_value, -0.001), 0.001);
          // throttle_value = min(max(throttle_value, -0.1), 0.1);
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"] =  throttle_value;

          //Display the MPC predicted trajectory 
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals(mpc.mpc_x.size());
          vector<double> mpc_y_vals(mpc.mpc_x.size()); 
          for (int i = 0; i < mpc.mpc_x.size(); i++) {
            mpc_x_vals[i] = mpc.mpc_x[i];
            mpc_y_vals[i] = mpc.mpc_y[i];            
          }        

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(x_car.size());
          vector<double> next_y_vals(x_car.size());

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (int i = 0; i < x_car.size(); i++) {
            next_x_vals[i] = x_car[i];
            next_y_vals[i] = y_car[i];
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
