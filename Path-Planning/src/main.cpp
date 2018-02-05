/**
https://github.com/darienmt/CarND-Path-Planning-Project-P1/blob/master/src/main.cpp
https://github.com/toluwajosh/CarND-Path-Planning-Project/blob/master/src/main.cpp
**/
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

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
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

	return {frenet_s,frenet_d};

}

float findCarLane(float d)
{
  int car_lane = -1;

  if ( d > 0 && d < 4 ) {
    car_lane = 0;
  } else if ( d > 4 && d < 8 ) {
    car_lane = 1;
  } else if ( d > 8 && d < 12 ) {
    car_lane = 2;
  }  
  return car_lane;
}

bool IsCarOnMyCloseLeft(float my_lane, float others_lane, float other_dist, float my_dist)
{ 
  if (abs(my_lane - others_lane)==1)
  {
    if(my_lane > others_lane)
    {
      // check no other car 100 meter infront of you in target lane
      // check no other car 50 meter behind you in target lane
      if (((other_dist > my_dist) && abs(other_dist-my_dist)<30) || ((other_dist < my_dist) && abs(other_dist-my_dist)<20))
      {
        cout << "another car close on my left lane..." << endl;
        return  true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return false;
}

bool IsCarOnMyCloseRight(float my_lane, float others_lane, float other_dist, float my_dist)
{ 
  if (abs(my_lane - others_lane)==1)
  {
    if(my_lane < others_lane)
    {
      // check no other car 100 meter infront of you in target lane
      // check no other car 50 meter behind you in target lane
      if (((other_dist > my_dist) && abs(other_dist-my_dist)<30) || ((other_dist < my_dist) && abs(other_dist-my_dist)<20))
      {
        cout << "another car close on my right lane..." << endl;
        return  true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return false;
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1; // middle lane
  double ref_vel = 0.0; //mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // TRY: 1
            // We will set the points 0.5 m apart. Since the car moves 50 times a second, a distance of 0.5m per 
            // move will create a velocity of 25 m/s. 25 m/s is close to 50 MPH.
            // Keep going straight with constant velocity at the car's current angle
            // Problem: going 0->50 mph instantly
            // double dist_inc = 0.5;
            // for(int i = 0; i < 50; i++)
            // {
            //   next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
            //   next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
            // }

            // TRY: 2
            // This code snippet starts the new path with whatever previous path points were left over from the last cycle. 
            // Then we append new waypoints, until the new path has 50 total waypoints.
            // Using information from the previous path ensures that there is a smooth transition from cycle to cycle. But the 
            // more waypoints we use from the previous path, the less the new path will reflect dynamic changes in the environment.
            // Ideally, we might only use a few waypoints from the previous path and then generate the rest 
            // of the new path based on new data from the car's sensor fusion information.
            // double pos_x;
            // double pos_y;
            // double angle;
            // int path_size = previous_path_x.size();

            // for(int i = 0; i < path_size; i++)
            // {
            //     next_x_vals.push_back(previous_path_x[i]);
            //     next_y_vals.push_back(previous_path_y[i]);
            // }

            // if(path_size == 0)
            // {
            //     pos_x = car_x;
            //     pos_y = car_y;
            //     angle = deg2rad(car_yaw);
            // }
            // else
            // {
            //     pos_x = previous_path_x[path_size-1];
            //     pos_y = previous_path_y[path_size-1];

            //     double pos_x2 = previous_path_x[path_size-2];
            //     double pos_y2 = previous_path_y[path_size-2];
            //     angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            // }

            // double dist_inc = 0.5;
            // for(int i = 0; i < 50-path_size; i++)
            // {    
            //     next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
            //     next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
            //     pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            //     pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            // }

            // TRY: 3
            // Using Frenet coordinates instead of cartesian
            // double dist_inc = 0.3; // 0.5 originally
            // for(int i = 0; i < 50; i++)
            // {
            //   double next_s = car_s + (i+1)*dist_inc;
            //   double next_d = 6;
            //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //   next_x_vals.push_back(xy[0]);
            //   next_y_vals.push_back(xy[1]);
            // }

            // TRY: 4
            // Use spline with Frenet Coordinates
            int prev_size =  previous_path_x.size(); // last path that car followed

            // cout << "prev_size = " << prev_size << endl;
            // Preventing collisions
            if(prev_size > 0)
            {
              car_s = end_path_s;
            }

            bool car_ahead_30 = false;
            bool car_ahead_50 = false;

            // find ref_v to use
            for (int i = 0; i < sensor_fusion.size(); ++i)
            {              
              // car is in my lane
              float d = sensor_fusion[i][6]; // d-value of ith car
              float car_lane = findCarLane(d);

              // if(d < (2+4*lane+2) && d > (2+4*lane-2))
              // 1. if car is in my lane and too close (ahead of me)
              if (car_lane == lane)
              {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // if using previous points, we can project s value outward is time
                check_car_s += ((double)prev_size*0.02*check_speed);

                // check s values greater than mine and s gap
                // if ith car (in future) is going to be in close proximity (< 30m) of my car (in future), 
                // then I need to take action
                if((check_car_s > car_s))
                {
                  // we can lower reference velocity so we dont crash into the car infront of us
                  // we could also flag to try to change lanes
                  // ref_vel = 29.5; //mph
                  if ((check_car_s-car_s) < 30)
                    car_ahead_30 = true;

                  if ((check_car_s-car_s) < 50)
                    car_ahead_50 = true;
                }

              }// if d
            }// for i
            // cout << "car_ahead_30 = " << car_ahead_30 << endl;

            bool car_left = false;
            bool car_right = false;

            if(car_ahead_30){
              for (int i = 0; i < sensor_fusion.size(); ++i)
              {
                float d = sensor_fusion[i][6]; // d-value of ith car
                float car_lane = findCarLane(d);  

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // if using previous points, we can project s value outward is time
                check_car_s += ((double)prev_size*0.02*check_speed);

                // check if this car is in my left lane
                if (IsCarOnMyCloseLeft(lane, car_lane, check_car_s, car_s)){
                  car_left = true;
                  cout << "found car on left..." << endl;
                }

                // check if the car is in my right lane 
                if (IsCarOnMyCloseRight(lane, car_lane, check_car_s, car_s)){
                  car_right = true;
                  cout << "found car on right..." << endl;
                }
              }
            }

            if(car_ahead_30)  {

              cout << "\n--------------" << endl;
              cout <<  "car_left = " << car_left << ", car_right = " << car_right  << endl;
              cout << "--------------" << endl;

              if(!car_left && lane>0)
              {
                cout << "NO CAR ON LEFT..." << endl;
                lane -= 1; // move to the adjacent left lane
              }
              else if(!car_right && lane<2)
              {
                cout << "NO CAR ON RIGHT..." << endl;
                lane += 1; // move to the adjacent right lane
              }
              else
              {
                // ~5 meter/sec
                cout << "NO option but to reduce speed right away..." << endl;
                ref_vel -= 0.224; // reduce ref-velocity if too close to another car and cannot change lane
              }
            }
            else if (ref_vel < 49.5 && !car_ahead_50) // speed up
            {
              ref_vel += 0.224; // accelerate slowly every cycle, if ref_vel is low
              ref_vel = min(ref_vel, 49.5);
            }


            //Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // Later we shall interpolate these points with a spline and fill it in with more points 
            // that control speed 
            vector<double> ptsx;
            vector<double> ptsy;

            // reference x,y, and yaw
            // the car's reference is its previous path end point (or starting point in the very beginning)
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // if previous path is almost empty, use the car as starting reference
            if(prev_size < 2)
            {
              // use 2 points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            else // Use previous path's end point as starting reference
            {
              // redefine reference state as previous path's end point
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Use the last 2 points to make the path tangent to the previous path's end point
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            // In Frenet, add evenly 30m spaced points ahead of the starting reference
            // see "lane" parameter being used here...
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            // So we have 5 points in total now: 2 prev points, and location of car at 30m, 60m, and 90m.

            // Transformation of local car coordinates: using car's frame of reference
            // The last point of previous path is at (0,0) at an angle 0.
            // Makes math easy
            for (int i = 0; i < ptsx.size(); ++i)
            {
              //shift car reference angle to 0 degree
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }

            // create a spline
            tk::spline s;

            // set (x,y) points to the spline
            s.set_points(ptsx, ptsy);

            // Start with all of the previous path points from last time
            for(int i = 0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }            

            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y); // car is at (0,0)

            double x_add_on = 0;

            // Fill up the rest of our path planner after filling it with previous points,
            // here we wil always output 50 points
            for (int i = 1; i < 50-previous_path_x.size(); ++i)
            {
              // see "ref_vel" parameter being used here...
              double N = (target_dist/(0.02*ref_vel/2.24)); //2.24 to convert miles/hr -> meter/sec
              double x_point = x_add_on + (target_x)/N; // x_point -> tickmark on the x-axis on image shown
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to normal after rotating it earlier
              x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
