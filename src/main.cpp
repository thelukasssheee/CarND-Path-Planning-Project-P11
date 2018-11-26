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
#include "spline.h"     // GPLv2 from https://kluge.in-chemnitz.de/opensource/spline/
#include "helper_functions.h"
#include "cost.cpp"


using namespace std;
using json = nlohmann::json;


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


// Main function
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_WPs_x;
  vector<double> map_WPs_y;
  vector<double> map_WPs_s;
  vector<double> map_WPs_dx;
  vector<double> map_WPs_dy;

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
  	map_WPs_x.push_back(x);
  	map_WPs_y.push_back(y);
  	map_WPs_s.push_back(s);
  	map_WPs_dx.push_back(d_x);
  	map_WPs_dy.push_back(d_y);
  }

  // Create splines interpolation for each map attribute (x,y,dx,dy) in dependence on s
  tk::spline map_s_x, map_s_y, map_s_dx, map_s_dy;
  map_s_x.set_points(map_WPs_s,map_WPs_x);
  map_s_y.set_points(map_WPs_s,map_WPs_y);
  map_s_dx.set_points(map_WPs_s,map_WPs_dx);
  map_s_dy.set_points(map_WPs_s,map_WPs_dy);

  // Start in center lane (lane 1)
  int lane = 1;

  // Reference velocity
  // double ref_v = 49.6; //mph
  double ref_v = 0.0; //mph

  h.onMessage([&map_s_x,&map_s_y,&map_s_dx,&map_s_dy,&ref_v,&lane]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
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

          // --------------------------------------------------------------
          // READ IN SIMULATOR DATA
          // --------------------------------------------------------------
          // Parse ego vehicle's localization Data from JSON object j[1]

        	double ego_x = j[1]["x"];
        	double ego_y = j[1]["y"];
        	double ego_s = j[1]["s"];
        	double ego_d = j[1]["d"];
        	double ego_yaw = j[1]["yaw"];
        	double ego_speed = j[1]["speed"];
        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
          // Determine amount of unprocessed path plan data from previous iteration
          int prev_size = previous_path_x.size();
        	// Previous path's end s and d values
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];
        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	auto sensor_fusion = j[1]["sensor_fusion"];
          vector<double> cars_s, cars_s_delta, cars_d, cars_v, cars_v_delta; // Reset variables
          cout << "Vehicles:";
          for(int j = 0; j < sensor_fusion.size(); j++) {
            double d = sensor_fusion[j][6];
            double sens_vx = sensor_fusion[j][3];
            double sens_vy = sensor_fusion[j][4];
            cars_s.push_back(sensor_fusion[j][5]);
            cars_s_delta.push_back(cars_s[j] - ego_s);
            cars_d.push_back(sensor_fusion[j][6]);
            cars_v.push_back(sqrt(sens_vx*sens_vx + sens_vy*sens_vy));
            cars_v_delta.push_back(cars_v[j] - ego_speed);
            printf(" |%2i: d=%3.1f,ds=%5.1f",j,cars_d[j],cars_s[j]-ego_s);
          }
          cout << endl;

          // --------------------------------------------------------------
          // STATE MACHINE: DECIDE WHAT TO DO
          // --------------------------------------------------------------

          // Detect best lane: check for cars ahead of ego vehicle and their speed
          // Start with lane 0:
          vector<double> laneCost = getBestLane(cars_s_delta, cars_d, cars_v_delta);


          // Sensor fusion input
          bool too_close = false;
          bool acc_dominant = false;

          // find ref_v to use
          for (int j=0; j < sensor_fusion.size(); ++j) {
            // car is in my lane
            float d = sensor_fusion[j][6];
            if (d < (2+4*lane + 2) && d > (2+4*lane - 2)) {
              double vx = sensor_fusion[j][3];
              double vy = sensor_fusion[j][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[j][5];

              // if using previous points can project s value outwards
              check_car_s += ((double)prev_size * 0.02 * check_speed);
              // check s values greater than mine and s gap
              if ((check_car_s > ego_s) && ((check_car_s - ego_s) < 100) ) {
                // Vehicle is within 100m. Reduce speed to match approaching vehicles speed.
                // ref_v = 29.5; // mph
                too_close = true;

                // if (lane > 0) {
                //   lane = 0;
                // }
              }
            }
          }

          if (too_close) {
            ref_v -= 0.224;
          }
          else {
            if (ref_v < 49.5) {
              ref_v += 0.224;
            }
          }



          // --------------------------------------------------------------
          // TRAJECTORY GENERATION: CALCULATE PATH
          // --------------------------------------------------------------



          // Create list with widely spaced waypoints, approximately 30m
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = ego_x;
          double ref_y = ego_y;
          double ref_s = ego_s;
          double ref_d = ego_d;
          double ref_yaw = deg2rad(ego_yaw);
          // If previous size is almost empty, use ego as starting reference
          if(prev_size < 2) {
            // Use current position and yaw angle to calculate tangent to car
            double ref_x_prev = ego_x - cos(ref_yaw);
            double ref_y_prev = ego_y - sin(ref_yaw);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            // double ref_x = ego_x;
            // double ref_y = ego_y;
            // double ref_yaw = deg2rad(ego_yaw);
          }
          // Enough previous points are available. Generate reference from last two points of path
          else {
            // Redefine reference state as previous path end point
            ref_s = end_path_s;
            ref_d = end_path_d;
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            // Also include point right before that time step
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            // Calculate yaw based on these two points
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            // Use last two points to make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            // cout << "Previous x,y: " << ref_x_prev << "," << ref_y_prev << ". ";
            // cout << "Current x,y: " << ref_x << "," << ref_y << "." << endl;
          }
          printf("Ego/ref yaw: %6.2f/%6.2f\n", deg2rad(ego_yaw), ref_yaw);
          printf("Ego s/d: %6.1f/%5.2f\n", ego_s, ego_d);
          printf("Ref s/d: %6.1f/%5.2f\n", ref_s, ref_d);

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(ref_s+ 30,(2+4*lane),map_s_x,map_s_y,map_s_dx,map_s_dy);
          vector<double> next_wp1 = getXY(ref_s+ 60,(2+4*lane),map_s_x,map_s_y,map_s_dx,map_s_dy);
          vector<double> next_wp2 = getXY(ref_s+ 90,(2+4*lane),map_s_x,map_s_y,map_s_dx,map_s_dy);
          // vector<double> next_wp0 = getXY(ego_s+ 30,(2+4*lane),map_s_x,map_s_y,map_s_dx,map_s_dy);
          // vector<double> next_wp1 = getXY(ego_s+ 60,(2+4*lane),map_s_x,map_s_y,map_s_dx,map_s_dy);
          // vector<double> next_wp2 = getXY(ego_s+ 90,(2+4*lane),map_s_x,map_s_y,map_s_dx,map_s_dy);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // Transform spline's base/reference points to car coordinate system
          for (int j = 0; j < ptsx.size(); ++j ) {
            // Transform to car coordinate system (subtract ref x,y global position)
            double shift_x = ptsx[j] - ref_x;   // Ref x,y is at end of previous path (i.e. future)
            double shift_y = ptsy[j] - ref_y;
            // Shift car reference angle to 0 degrees
            ptsx[j] = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            ptsy[j] = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

          cout << "Spline x,y: ";
          for (int j = 0; j < ptsx.size(); ++j ) {
            printf("%4.1f/%4.1f | ", ptsx[j], ptsy[j]);
          }
          cout << endl;
          // cout << "Spline s,d: ";
          // for (int j = 0; j < ptsx.size(); ++j ) {
          //   vector<double> sdconv = getFrenet(ptsx[j],ptsy[j])
          //   printf("%4.1f/%4.1f | ",ptsx[j, ptsy[j]]);
          // }
          // cout << endl;

          // create a spline and add x,y points to it
          tk::spline s;
          s.set_points(ptsx,ptsy);

          // Define the actual x,y points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all the previous path points from the last time
          // Re-use path points
          if (prev_size > 0){
            std::cout << "Reused points: " << prev_size << std::endl;
            for(int j = 0; j < prev_size; j++) {
              next_x_vals.push_back(previous_path_x[j]);
              next_y_vals.push_back(previous_path_y[j]);
            }
          }




          // Add new path points
          // Calculate how to break up spline points so that we travel at desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt( (target_x * target_x) + (target_y * target_y) );
          double x_add_on = 0;

          // Fill up the rest of our path planner after filling it with previous points. Here,
          // we will always output 50 points. Start point is end of previous path, i.e. future!
          // for (int j = 1; j <= 50-prev_size; j++) {
          for (int j = 1; j <= 50-prev_size; j++) {
            double N = (target_dist/(0.02*ref_v/2.24));  // 2.24: conversion mph to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);  // TODO: speed compensation

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // TODO: rotation in every step or rather just once for all?
            // Rotate back to normal after rotating it earlier (conversion back to global coord's)
            x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
            // Translate back to global coordinate system
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          // Prepare JSON object and provide ego path as x,y coordinates to simulator
        	json msgJson;
        	msgJson["next_x"] = next_x_vals;
        	msgJson["next_y"] = next_y_vals;
          // Create message from JSON object and send back to simulator
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
  // program doesn't compile :-(
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
