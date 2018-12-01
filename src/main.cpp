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
  bool init = true;
  int lane = 1;
  int lane_target = 1;
  int lane_best = 1;
  // Define current state
  string state = "STAY";

  // Reference velocity
  // double ref_v = 49.6; //mph
  double ref_v = 0.0; //mph

  h.onMessage([&map_s_x,&map_s_y,&map_s_dx,&map_s_dy,&ref_v,&lane,&lane_target,&lane_best,
               &state,&init]
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
        	double ego_v = j[1]["speed"];
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
          // cout << "Vehicles:";
          for(int j = 0; j < sensor_fusion.size(); j++) {
            double d = sensor_fusion[j][6];
            double sens_vx = sensor_fusion[j][3];
            double sens_vy = sensor_fusion[j][4];
            cars_s.push_back(sensor_fusion[j][5]);
            cars_s_delta.push_back(cars_s[j] - ego_s);
            cars_d.push_back(sensor_fusion[j][6]);
            cars_v.push_back(sqrt(sens_vx*sens_vx + sens_vy*sens_vy));
            cars_v_delta.push_back(cars_v[j] - ego_v/2.24);
            // printf(" |%2i: d=%3.1f,ds=%5.1f",j,cars_d[j],cars_s[j]-ego_s);
          }
          if (init) {
            lane = (int)round((ego_d-2)/4);
            lane_target = lane;
            lane_best = lane;
            if (ego_v > 40) {
              init = false;
              cout << "INITIALIZATION DONE!";
            }
          }
          cout << endl;


          // --------------------------------------------------------------
          // STATE MACHINE & BEST LANE DETECTION
          // --------------------------------------------------------------
          // Detect best lane: check for cars ahead of ego vehicle and their speed in each lane
          vector<double> laneScore = getBestLane(cars_s_delta, cars_d, cars_v_delta);
          double max_val = *std::max_element(laneScore.begin(),laneScore.end());
          int max_indx = std::max_element(laneScore.begin(),laneScore.end())-laneScore.begin();
          if ((max_val > laneScore[lane]) && (max_indx != lane)) {
            lane_best = max_indx;
          }
          cout << "Best lane = " << lane_best << " ";
          cout << "with score " << laneScore[lane_best] << ". " << endl;

          // Check lane feasibility before state machine is fired up
          // Important: only one lane may be changed at a time. Lane needs to be empty beforehand.
          vector<bool> laneFeasibility = getLaneFeasibility(cars_s_delta, cars_d, cars_v_delta);
          // Call state machine with 5 possible states: "STAY", "LCL", "LCL-P", "LCR", "LCR-P"
          State_Machine(state,lane,lane_target,lane_best,ego_v,laneScore,laneFeasibility,ego_d,init);
          cout << "State '" << state << "'. Curr Lane --> Target Lane = " ;
          cout << lane << "-->" << lane_target << endl;

          // --------------------------------------------------------------
          // VEHICLE SPEED CONTROLLER: SET REFERENCE SPEED TRAFFIC DEPENDENT
          // --------------------------------------------------------------
          bool too_close = false;
          bool acc_dominant = false;

          // find ref_v to use
          for (int j=0; j < sensor_fusion.size(); ++j) {
            // car is in target lane (= my lane for "STAY", neighbour lane for "LCR" or "LCL")
            float d = sensor_fusion[j][6];
            if (d < (2+4*lane_target + 2) && d > (2+4*lane_target - 2)) {
              double car_s = sensor_fusion[j][5];
              // only look at vehicles ahead of ego vehicle and within a certain distance
              if ((car_s > ego_s) && ((car_s - ego_s) < 50) ) {
                // Predict other vehicle position and speed
                double car_vx = sensor_fusion[j][3];
                double car_vy = sensor_fusion[j][4];
                double car_v = sqrt(car_vx*car_vx + car_vy*car_vy);
                double car_s_proj = car_s + ((double)prev_size * 0.02 * car_v/2.24);
                // Predict ego position and speed
                double ego_s_proj = end_path_s; // last point of previous points array
                // If vehicle is within ACC dominant distance range, adapt vehicle speed
                double delta_s_proj = car_s_proj - ego_s_proj;
                double delta_s_dot = delta_s_proj - (car_s - ego_s);
                double safety_dist = ego_v/1.9; // conversion to mph = x/2.24, t = x*2s --> 1.12
                // Vehicle within safety distance
                if (delta_s_proj < safety_dist) {
                  too_close = true;
                  acc_dominant = true;
                  if (delta_s_dot > 0) {
                    // Within safety distance, but distance increasing:
                    // Increase speed until ego is almost car speed to achieve target distance
                    ref_v = min((ref_v + 0.224*2), car_v * 0.95);
                    cout << "Case 1:";
                  } else {
                    // Within safety distance and distance is further decreasing:
                    // Reduce speed drastically
                    ref_v -= 0.224;
                    cout << "Case 2:";
                  }
                } else if (delta_s_proj >= safety_dist){
                  if (delta_s_dot > 0) {
                    acc_dominant = true;
                    // Outside safety distance and distance is further increasing (vehicle too slow)
                    // Increase speed so that ego is slightly faster than other car
                    ref_v = min((ref_v + 0.224*2), car_v * 1.05);
                    cout << "Case 3:";
                  } else {
                    // Outside safety distance, but distance further decreasing
                    // Do nothing, continue approaching the car
                    cout << "Case 4:";
                  }
                }
                cout << " delta_s: " << car_s - ego_s << " delta_s_proj: " << delta_s_proj << " , delta_s_dot: " << delta_s_dot << endl;
              }
            }
          }


          // No vehicle ahead: accelerate to full speed
          if ((too_close == false) && (acc_dominant == false)) {
            ref_v = min(49.5, ref_v + 0.224*2);
            cout << "Case 0: no vehicle ahead, full speed - YOLO!" << endl;
          }


          // --------------------------------------------------------------
          // TRAJECTORY GENERATION: CALCULATE PATH
          // --------------------------------------------------------------

          // Check if an elongated trajectory from lane change is still in place. If not, update
          // current lane and continue generating waypoints
          // if (prev_size > 50) {
          //
          // } else {
            // Create list with widely spaced waypoints, approximately 30m
            vector<pair<double,double>> spline_pts;

            double ref_x = ego_x;
            double ref_y = ego_y;
            double ref_s = ego_s;
            double ref_d = ego_d;
            double ref_yaw = deg2rad(ego_yaw);
            // If previous size is almost empty, use ego as starting reference
            if (prev_size < 3) {
              // Use current position and yaw angle to calculate tangent to car
              double ref_x_prev = ego_x - cos(ref_yaw);
              double ref_y_prev = ego_y - sin(ref_yaw);
              spline_pts.push_back(make_pair(ref_x_prev, ref_y_prev));
              spline_pts.push_back(make_pair(ref_x,      ref_y));
              cout << "Fallback path calculation: not enough previous waypoints. " << endl;
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
              spline_pts.push_back(make_pair(ref_x_prev, ref_y_prev));
              spline_pts.push_back(make_pair(ref_x,      ref_y));
            }
            printf("Ego/ref yaw: %6.2f/%6.2f\n", deg2rad(ego_yaw), ref_yaw);
            printf("Ego s/d: %6.1f/%5.2f\n", ego_s, ego_d);
            printf("Ref s/d: %6.1f/%5.2f\n", ref_s, ref_d);

            // In Frenet add evenly 30m spaced points ahead of the starting reference, but dependent
            // on target lane and current lane.
            double lane_d = 2. + 4.*lane_target;
            // In case target lane is 3, drive a little more inwards
            if (lane_target == 2) {
              lane_d -= 0.2;
            }
            vector<double> next_wp0 = getXY(ref_s+ 30,lane_d,map_s_x,map_s_y,map_s_dx,map_s_dy);
            vector<double> next_wp1 = getXY(ref_s+ 60,lane_d,map_s_x,map_s_y,map_s_dx,map_s_dy);
            vector<double> next_wp2 = getXY(ref_s+ 90,lane_d,map_s_x,map_s_y,map_s_dx,map_s_dy);
            spline_pts.push_back(make_pair(next_wp0[0], next_wp0[1]));
            spline_pts.push_back(make_pair(next_wp1[0], next_wp1[1]));
            spline_pts.push_back(make_pair(next_wp2[0], next_wp2[1]));

            // Transform spline's base/reference points to car coordinate system
            // for (int j = 0; j < ptsx.size(); ++j ) {
            for (int j = 0; j < spline_pts.size(); ++j ) {
              // Transform to car coordinate system (subtract ref x,y global position)
              double shift_x = spline_pts[j].first - ref_x;   // Ref x,y is at end of previous path (i.e. future)
              double shift_y = spline_pts[j].second - ref_y;
              // Shift car reference angle to 0 degrees
              spline_pts[j].first = (shift_x * cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
              spline_pts[j].second = (shift_x * sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }
            // Make sure that first two spline points are adjacent to each other
            if ((spline_pts[1].first - spline_pts[0].first) < 0.001) {
              spline_pts[0].first -= 1.;
            }

            // Sort points in ascending order to prevent crash due to signal noise at low ego speeds
            sort(spline_pts.begin(),spline_pts.end());
            // Prepare spline x,y arrays and output values
            cout << "Spline x,y: ";
            vector<double> spline_xpts(spline_pts.size()), spline_ypts(spline_pts.size());
            for (int j = 0; j < spline_pts.size(); ++j ) {
              spline_xpts[j] = spline_pts[j].first;
              spline_ypts[j] = spline_pts[j].second;
              printf("%4.1f/%4.1f | ", spline_pts[j].first, spline_pts[j].second);
            }
            cout << endl;

            // Create a spline and add x,y points to it
            tk::spline s;
            s.set_points(spline_xpts,spline_ypts);

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
            // Differentiate between lane change and "STAY" situations
            for (int j = 1; j <= 50-prev_size; j++) {
              double N = (target_dist/(0.02*ref_v/2.24));  // 2.24: conversion mph to m/s
              double x_point = x_add_on + target_x / N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

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
        // }
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
