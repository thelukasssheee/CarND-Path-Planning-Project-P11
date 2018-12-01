/*
 * helper_functions.h
 * Some helper functions for the path planning project.
 *  Created on: Nov 25, 2018
 *      Author: Michael Berner
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <math.h>
#include <fstream>
#include <vector>
#include <iostream>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

// #include <uWS/uWS.h>
// #include <thread>
// #include "spline.h"

using namespace std;


// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif //M_PI

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}


// Signum function
template <typename T> int signum(T val) {
    return (T(0) < val) - (val < T(0));
}


// Calculate distance between two x,y coordinates
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


// Calculate closest waypoint to x,y coordinate
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


// Provide next waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	double heading = atan2((map_y-y),(map_x-x));
	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);
  if(angle > pi()/2)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }
  return closestWaypoint;
}


// Provide next waypoint based on s coordinate
int PrevWaypoint_s(double s, const vector<double> &maps_s)
{
	int prev_wp = -1;
  // wrap around end of list
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
  // return value
	return prev_wp;
}


// Provide next waypoint based on s coordinate
int NextWaypoint_s(double s, const vector<double> &maps_s)
{
	int prev_wp = PrevWaypoint_s(s, maps_s);
  // calculate next waypoint: wrap-around at end of array
  int next_wp = (prev_wp+1)%maps_s.size();
  // return value
	return next_wp;
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  // Get previous and next waypoint
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
  // Prepare variables
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];
	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
  // calculate Frenet d value (lateral distance)
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);
	// correct sign of d value: comparing it to a center point
	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);
	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}
	// calculate Frenet s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}
	frenet_s += distance(0,0,proj_x,proj_y);
  // return Frenet s and d values
	return {frenet_s,frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d,
  tk::spline &s_x, tk::spline &s_y, tk::spline &s_dx, tk::spline &s_dy)
{
  // Get interpolated x,y,dx,dy values from spline by referencing s-value
  double path_x = s_x(s);
  double path_y = s_y(s);
  double dx = s_dx(s);
  double dy = s_dy(s);
  // Calculate final x,y pair by taking lateral displacement d into account
  double x = path_x + d*dx;
  double y = path_y + d*dy;
	return {x,y};
}


// Calculate optimal lane line to pick
vector<double> getBestLane(vector<double>& cars_s_d, vector<double>& cars_d, vector<double>& cars_v_d)
{
  // Initialize lane score with a high number
  vector<double> lane_score;
  int best_lane = 0;
  double best_score = -9999.;
  for (int j=0; j<3; j++){
    lane_score.push_back(10000.);
  }
  if (cars_s_d.size() > 0) {
    double horizon_max = 150.; // meters
    double horizon_min = -20.; // meters

    // Loop through all cars
    for(int j = 0; j < cars_s_d.size(); j++) {
      // Check if vehicle is within horizon
      if ((cars_s_d[j] < horizon_max) && (cars_s_d[j] > horizon_min)) {
        // Check which lane the vehicle is within
        for (int lane = 0; lane < 3; lane++) {
          // Is vehicle within current lane? If not, move on.
          if ((cars_d[j] > lane*4) && (cars_d[j] <= (lane+1)*4)) {
            // Calculate score. Optimization took place with www.desmos.com, see hyperlink
            // https://www.desmos.com/calculator/zqxwm2lmyl - Formula j(x,v)
            // Formula is rather complicated, but makes sure, that slower vehicles ahead and faster
            // vehicles behind get punished. Vice versa, faster vehicles ahead and slower vehicles
            // behind score higher. Highest score is achieved for an empty lane within horizon.
            double dv = cars_v_d[j];
            double ds = cars_s_d[j];
            // double score = a * ds + b * dv*signum(ds) + c * exp(d * ds*ds);
            double score = 0.1*ds+0.8*dv*signum(ds)-1000*exp(-0.04*ds*ds);
            // Add offset for vehicles behind (negative delta_s values)
            if (ds < 0) { score += 7.; }  // variable e_0 in desmos
            // Reduce score to lowest value within lane
            lane_score[lane] = min(lane_score[lane],score);
          }
        }
      }
    }
    for (int lane = 0; lane < 3; lane++) {
      cout << "Lane " << lane << ": ";
      for (int j = 0; j < cars_s_d.size(); j++) {
        if ((cars_d[j] > lane*4) && (cars_d[j] <= (lane+1)*4)) {
          printf(" | %i: dv=%2.0f,ds=%4.0f",j,cars_v_d[j],cars_s_d[j]);
        }
      }
      cout << " || Lane score = " << lane_score[lane] << endl;
      if (lane_score[lane] > best_score) {
        best_score = lane_score[lane];
        best_lane = lane;
      }
    }
  }
  return lane_score;
}


// Calculate optimal lane line to pick
vector<bool> getLaneFeasibility(vector<double>& cars_s_d, vector<double>& cars_d, vector<double>& cars_v_d)
{
  // Initialize lane score with a high number
  vector<double> lane_score;
  int best_lane = 0;
  double best_score = -9999.;
  for (int j=0; j<3; j++){
    lane_score.push_back(10000.);
  }
  if (cars_s_d.size() > 0) {
    double horizon_max =  40.; // meters
    double horizon_min = -20.; // meters

    // Loop through all cars
    for(int j = 0; j < cars_s_d.size(); j++) {
      // Check if vehicle is within horizon
      if ((cars_s_d[j] < horizon_max) && (cars_s_d[j] > horizon_min)) {
        // Check which lane the vehicle is within
        for (int lane = 0; lane < 3; lane++) {
          // Is vehicle within current lane? If not, move on.
          if ((cars_d[j] > lane*4) && (cars_d[j] <= (lane+1)*4)) {
            // Calculate score. Optimization took place with www.desmos.com, see hyperlink
            // https://www.desmos.com/calculator/7qhdrupbli - Formula j(x,v)
            // Formula is rather complicated, but makes sure, that slower vehicles ahead and faster
            // vehicles behind get punished. Vice versa, faster vehicles ahead and slower vehicles
            // behind score higher. Highest score is achieved for an empty lane within horizon.
            double dv = cars_v_d[j];
            double ds = cars_s_d[j];
            // double score = a * ds + b * dv*signum(ds) + c * exp(d * ds*ds);
            double score = 0.15*ds+0.5*dv*signum(ds)-1000*exp(-0.04*ds*ds);
            // Add offset for vehicles behind (negative delta_s values)
            if (ds < 0) { score += 7.; }  // variable e_0 in desmos
            // Reduce score to lowest value within lane
            lane_score[lane] = min(lane_score[lane],score);
          }
        }
      }
    }
  }
  // Create boolean output vector
  vector<bool> lane_feasibility(3);
  for (int lane = 0; lane < 3; lane++) {
    if (lane_score[lane] > 0) {
      lane_feasibility[lane] = true;
    }
    else {
      lane_feasibility[lane] = false;
    }
  }
  return lane_feasibility;
}

#endif /* HELPER_FUNCTIONS_H_ */
