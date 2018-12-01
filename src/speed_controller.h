/*
 * state_machine.h
 * Some helper functions for the path planning project.
 *  Created on: Dec 2nd, 2018
 *      Author: Michael Berner
 */

#ifndef SPEED_CONTROLLER_H_
#define SPEED_CONTROLLER_H_

#include <math.h>
#include <fstream>
#include <vector>
#include <iostream>

void speed_controller(vector<double> &cars_s,vector<double> &cars_s_delta,vector<double> &cars_d,
                        vector<double> &cars_v,int &lane_target,double &end_path_s, double &ego_v,
                        double &ego_s, bool &acc_dominant, bool &too_close, int &prev_size,
                        double &ref_v) {

  // find ref_v to use
  for (int j=0; j < cars_s.size(); ++j) {
    // car is in target lane (= my lane for "STAY", neighbour lane for "LCR" or "LCL")
    float d = cars_d[j];
    if (d < (2+4*lane_target + 2) && d > (2+4*lane_target - 2)) {
      double car_s = cars_s[j];
      double car_sd = cars_s_delta[j];
      // only look at vehicles ahead of ego vehicle and within a certain distance
      if ((car_sd > 0) && ((car_sd) < 50) ) {
        // Predict other vehicle position and speed
        double car_v = cars_v[j];
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
            ref_v -= 0.224*3;
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
        cout << " delta_s: " << car_s - ego_s << " delta_s_proj: " << delta_s_proj;
        cout << " , delta_s_dot: " << delta_s_dot << endl;
      }
    }
  }

  // No vehicle ahead: accelerate to full speed
  if ((too_close == false) && (acc_dominant == false)) {
    ref_v = min(49.5, ref_v + 0.224*2);
    cout << "Case 0: no vehicle ahead, full speed - YOLO!" << endl;
  }
}

#endif /* SPEED_CONTROLLER_H_ */
