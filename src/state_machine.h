/*
 * state_machine.h
 * Some helper functions for the path planning project.
 *  Created on: Dec 2nd, 2018
 *      Author: Michael Berner
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <math.h>
#include <fstream>
#include <vector>
#include <iostream>

void State_Machine(string &state, int &lane, int &lane_target, int &lane_best, double &ego_v,
                    vector<double> &laneScore, vector<bool> &laneFeasibility, double &ego_d,
                    bool &init) {
  if (state == "STAY") {
    // Check if better lane than current one is available & speed is high enough
    if ((lane_best > lane) && (init == false)) {
      state = "LCR-P";
    } else if (lane_best < lane) {
      state = "LCL-P";
    } else {
      lane_target = lane;
      state = "STAY";
    }
  }
  else if (state == "LCL-P") {
    // Check if best lane assumption left-hand-side to current lane is still valid
    if (lane_best < lane) {
      // Check if lane change is feasible. If not, stay with lane change preparation
      if (laneFeasibility[lane - 1]) {
        lane_target = lane - 1;
        state = "LCL";
      } else {
        lane_target = lane;
        state = "LCL-P";
      }
    // Lane change left is not anymore best option
    } else {
      lane_target = lane;
      state = "STAY";
    }
  }
  else if (state == "LCL") {
    // Lane change left is going to happen and stays until finished.
    // Check if lane change is finished
    if ( abs(ego_d - (2+lane_target*4)) < 0.3 ) {
      lane = lane_target;
      state = "STAY";
    } else {
      int a = 1;
    }
  }
  else if (state == "LCR-P") {
    // Check if best lane assumption right-hand-side to current lane is still valid
    if (lane_best > lane) {
      // Check if lane change is feasible. If not, stay with lane change preparation
      if (laneFeasibility[lane + 1]) {
        lane_target = lane + 1;
        state = "LCR";
      } else {
        lane_target = lane;
        state = "LCR-P";
      }
    // Lane change right is not anymore best option
    } else {
      lane_target = lane;
      state = "STAY";
    }
  }
  else if (state == "LCR") {
    // Lane change right is going to happen and stays until finished.
    // Check if lane change is finished
    double target_d = 2+lane_target*4.;
    if (lane_target == 2) {
      target_d -= 0.2;
    }
    if ( abs(ego_d - target_d) < 0.2 ) {
      lane = lane_target;
      state = "STAY";
    } else {
      int a = 1;
    }
  }
  else {
    cout << "Something went wrong with the state machine...!!" << endl;
  }
}

#endif /* STATE_MACHINE_H */
