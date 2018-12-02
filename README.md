# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Project summary / write-up
by Michael Berner, Student @ Udacity, March 2018 class

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

### Overview
The task in this project was to create a path planning algorithm, which is capable to drive a vehicle around a highway-like track within a simulator. 

[![Project 11 - Path Planning, Overview](./media/intro_overview.png)](./media/intro_overview.png)

Most important topics, which needed to be addressed:

- Collision-free drive
- Steer and drive vehicle without exceeding acceleration and jerk limitations
- Lane-change, if a faster lane is available

A complete list of topics to solve can be found in Udacity's [project rubric](https://review.udacity.com/#!/rubrics/1971/view) and below.

### Solution

The project was solved with the following main features:

- Implementation of a State Machine
- Splines to calculate the trajectory (path planning) 
- Interpolation of map data
- Cost function to identify best lanes
- Cost function to detect lane occupation 
- Speed controller /w Adaptive cruise control

I created a [Youtube video](https://youtu.be/p9Mxwi5ICnw) of my solution, as well. 

### Code structure

| ```./src/main.cpp``` | Main script |
| ```./src/speed_controller.h``` | ACC speed controller |
| ```./src/spline.h``` | Spline library from Tino Kluge (C), GNU GPL |
| ```./src/state_machine.h``` | State machine with states "STAY", "Lane Change left" etc. |

### Algorithm

The path planning algorithm in ```main.cpp``` is working in the following logic / order:

**1. Initialization phase**: All sensor signals from the simulator are being read in. Until vehicle has exceeded a first time 40mph, lane change is prohibited using an init flag (see lines until 156).
  
**2. Lane selection**: At first, two important inputs are being calculated. Which lane is the best in terms of being not jammed or fastest (line 169 in ```main.cpp``` & ```helper_functions.h``` line 184). Additionally, it is important to know which lanes are currently safe to drive on. This happens in line 174 (```main.cpp```) and line 241 (```helper_functions.h```). 

**3. State machine:** Based on these inputs, the state machine decides which are the next actions to take (e.g. prepare a lane change, stay in lane etc.). 

**4. Vehicle speed controller**: Vehicle speed is being adjusted with the speed controller. It is reacting on vehicles ahead, which are driving slower either on the current lane or target lane (in case of a planned lane change). Speed is reduced / increased and a safe distance is maintained.

**5. Trajectory generation**: Using splines and several inputs from the preceding functions, the trajectory is being calculated. It is tuned in such a way, that acceleration and jerk limitations are not violated. Pipeline size is 50 datapoints, which are processed by the simulator with a frequency of 50Hz (20ms / datapoint). This means the next second will always be provided to the simulator to compensate lag and generate a smoother trajectory.  

### Lane selection / cost functions

It took quite some time to find the ideal score function, which allows fast & safe travel of the vehicle. 

I considered three aspects for each lane: at what distance and speed compared to the ego vehicle are other vehicles. Closer vehicles are more dangerous. Vehicles ahead with much slower speeds are also dangerous. Vehicles behind, which are driving slower, are not issue at all. 

An extremely helpful browser based tool was the [Desmos Graphing Calculator](https://www.desmos.com/calculator).

Below, one of the applied score/cost functions is visualized.

[![Score function - visualized with Desmos graphic calculator](./media/cost_function.png)](./media/cost_function.png)

The x axis shows distance to other vehicles (always measured ```other car - ego vehicle```, thus: positive x values means other vehicles are ahead). The vertical axis shows the score, with higher values expressing favorable lanes. 

Within a horizon of [-20m...+150m], **all** vehicles are being assessed for each lane. If multiple vehicles are within one lane, the car scoring lowest is used for evaluation (see line 217 in ```helper_functions.h```) since the slowest vehicle might have an impact on the other cars of its lane in the near future.

For me, it was especially important to identify the areas right next to the ego vehicle as absolute no-go-zones, in case there is another vehicle. This was achieved by adding a exponential function with a high weight (around x=0), as you can see on the image above.  

### Finite state machine 

During the course of this project, I noticed quickly that behavior planning can become incredibely messy rather quickly. With all the negative consequences, such as unexpected behavior as the most important one. Therefore, a state machine similar to the one from the Udacity lesson was implemented.  

[![Finite State Machine - from Udacity course material](./media/finite_state_machine.png)](./media/finite_state_machine.png)

This was extremely helpful to make sure, that lane changes are actually executed when it is safe and until they are completely finished. All the code is stored in a separate helper function ```state_machine.h```. With less than 100 lines of code, a clear and structured vehicle behavior was achievable within this project. 




# Original Readme by Udacity

   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

