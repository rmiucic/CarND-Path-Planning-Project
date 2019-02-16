# CarND-Path-Planning-Project
[//]: # (Image References)
[map]: ./sparse_map_points.png "Map Points"
[sim]: ./simulation_pass.png "Simulation Pass"
[fsm1]: ./FSM.png "FSM1"
[lane]: ./lane_logic.png "Lane Logic"
[fsm2]: ./FSM2.png "fsm2"
[complete]: ./lane_change_complete.png "Lane Change Complete"
[image7]: ./examples/right.png "Right Flipped"
[image8]: ./examples/right_fliped.png "Right Flipped"
[image9]: ./examples/learning.png "Learning"


Self-Driving Car Engineer Nanodegree Program<br/>
![alt text][sim]<br/>
## Basic Build Instructions
1. Clone this repo.
2. go to src directory: `cd CarND-Path-Planning-Project/src`
2. the CMakeLists.txt is in /src directory
3. Compile: `make`
4. Run it: `./path_planning`.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals Acheived
See picture at the top
1. My car safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
2. I use Host Vehicle (HV) localization and sensor fusion data about the other Remote Vehicles (RVs) 
3. I also take advantage of provided sparse map list of waypoints around the highway. 
4. My HV stays close as possible to the 50 MPH speed limit, 
5. My HV is passing slower traffic when possible
6. HV avoids hitting other cars and  drives inside of the marked road lanes at all times, unless it is changing lanes.
7. My HV is able to make one complete loop around the 6946m highway and more. 
8. My HV is not exceeding total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Details
1. I started the code per class instructional video (https://youtu.be/7sI3VHFPP0w). 

2. Trajectory generation was done by building a spline (code from https://kluge.in-chemnitz.de/opensource/spline/)
the spline consisted of 5 points. In Fenet coordinates points are: previos_previous, previous, 30m ahead, 60m ahead, and 90m ahead. The trajectory was interpolated from the spline between 0 and 30 meters. Number of interpolated points depends on the taget speed `ref_vel` (which is ~49.5MPH if there are no RVs in front or lower value if there are RV in front). 

3. On to of wich I added a first version of Finite State Machine (FSM1) for lane change logic. FSM1 had 3 states: Lane 1, Lane 2, and Lane 3, coresponding to the lanes of the road. <br/>
![alt text][fsm1]<br/>
Basically this logic was trying to keep the HV in lane if there is no RV in front in the same lane for some distance  `inlane_ahead_zone` (set at 30.0m) or there are RVs in front lane and in avaliable adjacent lanes. RV is considered to be in adjacent lane if it is within `ahead_zone` (set at 35.0m) or `behind_zone` (set at 10.0m). The picture below shows variable involved in calculation.<br/>
![alt text][lane]<br/>
4. I quickly realized that only 3 states was not sufficient because the vehicle would not complete lane change fully before going back and forth between the lanes . Therefore I have introduce transitional states (L0_L1, L1_L0, L1_L2, L2_L1) in second version of Finite State Machine (FSM2).<br/>
 ![alt text][fsm2]<br/>
5. A sucessfull lane change in FSM2 means that the vehicle lateral offset is very close to lane center `lane_center-center_offset < HV_d < lane_center+center_offset`.<br/>
![alt text][complete]<br/>

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.<br/>
![alt text][map]<br/>
The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.





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

I have used Netbeans for debugging. In experience, Netbeans is much better (more stable, easier to use) than Eclipse. 

