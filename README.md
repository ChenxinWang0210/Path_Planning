# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview

The goal of this project is to implement a  C++ path planning  algorithm to  safely drive a car around a virtual highway in a [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) provided by Udacity .  The car's localization, sensor fusion and previous path data is sent from the simulator to the C++ program. A sparse map list of waypoints around the highway is also given to the C++ program. The path planning algorithm uses this information to generate trajectories for the car to follow.

Several requirements from Udaciy are listed as follows.

* * The car should try to go as close as possible to the 50 MPH speed limit
* The car should avoid collision
* The car should drive inside the lane, except for changing lanes
* The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3
* The car should be able to make one complete loop around the 6946m highway without any incidents listed above.

This project meets all the requirement as shown in the video

![](path_planning.gif)

![](path_planning_new.gif)


## Implementation

 

### Prediction

The C++ program recieves localization, sensor fusion and previous path data from the simulator. It uses this data to predict other car's locations. For example, the ego car predicts other cars' postions with the following codes

```
float d = sensor_fusion[i][6];
double vx = sensor_fusion[i][3];
double vy = sensor_fusion[i][4];
double check_speed = sqrt(vx*vx + vy*vy);
double check_car_s = sensor_fusion[i][5];
check_car_s +=((double)prev_size*0.02*check_speed);

```
The value of "d" indicates which lane other car is located. The "check_car_s" is the predicted Frenet s location.   

### Behavior Planning

If other car is in the ego car's lane  and ahead of the ego car, the ego car will check whether it is following too close to that car. If it is following too close, the ego car will either change lanes or slow down. If it is safe to change lanes, the ego car will perform lane change; otherwise, it will slow down every path-planning cycle.  

As to changing lanes, the following policy is implemented

* If it is safe to change to only one of the left or right lane, make this change.
          
* If both of changing left and right are feasible, check which one is better according to their scores. If the target lane ahead of the car is clear, a high score is given. A left clear lane is given a higher score than a right clear lane. If there is traffic ahead of the car in the target lane, the score is given based on the lowest traffic speed in that lane. Pick a lane with higher score. 

When the ego car is not following too close and the speed is below the target speed, the ego car will increase speed every path-planning cycle.  I tried a quick accelerating or slowing down algorithm in the code line 570-585, but it did not work well. The ego car experienced fast slowing down and accelerating frequently when it moved close to a car and was not able to change lanes. It was not a good driving behavior, so I dropped it.

The code lines 355-462 implement the behavior planning algorithm 

### Trajectory Generation 

The trajecotry generation follows Udacity [Walk-through video](https://www.youtube.com/watch?v=7sI3VHFPP0w). The code lines 470-607 do the job.  The general idea is as follows.

* The last two points from the previous trajectory (or the car position if there are no previous trajectory left) together with three evenly 30m spaced points ahead are used to create a spline. To make the math easier, these points are transformed to car coordinates for generating spline function. 

* For path continuity (less jerk), the previous path points are added in the trajectory for the current path-planning cycle.  There are 50 points in the trajectory for each path-planning cycle. Assume there are n previous path points. The rest (50-n) points are generated from the spline function and transformed from car coordinates to global map coordinates. 

## Build and Run 

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Note: Need to refer to [Udacity README](README_Udacity.md) on how to connect the C++ program with the simulator

## Authors

* Chenxin Wang

Thanks for the contributions from Udacity





