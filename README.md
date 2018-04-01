# Extended Kalman Filter Project

This project implements a kalman filter to estimate the state of a moving object of interest with noisy lidar 
and radar measurements. The project is based on udacity's starter code for project extended kalman filter
in the self-driving car engineer nanodegree program. 

# Project files
The following source files are (mainly) contained in this project:
* main.cpp: receives sensor measurements from the simulator in a loop and uses FusioneEKF
* FusionEKF.cpp: processes the sensor measurements and uses KalmanFilter
* kalman_filter.cpp: implements a kalman filter for predicting next state and updating 
using new measurements from lidar and radar. Holds the current state vector.
* tools.cpp: implements RMSE calculation and first order Jacobian matrix to be used for radar measurements.
* Eigen/*: the eigen library for handling vectors and matrices

# Project results

On the given datasets in udacity simulator (with version 1.45 of 15.6.2017) the following results 
for root mean squared error (RMSE) were achieved:
* dataset 1: rmse=(px=0.0974,py=0.0855,vx=0.4517,vy=0.4404)
* dataset 2: rmse=(px=0.0726,py=0.0965,vx=0.4219,vy=0.4937)

# Simulator Setup (Prerequisites) 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that must be used to set up and install 
[uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 
For windows you can use either Docker, VMware, or even 
[Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

