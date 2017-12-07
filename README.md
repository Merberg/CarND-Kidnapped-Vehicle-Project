# Overview
This repository contains the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.  Using the [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases), this project simulates a robot that has been kidnapped and transported to a new location. The robot has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.  A 2 dimensional particle filter will use map, observation, and control data to determine the robot's location.

---

## Dependencies
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

## Basic Build Instructions
Scripts have been included to streamline the build process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

## Inputs to the Particle Filter
The inputs to the particle filter are in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.



