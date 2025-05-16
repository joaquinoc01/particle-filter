# Particle Filter Localization

A simple particle filter implementation for 2D robot localization with known landmarks. The robot moves along a predefined path and updates its estimated position using sensor measurements.

## Features

- Particle filter based localization  
- Motion and measurement updates with Gaussian noise  
- Resampling based on particle weights  
- Works with 2D robot moving in a known landmark map  

## Requirements

- C++17 compatible compiler  
- CMake  
- Eigen3 (for matrix and vector math)  

## Build Instructions

mkdir build
cd build
cmake ..
make
./particle_filter

You can customize simulation parameters in `particle_filter.cpp` and `particle_filter.hpp`.

## Usage

Run the executable and observe the console output showing the robot's true state and estimated state at each step.

Example output:

Robot state: x = 1.97409, y = 1, theta = 0  
Estimated state: x = 1.95571, y = 0.983919, theta = -0.00251366  
...  
Robot state: x = 2.38807, y = 1.45584, theta = 6.11056  
Estimated state: x = 2.39347, y = 1.44747, theta = -0.14269

## Description

This particle filter implementation simulates a robot moving forward and turning in a square pattern. At each movement step, it takes noisy sensor measurements of landmarks and updates the particle weights accordingly. Resampling is done to focus particles in areas of higher probability.
