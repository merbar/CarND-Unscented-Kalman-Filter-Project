# Unscented Kalman Filter
Udacity Self-Driving Car Engineer Nanodegree Program

Implementation of an Unscented Kalman Filter using the CTRV motion model.
Sample data for Radar and Lidar input was provided by Mercedes Benz.

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Implementation

Implementation is mostly straightforward with emphasis on readability over speed.
The challenge here was to initialize everything in a reasonable way and tweak the process noise parameters to beat a certain required accuracy threshold.
I used Normalized Innovation Squared to make sure my filter is consistent. Visualization for all output is contained in a separate iPython notebook `ukf-visualization.ipynb`.

## Results

Reference-style: 
![alt text](img/fusion_acc_p.png)
