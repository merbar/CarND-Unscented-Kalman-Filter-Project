# Unscented Kalman Filter
## Udacity Self-Driving Car Engineer Nanodegree Program

Implementation of an Unscented Kalman Filter to track a detected object using the CTRV motion model. The data sources are simulated radar and lidar measurements, where the radar data differentiates itself by also containing the object's radial velocity.

Sample data for Radar and Lidar input was provided by Mercedes Benz.

This is part of a short series of projects exploring different Kalman filtering methods. The other is:
* [Extended Kalman Filter](https://github.com/merbar/CarND-Extended-Kalman-Filter-Project)

## Implementation

Implementation of the EKF is mostly straightforward with emphasis on readability over speed.  

The challenge here was to initialize everything in a reasonable way and tweak the process noise parameters to beat a certain required accuracy threshold (see table below).  

I used Normalized Innovation Squared to make sure my filter is consistent.

## Results
RMSE = Residual Mean Squared Error  
| RMSE goal | RMSE UKF | RMSE UKF (lidar only) | RMSE UKF (radar only) |
| --- | --- |  --- | --- |
| *0.09*  |  **0.0685** |  0.108 |  0.156 |
|  *0.10*  |  **0.0827** |  0.098 |  0.240 |
|  *0.40*  |  **0.3329** |  0.615 |  0.393 |
|  *0.30*  |  **0.2333** |  0.263 |  0.291 |


### Position accuracy: 
![Position accuracy](img/fusion_acc_p.png)

### Velocity accuracy: 
![Velocity accuracy](img/fusion_acc_v.png)

### Yaw accuracy: 
![Yaw accuracy](img/fusion_acc_yaw.png)

### Normalized Innovation Squared:
![NIS](img/fusion_nis_comp.png)

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
