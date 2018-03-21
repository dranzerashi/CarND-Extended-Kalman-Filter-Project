# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 



INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---
[result]: ./ashi.PNG "Result"
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `


## Results
The following result is obtained on the run of the Extended Kalaman Filter on the simulator. The table shows the Root Mean Squared Error result at the end of the run for the positions px, py and velocities vx,vy.

| Input  | RMSE   |
| -------|:------:| 
| px     | 0.0974 |
| py     | 0.0855 |
| vx     | 0.4517 |
| vy     | 0.4404 |

![alt text][result]
