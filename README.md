
# An Optimal Local Path Planning in High Speed Scenario

This repo is for an optimal local path planning algorithm deployed in F1tenth Rviz.

## Run the Rviz simulator
```roslaunch f110_simulator simulator.launch```
### Run your Follow the wall
```roslaunch f110_simulator wall_follow.launch```
### Run your Global Path tracking
```roslaunch f110_simulator stanlley_controller.launch```
### Run your Local Path tracking
```roslaunch f110_simulator frenet_controller.launch```

## Follow the wall
The wall following algorithm is to implement a Proportional Integral Derivative (PID) controller to make the F1tenth
drive parallel to the walls of a corridor at a fixed distance.
![Follow the wall](https://github.com/TongshenH/F1tenth/blob/main/result/wall_follow.gif)
## Global Path planning
Based on the predefined way points, the Stanley Controller is deployed to track the global path.
![Global Path Tracking](https://github.com/TongshenH/F1tenth/blob/main/result/stanley_03k.gif)
## Local Path planning  
An optimal local path planning has been designed in the Frenet frame, then the best path is selected by the cost function.
![Local Path planning](https://github.com/TongshenH/F1tenth/blob/main/result/costfn1.gif)
## Perception
The max gap is to find the maxmium gap range ahead of the vehicle.
![Max gap](https://github.com/TongshenH/F1tenth/blob/main/result/max_gap1.png)
## Integration
This part integrate Auto Emergency Brake (AEB), path planning and Perception together. 
![Integration](https://github.com/TongshenH/F1tenth/blob/main/result/final.gif)