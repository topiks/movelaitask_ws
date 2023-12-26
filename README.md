# movelaitask_ws

This repository using #turtlebot3 ros package from ROBOTIS to provide the robot phisics including sensors and actuators also its world on gazebo 11 simulation. All of the task result is explained in [this](https://drive.google.com/file/d/1G33DJiKuEHJNVGI5gunHwJYG51DkUTYg/view?usp=sharing) document

### Structure of Packages
```
|-- src
    |-- turtlebot3 (ROBOTIS)
    |-- turtlebot3_msgs (ROBOTIS)
    |-- turtlebot3_simulations (ROBOTIS)
    |-- movement_controller (ME)
```

### In movement_controller Package there are 3 Nodes
```
|-- movement_controller
    |-- control_pid (c++)
    |-- control_pure_pursuit (c++)
    |-- simplify_path (py)
```

## Installation
All of the installation steps is documented in this [Video](https://drive.google.com/file/d/1DZTHz19P_wwNGgBiONNE56TjIumUCMtw/view?usp=sharing) or follow the installation steps below
## How to Create Docker Container
```
git clone https://github.com/topiks/movelaitask_ws.git
cd path/movelaitask_ws
sudo docker build --no-cache -t ros_taufik .
xhost +
sudo docker compose up -d
```

## How to Launch PID Controller
```
sudo docker exec -it ros bash
roslaunch movement_controller pid.launch
```
![img](https://github.com/topiks/movelaitask_ws/blob/main/img/pid.png "pid controller") \
Here is the [Video](https://drive.google.com/file/d/1PvAO-_YjhpiayiwcvIm8lM3yrm1fiWzl/view?usp=sharing) Result Documentations

## How to Launch Pure Pursuit Controller
```
sudo docker exec -it ros bash
roslaunch movement_controller pure_pursuit.launch
```
![img](https://github.com/topiks/movelaitask_ws/blob/main/img/pure_pursuit.png "pure pursuit controller") \
Here is the [Video](https://drive.google.com/file/d/11jex7njR655JeiUE1KLEwod_3mLI6823/view?usp=sharing) Result Documentations
## How to Change Waypoints Path
```
cd path/moveltask_ws/src/movement_controller/param
# edit waypoint_param.yaml, each points is in cm, then save
sudo docker build --no-cache -t ros_taufik .
sudo docker compose up -d
```
## How to Launch Simplify Path Node
```
sudo docker exec -it ros bash
roslaunch movement_controller simpath.launch

# in other terminal tab
sudo docker exec -it ros bash
# publish number of output path points, adjusted
rostopic pub /filtered_path std_msgs/Int64 "data: 50"
```
![img](https://github.com/topiks/movelaitask_ws/blob/main/img/simplify_path.png "simplify path") \
Here is the [Video](https://drive.google.com/file/d/16KxoG0dSvYe09SOHVrp3sLFT5nw5whLe/view?usp=sharing) Result Documentations
## Seirios Demo Video
The Seirios Demo Video can be accessed here [Video](https://drive.google.com/file/d/1sFHoIaDMiL08yOQbOp_7Ec_2LuyTFyC3/view?usp=sharing) 
