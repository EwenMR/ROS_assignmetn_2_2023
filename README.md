# Research Track 2 second assignment using ROS and Jupyter Notebook (2024)
This repository contains all the necessary code to download and run the ros simulation with the use of the User Interface.

# Assignment
- (a) Create an interface using Jupyter Notebook to assign (or cancel) goals to the robot.
- (b) A plot with the robot's current position and current targets' positions in the environment.
- (c) A plot for the number of reached versus not-reached targets.

# Installing and Running the code
## Installing
You will need to clone this repository,
```bash
git clone https://github.com/EwenMR/assignment_2_2023.git
```
## Running
We want to go into the ros workspace first:
```bash
cd ros_ws
```

You must build the workspace using:
```bash
catkin_make
```

You then have to source the ros workspace:
```bash
source devel/setup.bash
```

Then we need to enter the correct branch
```bash
cd assignment_2_2023
git checkout RT2_asssignment2
```

After this we can set up the jupyter notebook:
```bash
jupyter notebook --allow-root --ip 0.0.0.0
```
Once the connection established, find and open the User_Interface file.

To launch Rviz and Gazebo:
```bash
roslaunch assignment_2_2023 assignment1.launch
```

To launch the action client and the service server nodes, with the choice of the size of the window for node (c)(replace 'X' with the desired window size):
```bash
roslaunch assignment_2_2023 assignment2.launch window_size:=X
```

If it is so that you are experiencing any issues regarding the execution of the new nodes it may be due to the permissions of the files, in which case you have to make them executable:
```bash
cd src/assignment_2_2023/scripts
chmod +x *
```

# Functionalities
Once the ROS environment launched you are to run the User Interface.

## How to set or cancel a new target
Use the two buttons for x and y, once the coordinates entered simply press 'Set Goal'.

To stop the robot and cancel the goal currently set simply press the 'Cancel Goal' button.

## Retrieving the last target set by the user
This can be done by calling the /get_last_target service:
```bash
rosservice call /get_last_target
```

## Retrieving the distance of the target from the robot's position (dist) with the average speed of the robot (mu):
This can be done calling another service /get_dist_mu:
```bash
rosservice call /get_dist_mu
```


