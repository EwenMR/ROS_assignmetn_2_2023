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

To start the ROS master or core:
```bash
roscore
```

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
## How to set or cancel a new target
Upon launching the assignment2 launch file the user should be prompted to enter two coordinates:
```bash
x,y
```

or to enter:
```bash
s
```
to stop the robot and cancel the goal currently set.

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

# Pseudocode for the action client
```
Define ActionClient class:
    Initialize action client for 'reaching_goal' server
    Initialize publishers to /pos and /goal and subscribe to 'odom' topic

    Define odom_callback function:
        Extract position and velocity from odometry
        Create and publish Posvel message

    Define send_goal function:
        Create goal with target coordinates and send to action server
        Publish current goal/target coordinates
        Log goal information

    Define cancel_goal function:
        Cancel all goals
        Log cancellation information

    Define listen_goals function:
        Continuously prompt user for input
        If 's' is entered, cancel goals
        If valid coordinates are entered, send goal
        Handle invalid input gracefully

Define main function:
    Initialize ROS node and ActionClient
    Start a separate thread for listening to user input and sending goals
    Keep the node running
    Wait for the goal thread to finish before exiting
```
