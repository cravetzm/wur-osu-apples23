# Field Trial Code for 2023. WUR and OSU Collaboration.

## Requirements

### ROS2, Moveit2 and UR Drivers
All development for this project was done in ROS2 Rolling. Other ROS2 distributions may work, but compatability is not guaranteed.

### Apple Messages
Custom services that accompany this package can be found in the apple_msgs package: https://github.com/cravetzm/apple_msgs

These services are required in order to run the code in this repository.

### Tree Sensorization

To log data about branch dynamics, follow the steps in https://github.com/markfrosty/Tree-Sensorization/tree/main. Your host computer should be able to view the topics published in Docker, but you should confirm by running ``` ros2 topic list ``` in a native terminal. 

## Current Issues

Several functions need to be added to the main script, including:

* Reading from the probe
* Reading from the digital calipers
* Logging the pose of the end-effector/  configuration of the robot

## Usage

### Step 1: Bring Up Robot

Bring up the robot by running 

```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=YOUR_ROBOT_IP launch_rviz:=false initial_joint_controller:=forward_position_controller 
```

For the OSU robots, the robot IP is 169.254.177.231 or 169.254.177.232. 

It is often helpful to run the robot in headless mode using headless_mode:=true. However, note that if you are doing so, you must re-connect to the teach pendant via ros service calls whenever you switch to local mode. It is highly recommended that you instead keep the robot set to "automatic" and simply press play after sending commands via teach pendant.

In another terminal, start MoveIt2 by running

```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

The robot should now be displayed in RViz in the same configuration as the real robot.

### Step 2: Activate Controllers

First we will need to activate the forward_position_controller. In a third terminal, run:

```
ros2 control switch_controllers --deactivate [...] --activate forward_position_controller
```

A call to ``` ros2 control list_controllers ``` should show the forward_position_controller as active

Next, we must activate the servo node. In the same terminal, call the activation service using:

```
 ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}

```

### Step 3: Make sure tree sensorization modules are connected.

Once again, the user is referred to https://github.com/markfrosty/Tree-Sensorization/tree/main

### Step 4: Run Control & Data Logging Script

First, launch the service client and custom controller node by running:

```
ros2 launch wur_osu_apples23 launch_picking_trials.py
```

Unfortunately, running nodes from launch files prevents them from recieving user input from the terminal. We will have to open one more terminal. 

We are now ready to begin our picking experiments. Navigate to the folder where you want the experiment data to be stored. Then, source the setup file for the workspace in which you have this package. Finally, run:

```
ros2 run wur_osu_apples23 run_trials
```

Follow the prompts to perform the picking trials.
