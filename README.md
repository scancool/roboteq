# Roboteq dual channel ros driver

This is the driver for all dual channel roboteq motor controllers, the driver includes the differential driver algorithm. 

* [youtube video](https://www.youtube.com/watch?v=rk8OZ5LdfDo)

## Getting Started

You will need ROS on your computer and the roboteq microbasic_script running on your motor controller.
Clone this repository in your catkin_ws/src/  folder

```
git clone https://github.com/scancool/roboteq.git
```

## Running the tests

Launch RVIZ by running the command:

```
roslaunch diff_drive rviz.launch
```
on a different terminal run this command to be able to control your robot

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

