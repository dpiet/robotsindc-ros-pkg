This ROS package provides teleoperation functionality through any websockets-enabled browser, allowing teleoperation
through a variety of devices such as the iPhone, iPad, Android devices, and PCs.

A tutorial related to this package is available at <a href="http://robotsindc.com/?p=207">RobotsInDC.com</a>

The package includes the following:

Nodes:
roomba400_lightweight -- Provides interface to iRobot 400-series roombas through the SCI specification. Subscribes to
the /cmd_vel topic.
teleop_key -- Standard keyboard teleoperation through command line for debugging/testing/local control.

Control Interface:
teleop.html -- forward-facing page for teleoperation through a browser

ROS package dependencies:
Rosbridge (if using web interface)
cereal_port

To run:

In a terminal:
> roscore
> rosrun roomba_400_series roomba400_light_node (assumes robot is attached to /dev/ttyUSB0)
> rosrun rosbridge rosbridge.py
(Optional) if you don't have a webserver, cd to your /home directory with the teleop.html file and use the magic of python:
> python -m SimpleHTTPServer