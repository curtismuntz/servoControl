Custom Arduino PID servo control algorithm for brushed DC motor position control along with MATLAB analysis.

Two separate implementations are explored in this repository.

IMU Feedback
============
    servoControl Arduino code to run servo based on IMU feedback sensor.

Visual Feedback:
================
    Built on top of ROS (ros.org), ROS nodes are used for visual feedback system.
        * image_sender node
            - basic camera driver that broadcasts the raw image frames to a ROS topic
        * ojbect_locator node
            - Listens for incoming raw images, locates orange objects within their frames, and selects the center of the largest contiguous orange object in the frame to broadcast as a measurement to visualServo.ino. Color is determined using intensity thresholding. Object size is determined by using contour area properties.
            - basic prototype algorithm can be seen under objectLocator.m
        * visualServo.ino node
            - Arduino sketch that communicates with ROS using rosserial. This node computes the PID output of the system and sends motor commands to keep the orange object in the center of the screen.