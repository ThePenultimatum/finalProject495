# Face Painter
-----------------------------------------------------------------------------------------
#### ME-495 Final Project, Fall 2018, Northwestern University
#### Group members:Mark Dyehouse, Veronica Medrano, Huan Weng, Chenge Yang, Guo Ye
-----------------------------------------------------------------------------------------
## Introduction

### Objective
### Perception
### Trajectory Planning
### Robot Control
-----------------------------------------------------------------------------------------
## Implementation

### Launch file
### [Nodes][scripts]
* asasas
### Topics
### Messages
### External Packages

# Connect to Sawyer
cd sawyer_ws/
./intera.sh

# Sawyer tutorial
rosrun intera_examples joint_torque_springs.py

# Open camera
rosrun intera_examples camera_display.py -c right_hand_camera -x 8
  -h, --help            show this help message and exit
  -c {head_camera,right_hand_camera}, --camera {head_camera,right_hand_camera}
                        Setup Camera Name for Camera Display
  -r, --raw             Specify use of the raw image (unrectified) topic
  -e, --edge            Streaming the Canny edge detection image
  -g GAIN, --gain GAIN  Set gain for camera (-1 = auto)
  -x EXPOSURE, --exposure EXPOSURE
                        Set exposure for camera (-1 = auto)

# Calibrate camera
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera_driver/image_raw camera:=/camera_driver

# AR Tag
roslaunch me495_vision ar_tag.launch
Topics:
Remap from:
/camera_image
/camera_info
To:
/io/internal_camera/right_hand_camera/camera_info
/io/internal_camera/right_hand_camera/image_raw
