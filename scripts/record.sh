#!/bin/sh

rosbag record /front_right_radar_data /gps_data /car_data /usb_cam/camera_info /usb_cam/image_raw -o /media/nvidia/SIMMAX/bagfiles/gps_pose_est
