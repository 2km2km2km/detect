#!/bin/bash

# 设置录制的rosbag文件名
BAG_NAME="bug.bag"

# 设置要录制的topic
TOPIC="/target_dist_calculator/detect_out /cloud_registered_body /detection/usb_cam/annotated_image /detection/usb_cam/bbox_info /detection/usb_cam1/annotated_image /detection/usb_cam1/bbox_info /detection/usb_cam2/annotated_image /detection/usb_cam2/bbox_info /detection/usb_cam3/annotated_image /detection/usb_cam3/bbox_info /Odometry"

# 启动rosbag录制
echo "Recording topic $TOPIC into $BAG_NAME"
rosbag record -O $BAG_NAME $TOPIC

# 提示用户录制完成
echo "Recording finished. Saved to $BAG_NAME"

