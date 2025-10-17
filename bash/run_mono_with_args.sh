#!/bin/bash

ARG1=$1 #folder with images
ARG2=$2 #trajectory file

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ros-noetic/Pangolin-0.9.2/build


#echo "The argument passed is: $1"


#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml /home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI0 "$ARG2"

/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml "$ARG1" "$ARG2"

#/home/ros-noetic/src/ORB_SLAM3/WLightI90/KeyframeTrajectory.txt
