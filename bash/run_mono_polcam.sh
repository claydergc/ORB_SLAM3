#!/bin/bash

#ARG1=$1 #folder with images
#ARG2=$2 #trajectory file

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ros-noetic/Pangolin-0.9.2/build


#echo "The argument passed is: $1"

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Apt/20251002_1805 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TUM1.yaml /home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI0 "$ARG2"

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_17-14_56_57/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_17-16_06_27/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_17-14_52_39/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_22-18_52_56/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_22-15_05_48/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/20250925_1/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/20251002_1805/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/20251002_1805/polcamI90 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Apt/20251003_1/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20250924_3/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20250924_2/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1721/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1723/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015_1723/polcamI135 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015_1723/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015_1723/DoLP_AoLP KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015_1718/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI135 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI135 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI140 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI172_OK KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI172 KeyframeTrajectory.txt


#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/polcamI160 KeyframeTrajectory.txt


#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725/DoLP_AoLP KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251015/20251015_1725 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251201/20251201_1602 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251204/20251204_1527 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251204/20251204_1533 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251208/20251208_1746 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20251211/20251211_1629 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20260120/20260120_0943 KeyframeTrajectory.txt

/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20260128/20260128_0830 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20260128/20260128_0835 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/Yandiwanba/20260128/20260128_0839 KeyframeTrajectory.txt

#/home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/mono_tum_polcam /home/ros-noetic/src/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/ros-noetic/src/ORB_SLAM3/Examples/Monocular/TRIO50S_606x507.yaml /home/ros-noetic/datasets/Polcam/GardensPoint/20260128/20260128_1603 KeyframeTrajectory.txt


#/home/ros-noetic/src/ORB_SLAM3/WLightI90/KeyframeTrajectory.txt
