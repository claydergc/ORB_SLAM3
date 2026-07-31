#!/bin/bash

#ARG1=$1 #folder with images
#ARG2=$2 #trajectory file

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ros-noetic/Pangolin-0.9.2/build


#echo "The argument passed is: $1"

/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam4  \
/home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt \
/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml \
/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI0_1224x1024 \
/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024 \
/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI90_1224x1024 \
/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI135_1224x1024 \
45 \
90 \
-45 \
45 \
KeyFrameTrajectory.txt \
FrameTrajectory.txt \
FrameKeypointsNumber.txt


# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam3  \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI90_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI0_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024 \
# KeyFrameTrajectory.txt \
# FrameTrajectory.txt \
# FrameKeypointsNumber.txt

#
# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam  \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024_2.yaml \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI90_1224x1024/ \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI_1224x1024/ \
# KeyframeTrajectory.txt \
# KeyFrameKeypointsNumber.txt


# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam2  \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI0_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI45_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI90_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI135_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI_1224x1024 \
# KeyframeTrajectory.txt \
# KeyFrameKeypointsNumber.txt

# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam2  \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt \
# /home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI0_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI45_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI90_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI135_1224x1024 \
# /home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/polcamI_1224x1024 \
# KeyframeTrajectory.txt \
# KeyFrameKeypointsNumber.txt
