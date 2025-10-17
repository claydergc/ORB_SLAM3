#!/home/claydergc/miniforge3/bin/python3

import subprocess
import signal

# --- Configure rectangle region ---
width, height = 1420, 920        # size of capture area
x, y = 0, 0                 # top-left corner
fps = "15"
folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI0"
output_file = ""
folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI90/KeyFrameTrajectory"

#subprocess.run("export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ros-noetic/Pangolin-0.9.2/build")
#program = ["./Examples/Monocular/mono_tum", "./Vocabulary/ORBvoc.txt", "./Examples/Monocular/TUM1.yaml", "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI90", "./WLightI90"]
#program = ["/home/claydergc/OneDrive/MyDockerPackages/ros-noetic/run_docker_orb_slam3_mono.sh", "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI0", "/home/ros-noetic/src/ORB_SLAM3/WLightI0"]

program = ["/home/claydergc/OneDrive/MyDockerPackages/ros-noetic/run_docker_container_orb_slam3_mono.sh", "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI0", "/home/ros-noetic/src/ORB_SLAM3/WLightI0"]

# Start ffmpeg recording for rectangle
record_cmd = [
    "ffmpeg",
    "-loglevel", "quiet",
    "-y",
    "-video_size", f"{width}x{height}",
    "-framerate", fps,
    "-f", "x11grab",
    "-i", f":1+{x},{y}",
    output_file
]

        
#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250822/WLight/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI/WLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250822/WOLight/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3_0/WOLight/polcamI/WOLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/S11_20250822/WOLightI/KeyFrameTrajectory"


#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250903/WLight/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3_1/polcamI/WLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/S11_20250903/WLightI/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250903/WOLight/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3_1/WOLight/polcamI/WOLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/S11_20250903/WOLightI/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250904/WLight/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3_2/WLight/polcamI/WLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/S11_20250904/WLightI/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250904/WLight2/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3_2/WLight2/polcamI/WLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/S11_20250904/WLightI2/KeyFrameTrajectory"


#program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250904/WOLight/polcamI"
#folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3_2/WOLight/polcamI/WOLightI_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/S11_20250904/WOLightI/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/S11/VaryingLight/20250906/polcamI"
#folder_video = "/home/claydergc/Videos/20250906_Polcam_ORB_SLAM3/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20250906/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/Apt_202509091410_T1_0/polcamI"
#folder_video = "/home/claydergc/Videos/Apt_202509091410_T1_0/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/Apt_202509091410_T1_0/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/Apt_202509091420_T2_1/polcamI"
#folder_video = "/home/claydergc/Videos/Apt_202509091420_T2_1/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/Apt_202509091420_T2_1/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/Apt_202509091635_T2_1/polcamI"
#folder_video = "/home/claydergc/Videos/Apt_202509091635_T2_1/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/Apt_202509091635_T2_1/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/Apt_202509091530_T2_1/polcamI"
#folder_video = "/home/claydergc/Videos/Apt_202509091530_T2_1/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/Apt_202509091530_T2_1/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_22-15_05_48/polcamI"
#folder_video = "/home/claydergc/Videos/2025_09_22-15_05_48/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/2025_09_22-15_05_48/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/rosbags/2025_09_22-18_52_56/polcamI"
#folder_video = "/home/claydergc/Videos/2025_09_22-18_52_56/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/2025_09_22-18_52_56/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/20250925_1/polcamI"
#folder_video = "/home/claydergc/Videos/20250925_1/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20250925_1/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/20250925_0/polcamI"
#folder_video = "/home/claydergc/Videos/20250925_0/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20250925_0/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/20250925_2/polcamI"
#folder_video = "/home/claydergc/Videos/20250925_2/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20250925_2/KeyFrameTrajectory"

#program[1] = "/home/ros-noetic/datasets/Polcam/Apt/20251002_1805/polcamI"
#folder_video = "/home/claydergc/Videos/20251002_1805_bottom/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20251002_1805_bottom/KeyFrameTrajectory"
#folder_video = "/home/claydergc/Videos/20251002_1805/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20251002_1805/KeyFrameTrajectory"
#folder_video = "/home/claydergc/Videos/20251002_1805_top/polcamI/run_"
#folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20251002_1805_top/KeyFrameTrajectory"

program[1] = "/home/ros-noetic/datasets/Polcam/Yandiwanba/20251015_1723/polcamI45"
folder_video = "/home/claydergc/Videos/20251015_1723_Yandiwanba/polcamI45/run_"
folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20251015_1723_Yandiwanba/polcamI45/KeyFrameTrajectory"

for j in range(20):
#for j in range(0,4):
    
    output_file = folder_video + str(j) + ".mkv"
    record_cmd[12] = output_file 
    rec_proc = subprocess.Popen(record_cmd)
    program[2] = folder_trajectory_file + str(j) + ".txt"
    # Run your C++ program
    prog_proc = subprocess.run(program)
    # Stop recording once program finishes
    rec_proc.send_signal(signal.SIGINT)
    rec_proc.wait()
