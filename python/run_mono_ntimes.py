#!/home/claydergc/miniforge3/bin/python3

import subprocess
import signal

# --- Configure rectangle region ---
width, height = 1420, 920        # size of capture area
x, y = 0, 0                 # top-left corner
fps = "20"
folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI0"
output_file = ""
folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI90/KeyFrameTrajectory"

#subprocess.run("export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ros-noetic/Pangolin-0.9.2/build")
#program = ["./Examples/Monocular/mono_tum", "./Vocabulary/ORBvoc.txt", "./Examples/Monocular/TUM1.yaml", "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI90", "./WLightI90"]
program = ["/home/claydergc/OneDrive/MyDockerPackages/ros-noetic/run_docker_orb_slam3_mono.sh", "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI0", "/home/ros-noetic/src/ORB_SLAM3/WLightI0"]

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


for i in range(4):

    if i==0:
        program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI0"
        folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI0/WLightI0_"
        folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI0/KeyFrameTrajectory"
    elif i==1:
        program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI45"
        folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI45/WLightI45_"
        folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI45/KeyFrameTrajectory"
    elif i==2:
        program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI90"
        folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI90/WLightI90_"
        folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI90/KeyFrameTrajectory"
    elif i==3:
        program[1] = "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI135"
        folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/polcamI135/WLightI135_"
        folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/TrajectoryFiles/WLightI135/KeyFrameTrajectory"

    for j in range(20):
        
        output_file = folder_video + str(j) + ".mkv"
        record_cmd[12] = output_file 
        rec_proc = subprocess.Popen(record_cmd)
        program[2] = folder_trajectory_file + str(j) + ".txt"
        # Run your C++ program
        prog_proc = subprocess.run(program)
        # Stop recording once program finishes
        rec_proc.send_signal(signal.SIGINT)
        rec_proc.wait()

