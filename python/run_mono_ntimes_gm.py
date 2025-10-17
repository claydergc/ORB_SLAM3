#!/home/claydergc/miniforge3/bin/python3

import subprocess
import signal

# --- Configure rectangle region ---
width, height = 1500, 920        # size of capture area
x, y = 0, 0                 # top-left corner
fps = "10"
folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/GM"
output_file = ""
folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/GMTrajectories/KeyFrameTrajectory"

#subprocess.run("export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ros-noetic/Pangolin-0.9.2/build")
#program = ["./Examples/Monocular/mono_tum", "./Vocabulary/ORBvoc.txt", "./Examples/Monocular/TUM1.yaml", "/home/ros-noetic/datasets/Polcam/S11_20250822/00/polcamI90", "./WLightI90"]
program = ["/home/claydergc/OneDrive/MyDockerPackages/ros-noetic/run_docker_orb_slam3_mono.sh", "", ""]

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


for i in range(1):

    program[1] = "/home/ros-noetic/datasets/Polcam/GM/polarimetric_imaging_dataset/20220621_132710/RGB_undistorted_padded"
    
    folder_video = "/home/claydergc/Videos/Polcam_ORB_SLAM3/GM/20220621_132710/run"
    
    folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/GMTrajectories/KeyFrameTrajectory"
        
    output_file = folder_video + str(i) + ".mkv"
    record_cmd[12] = output_file 
    rec_proc = subprocess.Popen(record_cmd)
    program[2] = folder_trajectory_file + str(i) + ".txt"
        # Run your C++ program
    prog_proc = subprocess.run(program)
        # Stop recording once program finishes
    rec_proc.send_signal(signal.SIGINT)
    rec_proc.wait()

