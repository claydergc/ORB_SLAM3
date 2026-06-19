import subprocess

executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam"
vocabulary = "/home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt"
settings = (
    "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml"
)
imgsCam0 = (
    "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI0_1224x1024/"
)
imgsCam1 = (
    "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/polcamI45_1224x1024/"
)

trajectoryFileFolder = "/home/ros-noetic/src/ORB_SLAM3_polcam/results/"


# program[3] = "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839"
# program[3] = "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0830"


# program[1] = "/home/ros-noetic/datasets/Polcam/Yandiwanba/20260128/20260128_0839"
# folder_video = "/media/claydergc/DATA/Videos/20260128/20260128_0839/polcamI0_I90/run_"
# folder_trajectory_file = "/home/ros-noetic/src/ORB_SLAM3/results/Trajectories/20260128/20260128_0839/polcamI0_I90/KeyFrameTrajectory"


# for j in range(10):
for j in range(2, 3):
    print(f"Running iteration {j}")

    trajectoryFile = trajectoryFileFolder + str(j).zfill(5) + "_KeyFrameTrajectory.txt"

    program = [
        executable,
        vocabulary,
        settings,
        imgsCam0,  # imgs cam0
        imgsCam1,  # imgs cam1
        trajectoryFile,  # output trajectory file
    ]

    # Run your C++ program
    prog_proc = subprocess.run(program)
