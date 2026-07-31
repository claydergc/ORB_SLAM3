import os
import subprocess

# executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam2"
executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam3"
# executable = "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/mono_tum_polcam4"
vocabulary = "/home/ros-noetic/src/ORB_SLAM3_polcam/Vocabulary/ORBvoc.txt"
settings = (
    "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024.yaml"
    # "/home/ros-noetic/src/ORB_SLAM3_polcam/Examples/Monocular/TRIO50S_1224x1024_2.yaml"
)

# base_path = "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0830/"
# base_path = "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0839/"
base_path = "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/"
# base_path = "/home/ros-noetic/datasets/Polcam02/Yandiwanba/20251015/1725/"
resultsBaseFolder = "/home/ros-noetic/src/ORB_SLAM3_polcam/results/"

# Extract location and datetime from base_path
# Format: .../Location/YYYYMMDD/HHMM/
path_parts = base_path.rstrip("/").split("/")
location = path_parts[-3]  # KelvinGrove
date = path_parts[-2]  # 20260128
time = path_parts[-1]  # 0839

# Create parent folder name: first letter of location + date + time
location_initial = location[0].lower()  # 'k' from KelvinGrove
parent_folder_name = f"{location_initial}_{date}_{time}_3pxKptsOverlap"
# parent_folder_name = f"{location_initial}_{date}_{time}_2pxKptsOverlap"
# parent_folder_name = f"{location_initial}_{date}_{time}_1pxKptsOverlap"
# parent_folder_name = f"{location_initial}_{date}_{time}_0pxKptsOverlap"
resultsFolder = os.path.join(resultsBaseFolder, parent_folder_name)

# Create the parent directory if it doesn't exist
os.makedirs(resultsFolder, exist_ok=True)

print(f"Results will be saved to: {resultsFolder}")
print()

# All combinations of polarized camera images (I0, I45, I90, I135)
polarization_angles = ["I", "I0", "I45", "I90", "I135"]

# Create all ordered pairs of combinations (including both directions)
combinations = []

# for i in range(len(polarization_angles)):
#     for j in range(len(polarization_angles)):
#         if i != j:  # Don't pair with itself
#             combinations.append((polarization_angles[i], polarization_angles[j]))

# print(f"Total combinations: {len(combinations)}")
# print(f"Combinations: {combinations}")
# print()


# combinations.append(("I", "I45"))
# combinations.append(("I", "I90"))
# combinations.append(("I", "I135"))
# combinations.append(("I155", "I100"))
# combinations.append(("I155", "I90"))
#
#
# Four channels combos
# I45I90 - I0I45, I0I90, I0I135, I45I0, I45I135, I90I0, I90I45, I90I135, I135I0, I135I45, I135I90
# II90 - II0, II45, II135

# combinations.append(("I45","I90","I0","I45"))
# combinations.append(("I45","I90","I0","I90"))
# combinations.append(("I45","I90","I0","I135"))
# combinations.append(("I45","I90","I45","I0"))
# combinations.append(("I45","I90","I45","I135"))
# combinations.append(("I45","I90","I90","I0"))
# combinations.append(("I45","I90","I90","I45"))
# combinations.append(("I45","I90","I90","I135"))
# combinations.append(("I45","I90","I135","I0"))
# combinations.append(("I45","I90","I135","I45"))
# combinations.append(("I45","I90","I135","I90"))
# combinations.append(("I","I90","I","I0"))
# combinations.append(("I","I90","I","I45"))
# combinations.append(("I","I90","I","I135"))

combinations.append(("I155","I90","I90","I45"))
combinations.append(("I155","I90","I0","I90"))
combinations.append(("I155","I90","I0","I135"))
combinations.append(("I155","I90","I45","I0"))
combinations.append(("I155","I90","I45","I135"))
combinations.append(("I155","I90","I90","I0"))
combinations.append(("I155","I90","I90","I45"))
combinations.append(("I155","I90","I90","I135"))
combinations.append(("I155","I90","I135","I0"))
combinations.append(("I155","I90","I135","I45"))
combinations.append(("I155","I90","I135","I90"))


# Track failed runs that couldn't be recovered
permanently_failed_runs = []
max_retries = 5  # Maximum number of retries per iteration

# Run each combination 10 times
for combo_idx, (cam0_angle, cam1_angle, cam2_angle, cam3_angle) in enumerate(combinations):
    imgsCam0 = base_path + f"polcam{cam0_angle}_1224x1024/"
    imgsCam1 = base_path + f"polcam{cam1_angle}_1224x1024/"
    imgsCam2 = base_path + f"polcam{cam2_angle}_1224x1024/"
    imgsCam3 = base_path + f"polcam{cam3_angle}_1224x1024/"

    # Create folder for this combination
    combo_folder_name = f"{cam0_angle}{cam1_angle}{cam2_angle}{cam3_angle}_1224x1024"
    trajectoryFileFolder = os.path.join(resultsFolder, combo_folder_name)

    # Create the directory if it doesn't exist
    os.makedirs(trajectoryFileFolder, exist_ok=True)

    print(
        f"\nCombination {combo_idx + 1}/{len(combinations)}: {cam0_angle} + {cam1_angle} + {cam2_angle} + {cam3_angle}"
    )
    print(f"Output folder: {trajectoryFileFolder}")

    # Run 10 times for this combination
    for iteration in range(10):
        iteration_num = iteration
        retry_count = 0
        success = False

        while not success and retry_count < max_retries:
            if retry_count == 0:
                print(f"  Running iteration {iteration_num}/10", end=" ... ")
            else:
                print(f"    Retry {retry_count}/{max_retries - 1}", end=" ... ")

            keyFramesTrajectoryFile = os.path.join(
                trajectoryFileFolder,
                str(iteration_num).zfill(5) + "_KeyFramesTrajectory.txt",
            )

            framesTrajectoryFile = os.path.join(
                trajectoryFileFolder,
                str(iteration_num).zfill(5) + "_FramesTrajectory.txt",
            )

            framesKeyPointsFile = os.path.join(
                trajectoryFileFolder,
                str(iteration_num).zfill(5) + "_FramesKeypointsNumber.txt",
            )

            program = [
                executable,
                vocabulary,
                settings,
                imgsCam0,  # imgs cam0
                imgsCam1,  # imgs cam1
                imgsCam2,  # imgs cam0
                imgsCam3,  # imgs cam1
                keyFramesTrajectoryFile,  # keyframes trajectory file
                framesTrajectoryFile,  # frames trajectory file
                framesKeyPointsFile,
            ]

            # Run your C++ program and capture both stdout and stderr
            prog_proc = subprocess.run(program, capture_output=True, text=True)

            # Check if process succeeded
            if prog_proc.returncode == 0:
                print("✓ Success")
                success = True
            else:
                # Check stderr for specific error messages
                error_output = prog_proc.stderr + prog_proc.stdout

                if (
                    "X Window System error" in error_output
                    or "BadAccess" in error_output
                    or "MIT-SHM" in error_output
                    or "Gdk-ERROR" in error_output
                ):
                    print(f"❌ X Window System Error")
                    retry_count += 1
                    if retry_count < max_retries:
                        print(
                            f"    Retrying same iteration (attempt {retry_count + 1}/{max_retries})..."
                        )
                    else:
                        print(
                            f"    ⚠️  Max retries ({max_retries}) reached. Moving to next iteration."
                        )
                        permanently_failed_runs.append(
                            {
                                "combination": f"{cam0_angle}{cam1_angle}",
                                "iteration": iteration_num,
                                "error_type": "X Window System Error (unrecoverable)",
                                "trajectory_file": framesTrajectoryFile,
                                "retries_attempted": retry_count,
                            }
                        )
                        success = True  # Exit retry loop but mark as failed
                else:
                    print(f"❌ Process failed (exit code: {prog_proc.returncode})")
                    # Extract first line of error
                    error_msg = (
                        error_output.split("\n")[0] if error_output else "Unknown error"
                    )
                    permanently_failed_runs.append(
                        {
                            "combination": f"{cam0_angle}{cam1_angle}",
                            "iteration": iteration_num,
                            "error_type": f"Exit code {prog_proc.returncode}",
                            "trajectory_file": framesTrajectoryFile,
                            "error_msg": error_msg[:100],
                            "retries_attempted": 0,
                        }
                    )
                    success = (
                        True  # Don't retry for non-X11 errors, move to next iteration
                    )

total_runs = len(combinations) * 10
print(f"\n{'=' * 70}")
print(f"All {total_runs} iterations completed!")
print(f"Results organized in {len(combinations)} folders under:")
print(f"  {resultsFolder}")

if permanently_failed_runs:
    print(f"\n⚠️  {len(permanently_failed_runs)} iteration(s) failed after retries:")
    print(f"{'=' * 70}")
    for failed in permanently_failed_runs:
        print(
            f"  Combination: {failed['combination']}, Iteration: {failed['iteration']}"
        )
        print(f"    Error Type: {failed['error_type']}")
        if failed["retries_attempted"] > 0:
            print(f"    Retries attempted: {failed['retries_attempted']}")
        if "error_msg" in failed:
            print(f"    Message: {failed['error_msg']}")
        print()
else:
    print(f"\n✓ All iterations completed successfully!")
