#!/home/claydergc/miniforge3/bin/python3

import glob
import os
import matplotlib.pyplot as plt

# Folder containing the txt files
folder_path = "."

# Get all txt files and sort them
files = sorted(glob.glob(os.path.join(folder_path, "*.txt")))

plt.figure()

for f in files:
    timestamps = []
    values = []

    with open(f, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            t, n = parts
            timestamps.append(float(t))
            values.append(float(n))

    if not timestamps:
        continue

    # Normalize timestamps to avoid large-number precision issues
    t0 = timestamps[0]
    timestamps = [t - t0 for t in timestamps]

    label = os.path.basename(f)
    plt.plot(timestamps, values, label=label)

plt.xlabel("Timestamp (normalized)")
plt.ylabel("n")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

