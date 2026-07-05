#!/usr/bin/env python3
"""Plot EuRoC ground-truth trajectory in 3D. Usage: python plot_gt.py [path_to_data.csv]"""
import sys
import numpy as np
import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else \
    "MH_01_easy/mav0/state_groundtruth_estimate0/data.csv"

# cols: 0=timestamp[ns], 1:4=pos xyz [m], 4:8=quat wxyz, 8:11=vel, 11:14=gyro bias, 14:17=accel bias
data = np.loadtxt(path, delimiter=",")          # '#' header line skipped automatically
t = (data[:, 0] - data[0, 0]) * 1e-9            # ns -> s, zeroed to start
pos = data[:, 1:4]                              # world-frame position

print(f"{len(data)} samples, {t[-1]:.1f} s, "
      f"{len(data)/t[-1]:.0f} Hz")
print(f"x range {np.ptp(pos[:,0]):.2f} m, "
      f"y {np.ptp(pos[:,1]):.2f} m, z {np.ptp(pos[:,2]):.2f} m")

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(projection="3d")
ax.plot(pos[:, 0], pos[:, 1], pos[:, 2], lw=0.8, color="tab:blue")
ax.scatter(*pos[0],  color="green", s=40, label="start")
ax.scatter(*pos[-1], color="red",   s=40, label="end")

ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
ax.set_box_aspect(np.ptp(pos, axis=0))
ax.set_title("EuRoC MH_01_easy — ground-truth trajectory")
ax.legend()
plt.tight_layout()
plt.show()

