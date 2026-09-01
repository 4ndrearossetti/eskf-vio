#!/usr/bin/env python3
"""Overlay detected corners on a frame. Usage: draw_corners.py <image> <corners.txt>"""
import sys
import numpy as np
import matplotlib.pyplot as plt

img = plt.imread(sys.argv[1])
pts = np.loadtxt(sys.argv[2])

plt.figure(figsize=(12, 8))
plt.imshow(img, cmap="gray")
plt.scatter(pts[:, 0], pts[:, 1], s=4, c="r", marker=".")
plt.title(f"{len(pts)} corners")
plt.axis("off")
plt.tight_layout()
plt.show()

