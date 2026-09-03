#!/usr/bin/env python3
"""Draw feature tracks over the last processed frame.
Usage: draw_tracks.py <image> <tracks.txt>"""
import sys
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

img = plt.imread(sys.argv[1])
tracks = defaultdict(list)
for line in open(sys.argv[2]):
    k, fid, x, y = line.split()
    tracks[int(fid)].append((int(k), float(x), float(y)))

plt.figure(figsize=(12, 8))
plt.imshow(img, cmap="gray")
for fid, pts in tracks.items():
    pts.sort()
    xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
    plt.plot(xs, ys, lw=0.8)
    plt.plot(xs[-1], ys[-1], "r.", ms=3)
plt.title(f"{len(tracks)} tracks over {max(p[0] for ps in tracks.values() for p in ps)+1} frames")
plt.axis("off"); plt.tight_layout(); plt.show()

