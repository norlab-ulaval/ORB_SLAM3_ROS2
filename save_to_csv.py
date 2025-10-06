import numpy as np
import csv

# 1️⃣ Load MapPoints from the TXT file
points = []
with open("map.txt", "r") as f:
    for line in f:
        line = line.strip()
        if line.startswith("MP"):  # Only parse MapPoints
            _, mp_id, x, y, z = line.split()
            points.append([float(x), float(y), float(z)])

points = np.array(points)
print(f"Loaded {points.shape[0]} MapPoints.")

# 2️⃣ Save points to CSV (no header)
with open("map_points.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(points)

print("Saved map_points.csv")

