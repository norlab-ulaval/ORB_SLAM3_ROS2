import open3d as o3d
import numpy as np

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

# 2️⃣ Create an Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Optional: color all points (here white)
pcd.paint_uniform_color([1.0, 1.0, 1.0])

# 3️⃣ Visualize
o3d.visualization.draw_geometries([pcd])

# 4️⃣ Optional: save to PLY for future use
o3d.io.write_point_cloud("map.ply", pcd)
print("Saved map.ply")

