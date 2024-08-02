import numpy as np
import open3d as o3d


points = (np.random.rand(1000, 3) - 0.5) / 4
colors = np.random.rand(1000, 3)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries( [pcd] )