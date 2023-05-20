import numpy as np
import open3d as o3d
import pandas as pd
import sys

Input_FILENAME = sys.argv[1]
test_PATH = "./test_data/" + Input_FILENAME
data = pd.read_csv(test_PATH, sep=" ", header=None, engine='python')
df = pd.DataFrame(data)

df.columns = ["X", "Y", "Z", "R", "G", "B"]

X = df["X"].to_numpy()
Y = df["Y"].to_numpy()
Z = df["Z"].to_numpy()
R = df["R"].to_numpy()
G = df["G"].to_numpy()
B = df["B"].to_numpy()

rgb = np.asarray([R, G, B])
rgb_t = np.transpose(rgb)
xyz = np.asarray([X, Y, Z])
xyz_t = np.transpose(xyz)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_t)
pcd.colors = o3d.utility.Vector3dVector(rgb_t)
print(np.asarray(pcd.colors))
result_PATH = test_PATH.split("/")[-1].split(".")[0] + ".ply"
print(result_PATH)
o3d.io.write_point_cloud(result_PATH, pcd)
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='SSI_OCT', width=1080,
                  height=720, visible=True)
pcd_load = o3d.io.read_point_cloud(
    result_PATH, format='ply')  # Read the point cloud
vis.add_geometry(pcd_load)  # Visualize the point cloud
vis.run()
vis.destroy_window()
