import numpy as np
import open3d as o3d
import pandas as pd

# input_file = "./test_data/oct_5/retina_oct_5.xyz"
# output_file = "./test_data/oct_5/retina_oct_5.ply"
input_file = "./test_data/oct_60/retina_oct_60.xyz"
output_file = "./test_data/oct_60/retina_oct_60.ply"

data = pd.read_csv(input_file, sep=" ", header=None)
print(data)
data.columns = ["X", "Y", "Z"]
X = data["X"].to_numpy()
Y = data["Y"].to_numpy()
Z = data["Z"].to_numpy()
xyz = np.asarray([X, Y, Z])
xyz_t = np.transpose(xyz)
print(xyz)
print(xyz_t)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_t)
# o3d.io.write_point_cloud(output_file, pcd)
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='SSI_OCT', width=1080,
                  height=720, visible=True)
vis.add_geometry(pcd)
view_ctl = vis.get_view_control()
# view_ctl.set_up((1, -1, 0))
view_ctl.set_front((1, 1, 1))
# set the positive direction of the x-axis as the up direction
# view_ctl.set_up((1, 0, 0))
# set the negative direction of the y-axis as the up direction
# view_ctl.set_up((0, -1, 0))
# view_ctl.set_front((1, 1, 1))
# set the positive direction of the x-axis toward you
# view_ctl.set_front((1, 0, 0))
# # set the original point as the center point of the window
# view_ctl.set_lookat((0, 0, 0))
vis.run()
