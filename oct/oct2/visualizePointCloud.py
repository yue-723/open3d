import open3d as o3d
# input_file = "./test_data/retina_oct_5.ply"
# input_file = "./test_data/lod_10000.ply"
# input_file = "./test_data/oct_5/bpa_mesh.ply"
input_file = "./test_data/oct_60/lod_100000.ply"
# input_file = "./test_data/Bunny.ply"
pcd = o3d.io.read_point_cloud(input_file) # Read the point cloud
o3d.visualization.draw_geometries([pcd]) # Visualize the point cloud 


radii = [0.5, 1, 2, 4]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))
o3d.visualization.draw_geometries([pcd, rec_mesh])