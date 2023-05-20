import open3d as o3d
# input_file = "./test_data/oct_5/lod_10000.ply"
# input_file = "./test_data/oct_5/lod_50000.ply"
# input_file = "./test_data/oct_5/lod_100000.ply"
# input_file = "./test_data/oct_5/bpa_mesh.ply"
# input_file = "./test_data/oct_60/lod_100000.ply"
# input_file = "./test_data/oct_60/bpa_mesh.ply"
input_file = "./test_data/Bunny.ply"
print("Testing IO for meshes ...")
mesh = o3d.io.read_triangle_mesh(input_file)
mesh.paint_uniform_color([1, 0.706, 0])
o3d.visualization.draw_geometries([mesh]) # Visualize the mesh
