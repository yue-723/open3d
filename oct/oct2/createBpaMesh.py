import open3d as o3d
import numpy as np
import random
import os

#function creation
def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods={}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        o3d.io.write_triangle_mesh(path+"lod_"+str(i)+extension, mesh_lod)
        mesh_lods[i]=mesh_lod
    print("generation of "+str(i)+" LoD successful")
    return mesh_lods

input_file = "./test_data/oct_5/retina_oct_5.ply"
output_path = "./test_data/oct_5/"
# input_file = "./test_data/oct_60/retina_oct_60.ply"
# output_path = "./test_data/oct_tmp/"

if not os.path.isdir(output_path):
    os.makedirs(output_path)

print("Testing IO for meshes ...")
# mesh = o3d.io.read_triangle_mesh("./test_data/retina_oct_5.ply")
pcd = o3d.io.read_point_cloud(input_file) # Read the point cloud

# add normals information in pcd
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# visualize with normal
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
# show first 10 normals
objectNormals = pcd.normals
print(np.asarray(objectNormals)[:10])

# visualize without normal
o3d.visualization.draw_geometries([pcd]) # Visualize the point cloud 

distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))

dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

o3d.io.write_triangle_mesh(output_path+"bpa_mesh.ply", dec_mesh)

#execution of function
my_lods = lod_mesh_export(bpa_mesh, [100000,50000,10000,1000,100], ".ply", output_path)

o3d.visualization.draw_geometries([my_lods[100]])
