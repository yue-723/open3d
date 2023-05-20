import open3d as o3d
import numpy as np
import copy
import time


def apply_noise(pcd, mu, sigma):
    noisy_pcd = copy.deepcopy(pcd)
    points = np.asarray(noisy_pcd.points)
    print(len(points))
    points += np.random.normal(mu, sigma, size=points.shape)
    print(len(points))
    noisy_pcd.points = o3d.utility.Vector3dVector(points)
    return noisy_pcd


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw([source_temp, target_temp])


def point_to_point_icp(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    start_time = time.time()
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print("Time cost: ", time.time()-start_time, "s")
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation, "\n")
    draw_registration_result(source, target, reg_p2p.transformation)


def point_to_plane_icp(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")
    start_time = time.time()
    reg_p2l = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    print("Time cost: ", time.time()-start_time, "s")
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation, "\n")
    draw_registration_result(source, target, reg_p2l.transformation)


if __name__ == "__main__":

    source = o3d.io.read_point_cloud("./test_data/cloud_bin_0.pcd")
    target = o3d.io.read_point_cloud("./test_data/cloud_bin_1.pcd")
    source.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([source],
                                      zoom=0.4459,
                                      front=[0.353, -0.469, -0.809],
                                      lookat=[2.343, 2.217, 1.809],
                                      up=[-0.097, -0.879, 0.467])
    print("Total points of source: ", len(source.points))
    print("Total points of target: ", len(target.points))
    mu, sigma = 0, 0.1  # mean and standard deviation
    source_noisy = apply_noise(source, mu, sigma)

    print("Source PointCloud + noise:")
    o3d.visualization.draw_geometries([source_noisy],
                                      zoom=0.4459,
                                      front=[0.353, -0.469, -0.809],
                                      lookat=[2.343, 2.217, 1.809],
                                      up=[-0.097, -0.879, 0.467])
    threshold = 1.0
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                             [-0.139, 0.967, -0.215, 0.7],
                             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    # draw_registration_result(source, target, trans_init)

    # print("Initial alignment")
    # evaluation = o3d.pipelines.registration.evaluate_registration(
    #     source, target, threshold, trans_init)
    # print(evaluation, "\n")
    start_time = time.time()
    # point_to_point_icp(source, target, threshold, trans_init)
    # point_to_plane_icp(source, target, threshold, trans_init)

    print("Robust point-to-plane ICP, threshold={}:".format(threshold))
    loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    print("Using robust loss:", loss)
    p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
                                                          threshold, trans_init,
                                                          p2l)
    print("Time cost: ", time.time()-start_time, "s")
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(source, target, reg_p2l.transformation)

# import open3d as o3d
# import numpy as np
# import time

# if __name__ == "__main__":
#     o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
#     source_raw = o3d.io.read_point_cloud("./test_data/cloud_bin_0.pcd")
#     target_raw = o3d.io.read_point_cloud("./test_data/cloud_bin_1.pcd")

#     source = source_raw.voxel_down_sample(voxel_size=0.02)
#     target = target_raw.voxel_down_sample(voxel_size=0.02)
#     trans = [[0.862, 0.011, -0.507, 0.0], [-0.139, 0.967, -0.215, 0.7],
#              [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]]
#     source.transform(trans)

#     flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
#     source.transform(flip_transform)
#     target.transform(flip_transform)

#     vis = o3d.visualization.Visualizer()
#     vis.create_window()
#     vis.add_geometry(source)
#     vis.add_geometry(target)
#     threshold = 0.05
#     icp_iteration = 100
#     # save_image = False

#     for i in range(icp_iteration):
#         reg_p2l = o3d.pipelines.registration.registration_icp(
#             source, target, threshold, np.identity(4),
#             o3d.pipelines.registration.TransformationEstimationPointToPlane(),
#             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1))
#         source.transform(reg_p2l.transformation)
#         vis.update_geometry(source)
#         vis.poll_events()
#         vis.update_renderer()
#         if i % 15 == 0:
#             vis.capture_screen_image("temp_%04d.jpg" % i)
#     vis.destroy_window()
#     o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)
