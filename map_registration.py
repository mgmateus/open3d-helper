import open3d as o3d
import numpy as np
import copy
import cv2

class R3D:
    @staticmethod
    def transformation_matrix(position, orientation):
        x, y, z = position
        qx, qy, qz, qw = orientation
        
        T = np.eye(4)
        T[:3,3] = [-y, z, x]
        # T[:3,3] = [x, y, z]
        R = np.eye(4)
        R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qy, qz, qx, qw))
        # R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qy, qz, qx, qw))
        # C = np.array([
        #     [ 1,  0,  0,  0],
        #     [ 0,  -1, 0,  0],
        #     [ 0,  0,  -1,  0],
        #     [ 0,  0,  0,  1]
        # ])
        
        C = np.array([
            [ 1,  0,  0,  0],
            [ 0,  0, -1,  0],
            [ 0,  1,  0,  0],
            [ 0,  0,  0,  1]
        ])

        return R.T @ T @ C
    
    
    
    def __init__(self, depth_trunc, image_width, image_height, fov):
        
        
        fov_rad = fov * np.pi/180
        fd = (image_width/2.0) / np.tan(fov_rad/2.0)

        self.__intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.__intrinsic.set_intrinsics(image_width, image_height, fd, fd, image_width/2 - 0.5, image_height/2 - 0.5)

        self.__image_width = image_width
        self.__image_height = image_height
        self.__depth_trunc = depth_trunc

        self.i =0
        self.pcd = o3d.geometry.PointCloud()
        self.cams = []
        self.geos = [self.pcd]
    
        self.pcds = []
        self.views = []
        
        self.past_transformation = np.eye(4)
        self.past_rgbd_image = None
        
        # self.volume = o3d.pipelines.integration.UniformTSDFVolume(
        #     length=1000,
        #     resolution=1024,
        #     sdf_trunc=0.01,
        #     color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        # )
        
        # self.vis =  o3d.visualization.VisualizerWithKeyCallback()
        # self.vis.register_key_callback(32, self.key_callback)
        # self.vis.create_window(window_name="Point Cloud Visualizer",
        #                 width=800, height=800)
        # self.vis.add_geometry(self.pcd)
        # render_opt = self.vis.get_render_option()
        # render_opt.point_size = 1.0
        
    @property
    def image_width(self):
        return self.__image_width
    
    @property
    def image_height(self):
        return self.__image_height
    
    

    # def dataset_generate(self, rgb, depth, segmentation, tf):
        
        
    # def create_point_cloud(self, rgb, depth, camera_position, camera_orientation):
    #     rgb_ = o3d.geometry.Image(rgb)
    #     depth_ = o3d.geometry.Image(depth)
    #     rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_, 
    #                                                               depth_, 
    #                                                               depth_scale=1.0, 
    #                                                               depth_trunc=self.__depth_trunc, 
    #                                                               convert_rgb_to_intensity=True)
        
    #     if self.past_rgbd_image is None:
    #         self.past_rgbd_image = rgbd
        
    #     success, M, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
    #         rgbd, self.past_rgbd_image, self.__intrinsic
    #     )
        
    #     self.past_rgbd_image = rgbd
    #     self.transformation = self.transformation @ M
        
    #     pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic)
    #     pcd = pcd.voxel_down_sample(voxel_size=0.02)
        
    #     view = o3d.geometry.LineSet.create_camera_visualization(self.__intrinsic, extrinsic= self.transformation)
        
    #     return pcd, view
            
    # def key_callback(self, vis):
    #     ...
        
    def create_point_cloud(self, rgb, depth, camera_position, camera_orientation):
        rgb_ = o3d.geometry.Image(rgb)
        depth_ = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_, 
                                                                  depth_, 
                                                                  depth_scale=1.0, 
                                                                  depth_trunc=self.__depth_trunc, 
                                                                  convert_rgb_to_intensity=False)
        
        
        F = self.transformation_matrix(camera_position, camera_orientation)      
        # if np.array_equal(self.past_transformation, np.eye(4)):
        #     self.past_transformation = F
        
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic)
        # pcd = pcd.voxel_down_sample(voxel_size=2)
        
        
        # view = o3d.geometry.LineSet.create_camera_visualization(self.__intrinsic, extrinsic= F)
        
        # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic)
        
        return pcd, F
        
    def rgb_point_cloud(self, rgb, depth):
        rgb_ = o3d.geometry.Image(rgb)
        depth_ = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_, 
                                                                  depth_, 
                                                                  depth_scale=1.0, 
                                                                  depth_trunc=self.__depth_trunc, 
                                                                  convert_rgb_to_intensity=False)
            
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic)

        
    # def registration(self, rgb, depth, camera_position, camera_orientation):
    #     pcd, v, F = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
    #     pcd.transform(self.past_transformation)
    #     self.past_transformation = np.linalg.inv(self.past_transformation) @ F
    #     self.__pcd += pcd
    #     # self.past_transformation = F
    #     self.__cams.append(v)
    #     # geos = [self.__pcd]
    #     # geos.extend(self.__cams)
            
    #     return pcd , v
    
    
    def preprocess_point_cloud(self,pcd, voxel_size):
        pcd_down = pcd.voxel_down_sample(voxel_size)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0,
                                                max_nn=30))
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5.0,
                                                max_nn=100),
        )
        return (pcd_down, pcd_fpfh)

    def ransac(self,src_down,
                dst_down,
                src_fpfh,
                dst_fpfh,
                distance_threshold= 0.01, 
                max_iterations= 10000, 
                confidence= 0.999):
        
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                src_down,
                dst_down,
                src_fpfh,
                dst_fpfh,
                mutual_filter=False,
                max_correspondence_distance=distance_threshold,
                estimation_method=o3d.pipelines.registration.
                TransformationEstimationPointToPoint(False),
                ransac_n=3,
                checkers=[
                    o3d.pipelines.registration.
                    CorrespondenceCheckerBasedOnEdgeLength(0.99),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                        distance_threshold),
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
                    max_iterations, confidence),
            )
        
        return result.transformation
    
    def p2p_icp_registration(self,source_cloud, target_cloud, n_points=10000, threshold=0.02, \
        relative_fitness=1e-10, relative_rmse=1e-8, max_iteration=10000, max_correspondence_distance=50, F= np.eye(4)):
        
        """ align the source cloud to the target cloud using point-to-point ICP registration algorithm
        Args: 
            source_cloud (open3d.geometry.PointCloud): source open3d point cloud
            target_cloud (open3d.geometry.PointCloud): target open3d point cloud
            for other parameter, go to http://www.open3d.org/docs/0.9.0/python_api/open3d.registration.registration_icp.html
        Returns:
            icp_result (open3d.registration.RegistrationResult): registration result
        """
        
        
        source_cloud = copy.deepcopy(source_cloud)
        target_cloud = copy.deepcopy(target_cloud)
        n_source_points = np.shape(source_cloud.points)[0]
        n_target_points = np.shape(target_cloud.points)[0]
        n_sample = np.min([n_source_points, n_target_points, n_points])
        source_idxes = np.random.choice(n_source_points, n_sample, replace=False)
        target_idxes = np.random.choice(n_target_points, n_sample, replace=False)
        source_cloud = source_cloud.select_by_index(source_idxes)
        target_cloud = target_cloud.select_by_index(target_idxes)
        trans_init = F
        evaluation = o3d.pipelines.registration.evaluate_registration(source_cloud, target_cloud, threshold, trans_init)
        icp_result = o3d.pipelines.registration.registration_icp(
            source = source_cloud, target = target_cloud, max_correspondence_distance=max_correspondence_distance, # unit in millimeter
            init = F,  
            estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                                                relative_fitness=relative_fitness,
                                                relative_rmse=relative_rmse,
                                                max_iteration=max_iteration))                                               
        return icp_result, evaluation
    
    def ppf_icp_registration(self, source_cloud, target_cloud, n_points=3000, n_iter=100, tolerance=0.001, num_levels=5, scale=0.001):
        """ align the source cloud to the target cloud using point pair feature (PPF) match
        Args: 
            source_cloud (open3d.geometry.PointCloud): source open3d point cloud
            target_cloud (open3d.geometry.PointCloud): target open3d point cloud
            for other parameter, go to https://docs.opencv.org/master/dc/d9b/classcv_1_1ppf__match__3d_1_1ICP.html
        Returns:
            pose (np.array): 4x4 transformation between source and targe cloud
            residual (float): the output resistration error
        """
        source_cloud = copy.deepcopy(source_cloud)
        source_cloud = source_cloud.voxel_down_sample(scale)
        target_cloud = copy.deepcopy(target_cloud)
        n_source_points = np.shape(source_cloud.points)[0]
        n_target_points = np.shape(target_cloud.points)[0]
        n_sample = np.min([n_source_points, n_target_points, n_points])
        if n_sample == 0:
            return None, 10000
        if n_source_points > n_points:
            source_idxes = np.random.choice(n_source_points, n_sample, replace=False)
            source_cloud = source_cloud.select_by_index(source_idxes)
        if n_target_points > n_points:
            target_idxes = np.random.choice(n_target_points, n_sample, replace=False)
            target_cloud = target_cloud.select_by_index(target_idxes)
        target_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        source_np_cloud = np.concatenate([np.asarray(source_cloud.points), np.asarray(source_cloud.normals)], axis=1).astype(np.float32)
        target_np_cloud = np.concatenate([np.asarray(target_cloud.points), np.asarray(target_cloud.normals)], axis=1).astype(np.float32)
        icp_fnc = cv2.ppf_match_3d_ICP(n_iter, tolerence=tolerance, rejectionScale=2.5, numLevels=num_levels) 
        try:
            retval, residual, pose = icp_fnc.registerModelToScene(source_np_cloud, target_np_cloud)
        except:
            return None, 10000
        else:
            return pose, residual
        
    def registration(self, rgb, depth, camera_position, camera_orientation):
        pcd, v, F = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
        
        
        # distance_threshold = 0.001  # distância máxima de correspondência (em metros)
        
        # reg_result = o3d.pipelines.registration.registration_icp(
        #     self.__pcd, pcd, distance_threshold,
        #     np.eye(4),  # estimativa inicial da matriz de transformação
        #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50000))
        
        if not self.__pcd.is_empty():
            # if np.array_equal(self.past_transformation, np.eye(4)):
        #     src_down, src_fpfh = self.preprocess_point_cloud(self.__pcd, 0.5)
        #     dst_down, dst_fpfh = self.preprocess_point_cloud(pcd, 0.5)

        #     t = self.ransac(src_down, dst_down, src_fpfh, dst_fpfh)
            
            
            
        #     self.__pcd.transform(t)
            
            reg_result, _ = self.ppf_icp_registration(self.__pcd, pcd)
                # self.past_transformation = reg_result.transformation
            # self.__pcd.transform(reg_result.transformation)
            pcd.transform(np.linalg.inv(reg_result))
            
            
            
        
        self.pcd+=pcd
        self.cams.append(v)
        
        self.geos = [self.pcd]
        self.geos.extend(self.cams)
        return pcd, F
    
    # def registration(self, rgb, depth, camera_position, camera_orientation):
    #     pcd, v = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
        
    #     self.__pcd += pcd
    #     self.__cams.append(v)
    #     geos = [self.__pcd]
    #     geos.extend(self.__cams)
            
    #     return pcd , v
        
        
        
    # def registration(self, rgb, depth, camera_position, camera_orientation):
    #     pcd, v = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
        
    #     self.__pcd += pcd
    #     self.__cams.append(v)
    #     geos = [self.__pcd]
    #     geos.extend(self.__cams)
      
    #     self.vis.update_geometry(self.__pcd)
    #     self.vis.poll_events()
    #     self.vis.update_renderer()
        
    #     o3d.visualization.draw_geometries([self.__pcd] + self.__cams)
    #     return pcd


# depth_trunc = 100
# width = 672
# height = 376
# fov = 90

# reconstruction = R3D(depth_trunc, width, height, fov)


def transformation_matrix(position, orientation):
        x, y, z = position
        qx, qy, qz, qw = orientation
        
        T = np.eye(4)
        T[:3,3] = [y, z, x]
        # T[:3,3] = [x, y, z]
        R = np.eye(4)
        R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qy, qz, qx, qw))
        # R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qy, qz, qx, qw))
        # C = np.array([
        #     [ 1,  0,  0,  0],
        #     [ 0,  -1, 0,  0],
        #     [ 0,  0,  -1,  0],
        #     [ 0,  0,  0,  1]
        # ])
        
        C = np.array([
            [ 1,  0,  0,  0],
            [ 0,  0, -1,  0],
            [ 0,  1,  0,  0],
            [ 0,  0,  0,  1]
        ])

        return R.T @ T @ C
    
def transformation_matrix2(position, orientation):
        x, y, z = position
        rx, ry, rz = orientation
        
        T = np.eye(4)
        T[:3,3] = [x, y, z]
        R = np.eye(4)
        R[:3,:3] = o3d.geometry.get_rotation_matrix_from_xyz((rx, ry, rz))
        

        return R.T @ T 
    
if __name__ == "__main__":
    ABS_PATH = '/home/airsim/AirSim/ros/src/hybrid_nbv_planning/files/meshes/'
    mesh_names = [ABS_PATH+'plataform.PLY']
    mesh = o3d.io.read_triangle_mesh(mesh_names[0])
    mesh.compute_vertex_normals()
    
    
    depth_trunc = 1000
    image_width = 672
    image_height = 376
    fov = 90
    fov_rad = fov * np.pi/180
    fd = (image_width/2.0) / np.tan(fov_rad/2.0)
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(image_width, image_height, fd, fd, image_width/2 - 0.5, image_height/2 - 0.5)
    intr = np.array(intrinsic.intrinsic_matrix)
    n = np.hstack((intr.T[0], intr.T[1]))
    nn = np.hstack((n, intr.T[2]))
    print(nn)
    
    R = mesh.get_rotation_matrix_from_xyz((0, np.pi/2, 0))
    mesh.rotate(R, center=(0,0,0))
    t_structure = [114.0999984741211, 12.59999942779541, 45.3045]
    mesh.translate(t_structure)
    pcd = mesh.sample_points_poisson_disk(5000)
    
    positions = [[0, 0, 23.65148162841797], [0, 0, 23.65148162841797]]
    views = []
    for pos in positions:
        transform = transformation_matrix2(pos, [0,0,0])
        view = o3d.geometry.LineSet.create_camera_visualization(intrinsic, transform)
        views.append(view)
        
    
    geos = [pcd]
    geos.extend(views)
    
    # o3d.visualization.draw(views)
    
    # pcd = mesh.sample_points_poisson_disk(5000)
    
    # diameter = np.linalg.norm(
    #     np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    
    # print("Define parameters used for hidden_point_removal")
    
    # camera = [1000, 1000, diameter]
    # radius = diameter * 100

    # print("Get all points that are visible from given view point")
    # _, pt_map = pcd.hidden_point_removal(camera, radius)

    # print("Visualize result")
    # pcd_ = pcd.select_by_index(pt_map)
    # pcd_.transform(transform)
    # o3d.visualization.draw([pcd_])
