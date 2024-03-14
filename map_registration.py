import open3d as o3d
import numpy as np

class R3D:
    @staticmethod
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
    
    
    def __init__(self, depth_trunc, image_width, image_height, fov):
        
        
        fov_rad = fov * np.pi/180
        fd = (image_width/2.0) / np.tan(fov_rad/2.0)

        self.__intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.__intrinsic.set_intrinsics(image_width, image_height, fd, fd, image_width/2 - 0.5, image_height/2 - 0.5)

        self.__image_width = image_width
        self.__image_height = image_height
        self.__depth_trunc = depth_trunc

        self.i =0
        self.__pcd = o3d.geometry.PointCloud()
        self.__cams = []
        self.geos = [self.__pcd]
        
        self.pcds = []
        self.views = []
        
        self.past_transformation = np.eye(4)
        self.past_rgbd_image = None
        
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
    
    @property
    def pcd(self):
        return self.__pcd
    
    @property
    def cams(self):
        return self.__cams

    
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
            
    def key_callback(self, vis):
        ...
        
    def create_point_cloud(self, rgb, depth, camera_position, camera_orientation):
        rgb_ = o3d.geometry.Image(rgb)
        depth_ = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_, 
                                                                  depth_, 
                                                                  depth_scale=1.0, 
                                                                  depth_trunc=self.__depth_trunc, 
                                                                  convert_rgb_to_intensity=False)
        
        
        F = self.transformation_matrix(camera_position, camera_orientation)
        if np.array_equal(self.past_transformation, np.eye(4)):
            self.past_transformation = F
       
        
        # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic, extrinsic= self.past_transformation)
        # pcd = pcd.voxel_down_sample(voxel_size=0.05)
        
        # view = o3d.geometry.LineSet.create_camera_visualization(self.__intrinsic, extrinsic= self.past_transformation)
        
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic)
        
        return pcd, None, F
        
            
    
        
    def registration(self, rgb, depth, camera_position, camera_orientation):
        pcd, v, F = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
        pcd.transform(self.past_transformation)
        self.past_transformation = np.linalg.inv(self.past_transformation) @ F
        self.__pcd += pcd
        # self.past_transformation = F
        self.__cams.append(v)
        # geos = [self.__pcd]
        # geos.extend(self.__cams)
            
        return pcd , v
        
    # def registration(self, rgb, depth, camera_position, camera_orientation):
    #     pcd, v = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
        
    #     if self.pcd.points == 0:
    #         self.__pcd = pcd
    #     self.__pcd+=pcd
    #     self.__cams.append(v)
        
    #     self.geos = [self.__pcd]
    #     self.geos.extend(self.__cams)
    #     return pcd, self.past_transformation
    
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
        print(view)
        
    
    geos = [pcd]
    geos.extend(views)
    
    o3d.visualization.draw(views)
    
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
