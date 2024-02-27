import open3d as o3d
import numpy as np

class R3D:
    @staticmethod
    def transformation_matrix(position, orientation):
        x, y, z = position
        qw, qx, qy, qz = orientation
        
        T = np.eye(4)
        T[:3,3] = [-y, -z, -x]
        
        R = np.eye(4)
        R[:3,:3] = o3d.geometry.get_rotation_matrix_from_quaternion((qw, qy, qz, qx))
        
        C = np.array([
            [ 1,  0,  0,  0],
            [ 0,  0, -1,  0],
            [ 0,  1,  0,  0],
            [ 0,  0,  0,  1]
        ])

        return R.T @ T @ C
    
    def __init__(self, depth_trunc, image_width, image_height, fov):
        self.__depth_trunc = depth_trunc

        self.__pcd = o3d.geometry.PointCloud()
        self.__cams = []
        self.__model = o3d.t.pipelines.slam.Model(voxel_size= 1, device=o3d.core.Device("CUDA:0"))
        
        fov_rad = fov * np.pi/180
        fd = (image_width/2.0) / np.tan(fov_rad/2.0)

        self.__intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.__intrinsic.set_intrinsics(image_width, image_height, fd, fd, image_width/2 - 0.5, image_height/2 - 0.5)

        self.__image_width = image_width
        self.__image_height = image_height

        self.i =0

        # self.vis = o3d.visualization.Visualizer()
        # axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # self.vis.create_window()
        # self.vis.add_geometry(self.__pcd)

    @property
    def image_width(self):
        return self.__image_width
    
    @property
    def image_height(self):
        return self.__image_height
    
    
    def create_point_cloud(self, rgb, depth, camera_position, camera_orientation):
        rgb_ = o3d.geometry.Image(rgb)
        depth_ = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_, 
                                                                  depth_, 
                                                                  depth_scale=1.0, 
                                                                  depth_trunc=self.__depth_trunc, 
                                                                  convert_rgb_to_intensity=False)
        
        F = self.transformation_matrix(camera_position, camera_orientation)
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic), \
            o3d.geometry.LineSet.create_camera_visualization(self.__intrinsic, F)
    
        # return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic, extrinsic=F), \
        #     o3d.geometry.LineSet.create_camera_visualization(self.__intrinsic, F)
    
    def create_point_cloud2(self, rgb, depth):
        # T_frame_to_model = o3d.core.Tensor(np.identity(4))
        rgb = o3d.geometry.Image(rgb)
        depth = o3d.geometry.Image(depth)
        input_frame = o3d.t.pipelines.slam.Frame(depth.rows, depth.columns,
                                             self.__intrinsic, device=o3d.core.Device("CUDA:1"))
        raycast_frame = o3d.t.pipelines.slam.Frame(depth.rows, depth.columns,
                                             self.__intrinsic, device=o3d.core.Device("CUDA:1"))
        
        input_frame.set_data_from_image('depth', depth)
        input_frame.set_data_from_image('color', rgb)

        result = self.__model.track_frame_to_model(input_frame, raycast_frame)
        T_frame_to_model = T_frame_to_model @ result.transformation

        self.__cams.append(T_frame_to_model.cpu().numpy())
        self.__model.update_frame_pose(self.i, T_frame_to_model)
        self.__model.integrate(input_frame)
        self.__model.synthesize_model_frame(raycast_frame)
        self.i+=1
    
    def registration(self, rgb, depth, camera_position, camera_orientation):
        pcd, view = self.create_point_cloud(rgb, depth, camera_position, camera_orientation)
        # self.__pcd += pcd
        # self.__cams.append(view)
        # geos = [self.__pcd]
        # geos.extend(self.__pcd)
        # self.vis.update_geometry(self.__pcd)
        # self.vis.poll_events()
        # self.vis.update_renderer()
        
        return pcd, view


# depth_trunc = 100
# width = 672
# height = 376
# fov = 90

# reconstruction = R3D(depth_trunc, width, height, fov)




    




# ABS_PATH = '/home/airsim/AirSim/ros/src/hybrid_nbv_planning/files/meshes/'
# mesh_names = [ABS_PATH+'plataform.PLY']


# mesh = o3d.io.read_triangle_mesh(mesh_names[0])
# mesh.compute_vertex_normals()
# pcd = mesh.sample_points_poisson_disk(5000)
# # o3d.visualization.draw_geometries([pcd])
# diameter = np.linalg.norm(
#     np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
# # o3d.visualization.draw_geometries([pcd])
# print("Define parameters used for hidden_point_removal")
# camera = [1000, 1000, diameter]
# radius = diameter * 100

# print("Get all points that are visible from given view point")
# _, pt_map = pcd.hidden_point_removal(camera, radius)

# print("Visualize result")
# pcd_ = pcd.select_by_index(pt_map)
# o3d.visualization.draw_geometries([pcd_])