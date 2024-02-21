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
    
    def __ini__(self):
        self.__pcd = o3d.geometry.PointCloud()
        self.__cams = []
        
    
    
    # def create_point_cloud(self, )



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