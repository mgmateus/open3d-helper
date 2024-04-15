import open3d as o3d
import numpy as np

class R3D:
    def __init__(self, depth_trunc, image_width, image_height, fov):
        
        
        fov_rad = fov * np.pi/180
        fd = (image_width/2.0) / np.tan(fov_rad/2.0)

        self.__intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.__intrinsic.set_intrinsics(image_width, image_height, fd, fd, image_width/2 - 0.5, image_height/2 - 0.5)

        self.__image_width = image_width
        self.__image_height = image_height
        self.__depth_trunc = depth_trunc

    @property
    def image_width(self):
        return self.__image_width
    
    @property
    def image_height(self):
        return self.__image_height
        
    def generate_rgbd_point_cloud(self, rgb, depth):
        rgb_ = o3d.geometry.Image(rgb)
        depth_ = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_, 
                                                                  depth_, 
                                                                  depth_scale=1.0, 
                                                                  depth_trunc=self.__depth_trunc, 
                                                                  convert_rgb_to_intensity=False)
            
        return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.__intrinsic)
    