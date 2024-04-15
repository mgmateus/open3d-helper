import open3d as o3d
import numpy as np

import os
import json
import cv2

from numpy.typing import NDArray

class R3D:
    def __init__(self, dataset_path : str, 
                 depth_trunc : float, 
                 image_width : int, 
                 image_height : int, 
                 fov : int):
        
        self.__dataset_path = dataset_path or os.path.dirname(__file__)
        
        fov_rad = fov * np.pi/180
        fd = (image_width/2.0) / np.tan(fov_rad/2.0)

        self.__intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.__intrinsic.set_intrinsics(image_width, image_height, fd, fd, image_width/2 - 0.5, image_height/2 - 0.5)
        
        self.__intrinsic_matrix = self.__intrinsic.intrinsic_matrix
        self.__npintrinsic_matrix = np.array(self.__intrinsic_matrix)

        transform = self.__npintrinsic_matrix.T
        self.__data_intrinsic_matrix = np.hstack((np.hstack((transform[0], transform[1])), transform[2]))
        self.__data_intrinsic = {
            "width" : image_width,
            "height" : image_height,
            "intrinsic_matrix" : self.__data_intrinsic_matrix
        }
        print(self.__data_intrinsic)
        
    #     self.__count_name = 0
        
    # def _autoincrement_save_image(self, img : NDArray, complement_path : str = ""):
    #     path = self.__dataset_path + "/" + complement_path + "/" + str(self.__count_name) + ".png"
    #     self.__count_name += 1
    #     cv2.imwrite(path, img)
        
         
    # def generate_intrinsic_json(self, complement_path : str = ""):
    #     path = self.__dataset_path + '/' + complement_path + '/intrinsics.json'
    #     with open(path, "w") as write_file:
    #         json.dump(self.__data_intrinsic, write_file)
            
    
            
    # def store(self, rgb : NDArray = None, depth : NDArray = None, segmentation : NDArray = None, complement_path : str = ""):
    #     current_path = self.__dataset_path + "/" + complement_path
    #     rgb_path = current_path + "/rgb"
    #     depth_path = current_path + "/depth"
    #     segmentation_path = current_path + "/segmentation"
        
    #     if not os.path.exists(current_path):
    #         os.makedirs(current_path)
            
    #     if not os.path.exists(rgb_path):
    #         os.makedirs(rgb_path)
            
    #     if not os.path.exists(depth_path):
    #         os.makedirs(depth_path)
            
    #     if not os.path.exists(segmentation_path):
    #         os.makedirs(segmentation_path)
            
        
            
if __name__ == "__main__":
    dt = 'teste'
    depth_trunc = 1000
    width = 672
    height = 376
    fov = 90
        
    R3D(dt, depth_trunc, width, height, fov)
        
        
        
        
