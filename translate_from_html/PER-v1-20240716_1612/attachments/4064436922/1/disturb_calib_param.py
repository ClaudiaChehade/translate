import os
import cv2
import numpy as np
import argparse


class DisturbCalibParam():
    def __init__(self, config):
        self.folder_path = config.folder_path
        self.sensor_name = config.sensor_name
        self.delta_yaw = config.delta_yaw
        self.delta_pitch = config.delta_pitch
        self.delta_roll = config.delta_roll
        self.yaml_name = config.folder_path + "/" + config.sensor_name + ".yaml"
        self.yaml_backup_name = config.folder_path + "/" + config.sensor_name + "_backup.yaml"
        assert os.path.exists(self.folder_path), "Error of xxtrinsic path: {}".format(config.folder_path)
        assert os.path.exists(self.yaml_name), "yaml is not exist: {}".format(self.yaml_name)


    def GenerateNewParamFIle(self):
        '''
            Genetate new xxtrinsic yaml file after disturbed
        '''
        print("Start...", self.yaml_name)
        if not os.path.exists(self.yaml_backup_name):
            os.rename(self.yaml_name, self.yaml_backup_name)
        
        self.ReadYamlFile()
        print("before disturb: \n", self.transform)
        euler = self.Transformation2EulerAngle(self.transform)
        euler[0] += self.delta_roll
        euler[1] += self.delta_pitch
        euler[2] += self.delta_yaw
        R = self.EulerAngle2Rotation(euler)
        self.transform[:3, :3] = R
        print("after disturb: \n", self.transform)

        self.WriteYamlFile()

    
    def ReadYamlFile(self):
        fs_param = cv2.FileStorage(self.yaml_backup_name, cv2.FileStorage_READ)

        self.model_type = fs_param.getNode('model_type').string()
        self.camera_name = fs_param.getNode('camera_name').string()
        self.image_width = fs_param.getNode('image_width').real()
        self.image_height = fs_param.getNode('image_height').real()
        distortion_params = fs_param.getNode('distortion_parameters')
        self.k1 = distortion_params.getNode("k1").real()
        self.k2 = distortion_params.getNode("k2").real()
        self.k3 = distortion_params.getNode("k3").real()
        self.k4 = distortion_params.getNode("k4").real()
        self.k5 = distortion_params.getNode("k5").real()
        self.k6 = distortion_params.getNode("k6").real()
        self.p1 = distortion_params.getNode("p1").real()
        self.p2 = distortion_params.getNode("p2").real()
        projection_params = fs_param.getNode('projection_parameters')
        self.fx = projection_params.getNode("fx").real()
        self.fy = projection_params.getNode("fy").real()
        self.cx = projection_params.getNode("cx").real()
        self.cy = projection_params.getNode("cy").real()
        self.transform = fs_param.getNode('T_v_c').mat()
        fs_param.release()


    def WriteYamlFile(self):
        fs_param_write = cv2.FileStorage(self.yaml_name, cv2.FileStorage_WRITE)
        distortion_param = {}
        projection_param = {}
        distortion_param["k1"] = self.k1
        distortion_param["k2"] = self.k2
        distortion_param["k3"] = self.k3
        distortion_param['k4'] = self.k4
        distortion_param['k5'] = self.k5
        distortion_param['k6'] = self.k6
        distortion_param['p1'] = self.p1
        distortion_param['p2'] = self.p2
        projection_param["fx"] = self.fx
        projection_param["fy"] = self.fy
        projection_param["cx"] = self.cx
        projection_param["cy"] = self.cy

        fs_param_write.write("model_type", self.model_type)
        fs_param_write.write("camera_name", self.camera_name)
        fs_param_write.write("image_width", int(self.image_width))
        fs_param_write.write("image_height", int(self.image_height))

        fs_param_write.startWriteStruct("distortion_parameters", cv2.FileNode_MAP)
        for key, value in distortion_param.items():
            fs_param_write.write(key, value)
        fs_param_write.endWriteStruct()

        fs_param_write.startWriteStruct("projection_parameters", cv2.FileNode_MAP)
        for key, value in projection_param.items():
            fs_param_write.write(key, value)
        fs_param_write.endWriteStruct()

        np.set_printoptions(suppress=True)
        fs_param_write.write("T_v_c", np.array(self.transform).reshape(4, 4))
        fs_param_write.release()


    def Transformation2EulerAngle(self, transform):
        scale = 180 / np.pi
        roll = scale * np.arctan2(transform[2, 1], transform[2, 2])
        pitch = scale * np.arctan2(-transform[2, 0], np.sqrt(transform[0, 0] * transform[0, 0] + transform[1, 0] * transform[1, 0]))
        yaw = scale * np.arctan2(transform[1, 0], transform[0, 0])
        return [roll, pitch, yaw]


    def EulerAngle2Rotation(self, euler):
        theta = [deg * np.pi / 180.0 for deg in euler]
        R_x = np.array([[1,        0,                 0        ],
                        [0, np.cos(theta[0]), -np.sin(theta[0])],
                        [0, np.sin(theta[0]), np.cos(theta[0]) ]])          
        R_y = np.array([[np.cos(theta[1]),  0, np.sin(theta[1])],
                        [0,                 1,        0        ],
                        [-np.sin(theta[1]), 0, np.cos(theta[1])]])    
        R_z = np.array([[np.cos(theta[2]), -np.sin(theta[2]), 0],
                        [np.sin(theta[2]),  np.cos(theta[2]), 0],
                        [0,                 0,                1]])                
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Add Disturb Calibration Parameter toolkit')
    parser.add_argument('--folder_path', type=str, default='./xxtrinsic/',  help='floder path of xxtrinsic files')
    parser.add_argument('--sensor_name', type=str, default='front_wide',  help='sensor name of yaml, like "front_wide"')
    parser.add_argument('--delta_yaw', type=float, default=0.0, help='disturb of yaw')
    parser.add_argument('--delta_pitch', type=float, default=0.0, help='disturb of pitch')
    parser.add_argument('--delta_roll', type=float, default=0.0, help='disturb of roll') 
    args = parser.parse_args()
    disturb_calib_param = DisturbCalibParam(args)
    disturb_calib_param.GenerateNewParamFIle()
