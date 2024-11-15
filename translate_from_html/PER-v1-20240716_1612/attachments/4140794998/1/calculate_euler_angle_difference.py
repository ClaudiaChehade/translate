import numpy as np

def Transformation2EulerAngle(transform):
    scale = 180 / np.pi
    roll = scale * np.arctan2(transform[2, 1], transform[2, 2])
    pitch = scale * np.arctan2(-transform[2, 0], np.sqrt(transform[0, 0] * transform[0, 0] + transform[1, 0] * transform[1, 0]))
    yaw = scale * np.arctan2(transform[1, 0], transform[0, 0])
    return [roll, pitch, yaw]

def calculate_euler_angle_difference(transform_world1, transform_world2):
    transform_matrix1 = np.array(transform_world1).reshape(4, 4)
    transform_matrix2 = np.array(transform_world2).reshape(4, 4)
    euler_angles1 = Transformation2EulerAngle(transform_matrix1)
    euler_angles2 = Transformation2EulerAngle(transform_matrix2)
    print("offline calib:", euler_angles1)
    print("online calib:", euler_angles2)
    euler_angle_difference = np.array(euler_angles2) - np.array(euler_angles1)
    return euler_angle_difference

print("please input of offline calib:")
transform_world1_input = input().strip().split()
transform_world1 = [float(value) for value in transform_world1_input]

print("please input of online calib:")
transform_world2_input = input().strip().split()
transform_world2 = [float(value) for value in transform_world2_input]

euler_angle_difference = calculate_euler_angle_difference(transform_world1, transform_world2)
print("euler angle difference:", euler_angle_difference)