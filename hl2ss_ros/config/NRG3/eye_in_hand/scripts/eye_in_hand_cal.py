import cv2
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix

# Ref: https://forum.opencv.org/t/eye-to-hand-calibration/5690/9
# def calibrate_eye_hand(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, eye_to_hand=True):

#     if eye_to_hand:
#         # change coordinates from gripper2base to base2gripper
#         R_base2gripper, t_base2gripper = [], []
#         for R, t in zip(R_gripper2base, t_gripper2base):
#             R_b2g = R.T
#             t_b2g = -R_b2g @ t
#             R_base2gripper.append(R_b2g)
#             t_base2gripper.append(t_b2g)

#         # change parameters values
#         R_gripper2base = R_base2gripper
#         t_gripper2base = t_base2gripper

#     # calibrate
#     R, t = cv2.calibrateHandEye(
#         R_gripper2base=R_gripper2base,
#         t_gripper2base=t_gripper2base,
#         R_target2cam=R_target2cam,
#         t_target2cam=t_target2cam,
#     )

#     return R, t
# Ref: https://github.com/JonesCVBS/HandEyeCalibration-using-OpenCV/blob/master/HandEyeCalibration_class.py
#solve hand-eye calibration
# for i in range(0, 5):
#     print("Method:", i)
#     self.R_cam2gripper, self.t_cam2gripper = cv2.calibrateHandEye(
#         self.R_cam2target,
#         self.T_cam2target,
#         self.R_vecEE2Base,
#         self.tEE2Base,
#         method=i
#     )

def eye_in_hand_cal(t_headset2world, q_headset2world, t_target2cam, q_target2cam):

    # Convert quaternion to SO3 rotation matrix
    R_mat_headset2world = [quaternion_matrix(Q)[0:3, 0:3] for Q in q_headset2world]
    R_mat_target2cam = [quaternion_matrix(Q)[0:3, 0:3] for Q in q_target2cam]

    # Convert SO3 rotation matrix to rotation vector (Axis with angle magnitude(rads))
    R_vec_headset2world = [cv2.Rodrigues(R)[0] for R in R_mat_headset2world]
    R_vec_target2cam = [cv2.Rodrigues(R)[0] for R in R_mat_target2cam]

    # Calibrate
    # Order is based on: https://github.com/JonesCVBS/HandEyeCalibration-using-OpenCV/blob/master/HandEyeCalibration_class.py
    R_cam2headset, t_cam2headset = cv2.calibrateHandEye(
        R_gripper2base = R_vec_headset2world,
        t_gripper2base = t_headset2world,
        R_target2cam   = R_vec_target2cam,
        t_target2cam   = t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    # Create SE3 transformation matrix
    TF_cam2headset         = np.eye(4)
    TF_cam2headset[:3, :3] = R_cam2headset
    TF_cam2headset[:3, 3]  = t_cam2headset.flatten()

    # Transform and create SE3 transformation matrix
    TF_headset2cam        = np.eye(4)
    TF_headset2cam[:3, :3] = R_cam2headset.T
    TF_headset2cam[:3, 3] = (-R_cam2headset.T @ t_cam2headset).flatten()

    return TF_cam2headset, TF_headset2cam

def main():
    # Load data
    # data_inv = np.loadtxt('/project/ws_dev/src/hl2ss/hl2ss_ros/config/NRG3/eye_in_hand/data/nrg3_LF_cal_data_100_inv.txt', skiprows=1, delimiter=',')
    # data_inv = np.loadtxt('/project/ws_dev/src/hl2ss/hl2ss_ros/config/NRG3/eye_in_hand/data/nrg3_RF_cal_data_100_inv.txt', skiprows=1, delimiter=',')
    data_inv = np.loadtxt('/project/ws_dev/src/hl2ss/hl2ss_ros/config/NRG3/eye_in_hand/verification/nrg3_baselink_to_rignode_data_inv.txt', skiprows=1, delimiter=',')

    # Headset to World
    t_headset2world = data_inv[:, 0:3]    # x, y, z [m]
    q_headset2world = data_inv[:, 3:7]    # qx, qy, qz, qw [rad]

    # Camera to Target
    t_target2cam    = data_inv[:, 7:10]   # x, y, z [m]
    q_target2cam    = data_inv[:, 10:14]  # qx, qy, qz, qw [rad]

    # Calibrate
    TF_cam2headset, TF_headset2cam = eye_in_hand_cal(t_headset2world, q_headset2world, t_target2cam, q_target2cam)

    # Print
    print(f"Headset to Camera")
    print(np.array2string(TF_headset2cam, precision=4, suppress_small=True))

    # Print
    print(f"Camera to Headset")
    print(np.array2string(TF_cam2headset, precision=4, suppress_small=True))

if __name__ == "__main__":
    main()