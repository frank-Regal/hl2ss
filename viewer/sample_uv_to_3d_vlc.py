#------------------------------------------------------------------------------
# Convert UV coordinates to 3D points
#------------------------------------------------------------------------------

from pynput import keyboard

import multiprocessing as mp
import open3d as o3d
import numpy as np
import cv2
import hl2ss
import hl2ss_lnm
import hl2ss_mp
import hl2ss_3dcv

# Convert UV coordinates to 3D points ----------------------------------------
def uv_to_world(u, v, depth, intrinsics, extrinsics, pose):
    # Normalize coordinates
    x = (u - intrinsics[2, 0]) / intrinsics[0, 0]
    y = (v - intrinsics[2, 1]) / intrinsics[1, 1]
    
    # Create ray
    xy1 = np.array([x, y, 1])
    
    # Scale by depth
    ray_length = np.sqrt(np.sum(xy1*xy1))
    point_camera = xy1 * (depth / ray_length)
    
    # Transform to world
    # camera_to_world = pose @ hl2ss_3dcv.rignode_to_camera(extrinsics)
    camera_to_world = hl2ss_3dcv.camera_to_rignode(extrinsics) @ hl2ss_3dcv.reference_to_world(pose)
    point_world = hl2ss_3dcv.transform(point_camera, camera_to_world)
    
    return point_world

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.11.21"

# Port
# Options:
# hl2ss.StreamPort.RM_VLC_LEFTFRONT
# hl2ss.StreamPort.RM_VLC_LEFTLEFT
# hl2ss.StreamPort.RM_VLC_RIGHTFRONT
# hl2ss.StreamPort.RM_VLC_RIGHTRIGHT
port = hl2ss.StreamPort.RM_VLC_RIGHTRIGHT

# Operating mode
# 0: video
# 1: video + rig pose
# 2: query calibration (single transfer)
mode = hl2ss.StreamMode.MODE_1

# Framerate denominator (must be > 0)
# Effective framerate is framerate / divisor
divisor = 1 

# Video encoding profile and bitrate (None = default)
profile = hl2ss.VideoProfile.H264_BASE
bitrate = None

# Calibration path (must exist but can be empty)
calibration_path = '../calibration'

#------------------------------------------------------------------------------

if (mode == hl2ss.StreamMode.MODE_2):
    data = hl2ss_lnm.download_calibration_rm_vlc(host, port)
    print('Calibration data')
    print('Image point to unit plane')
    print(data.uv2xy)
    print('Extrinsics')
    print(data.extrinsics)
    print('Undistort map')
    print(data.undistort_map)
    print('Intrinsics (undistorted only)')
    print(data.intrinsics)
    quit()

enable = True

def on_press(key):
    global enable
    enable = key != keyboard.Key.esc
    return enable

listener = keyboard.Listener(on_press=on_press)
listener.start()

calibration_vlc = hl2ss_3dcv.get_calibration_rm(host, port, calibration_path)

client = hl2ss_lnm.rx_rm_vlc(host, port, mode=mode, divisor=divisor, profile=profile)
client.open()

while (enable):
    data = client.get_next_packet()

    print(f'Frame captured at {data.timestamp}')
    print(f'Sensor Ticks: {data.payload.sensor_ticks}')
    print(f'Exposure: {data.payload.exposure}')
    print(f'Gain: {data.payload.gain}')
    print(f'Pose')
    print(data.pose)
    
    # Get next VLC image -----------------------------------------------------
    data_vlc = client.get_next_packet()
        
    print(data_vlc.pose)

    # Compute UV to 3D points -------------------------------------------------
    uv_points = np.array([[320, 240], [400, 300]])
    depth_points = np.array([1.0, 1.5])
        
    for i in range(uv_points.shape[0]):
        points_world = uv_to_world(uv_points[i, 0], 
                                   uv_points[i, 1], 
                                   depth_points[i], 
                                   calibration_vlc.intrinsics, 
                                   calibration_vlc.extrinsics, 
                                   data_vlc.pose)
        print(f"World Point for [{uv_points[i, 0]}, {uv_points[i, 1]}, {depth_points[i]}]: {points_world}")

    # cv2.imshow('Video', data.payload.image)
    # cv2.waitKey(1)

client.close()
listener.join()