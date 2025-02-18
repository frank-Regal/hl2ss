#------------------------------------------------------------------------------
# This script receives encoded video from the HoloLens cameras, displays it,
# and saves it to video files.
# Press esc to stop.
#------------------------------------------------------------------------------

from pynput import keyboard
import multiprocessing as mp
import numpy as np
import cv2
from viewer import hl2ss
from viewer import hl2ss_lnm
from viewer import hl2ss_mp
import os
from datetime import datetime
import time
from viewer import hl2ss_3dcv
# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.11.33'

# Video output settings
output_dir = 'recorded_videos'
fps = 30

# Ports
ports = [
    hl2ss.StreamPort.RM_VLC_LEFTFRONT,
    hl2ss.StreamPort.RM_VLC_LEFTLEFT,
    hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
    hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
]

# PV parameters
pv_width     = 760
pv_height    = 428
pv_framerate = 30

# Maximum number of frames in buffer
buffer_elements = 150

#------------------------------------------------------------------------------

def create_video_writers(ports):
    writers = {}
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created output directory: {output_dir}")

    for port in ports:
        port_name = hl2ss.get_port_name(port)
        filename = f'{output_dir}/{port_name}_{timestamp}.mp4'

        # Configure video writer based on stream type
        writers[port] = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc('m','p','4','v'), fps, (480, 640))
        print(f"Created video writer for {port_name}")

    return writers

if __name__ == '__main__':
    if ((hl2ss.StreamPort.RM_DEPTH_LONGTHROW in ports) and (hl2ss.StreamPort.RM_DEPTH_AHAT in ports)):
        print('Error: Simultaneous RM Depth Long Throw and RM Depth AHAT streaming is not supported.')
        quit()

    # Create video writers
    video_writers = create_video_writers(ports)

    # Keyboard events ---------------------------------------------------------
    enable = True

    def on_press(key):
        global enable
        enable = key != keyboard.Key.esc
        return enable

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # Start PV Subsystem if PV is selected ------------------------------------
    # if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
    #     hl2ss_lnm.start_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)

    # Start streams ---------------------------------------------------------
    client_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
    client_rc.open()
    client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.InterfacePriority.HIGHEST)
    client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss.InterfacePriority.HIGHEST)
    client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.InterfacePriority.HIGHEST)
    client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.InterfacePriority.HIGHEST)
    client_rc.close()

    #SETTINGS
    mode = hl2ss.StreamMode.MODE_0 # Mode 0 (Video Only)
    profile = hl2ss.VideoProfile.H264_BASE # H.264 Base Profile
    level = hl2ss.H26xLevel.H264_3 # H.264 Level 3
    bitrate = 2*1024*1024 # 2 Mbps

    producer = hl2ss_mp.producer()
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, mode=mode, profile=profile, level=level, bitrate=bitrate))
    producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, mode=mode, profile=profile, level=level, bitrate=bitrate))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, mode=mode, profile=profile, level=level, bitrate=bitrate))
    producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss_lnm.rx_rm_vlc(host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, mode=mode, profile=profile, level=level, bitrate=bitrate))

    consumer = hl2ss_mp.consumer()
    manager = mp.Manager()
    sinks = {}

    for port in ports:
        producer.initialize(port, buffer_elements)
        producer.start(port)
        sinks[port] = consumer.create_sink(producer, port, manager, None)
        sinks[port].get_attach_response()
        while (sinks[port].get_buffered_frame(0)[0] != 0):
            pass
        print(f'Started {port}')

    # Display and Recording Functions -----------------------------------------
    def display_vlc(port, payload, writer):
        if (payload.image is not None and payload.image.size > 0):
            cv2.imshow(hl2ss.get_port_name(port), payload.image)


    def write_vlc(port, payload, writer):
        if (payload.image is not None and payload.image.size > 0):
            if writer is not None:
                # Rotate image for correct orientation based on camera port ---------------
                image_corrected = cv2.rotate(payload.image, hl2ss_3dcv.rm_vlc_get_rotation(port))
                if (image_corrected.shape[0] == 0):
                    print(f'Image is empty for {port}')
                    return
                frame = hl2ss_3dcv.rm_vlc_to_rgb(image_corrected)
                if (frame.shape[0] == 0):
                    print(f'Frame is empty for {port}')
                    return
                
                # Create directory for this camera if it doesn't exist
                camera_dir = os.path.join('frames', hl2ss.get_port_name(port))
                os.makedirs(camera_dir, exist_ok=True)

                # Save frame as image with timestamp
                timestamp = time.time()
                filename = os.path.join(camera_dir, f'frame_{timestamp:.3f}.png')

                # Print type of image_corrected
                print(f"Type of image_corrected: {type(image_corrected)} | Type of frame: {type(frame)}")
                print(f"Shape of image_corrected: {image_corrected.shape} | Shape of frame: {frame.shape}")
                print(f"Data type of image_corrected: {image_corrected.dtype} | Data type of frame: {frame.dtype}")
                # blank_image = np.zeros((image_corrected.shape[0], image_corrected.shape[1], 3), dtype=np.uint8)
                
                if (writer is not None):
                    writer.write(frame)
                else:
                    print(f'Writer is None for {port}')



    # Create Display Map ----------------------------------------------------
    DISPLAY_MAP = {
        hl2ss.StreamPort.RM_VLC_LEFTFRONT     : lambda port, payload: write_vlc(port, payload, video_writers.get(port)),
        hl2ss.StreamPort.RM_VLC_LEFTLEFT      : lambda port, payload: write_vlc(port, payload, video_writers.get(port)),
        hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : lambda port, payload: write_vlc(port, payload, video_writers.get(port)),
        hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : lambda port, payload: write_vlc(port, payload, video_writers.get(port))
    }

    # Main loop ------------------------------------------------------------
    start_time = time.time()
    try:
        while (enable):
            for port in ports:
                _, data = sinks[port].get_most_recent_frame()
                if (data is not None):
                    DISPLAY_MAP[port](port, data.payload)

            if(time.time() - start_time > 10):
                enable = False

    finally:
        # Cleanup ----------------------------------------------------------
        # Release video writers
        for writer in video_writers.values():
            if writer is not None:
                writer.release()
        print("Released all video writers")

        # Stop streams
        for port in ports:
            sinks[port].detach()
            producer.stop(port)
            print(f'Stopped {port}')

        # Stop PV Subsystem if PV is selected
        if (hl2ss.StreamPort.PERSONAL_VIDEO in ports):
            hl2ss_lnm.stop_subsystem_pv(host, hl2ss.StreamPort.PERSONAL_VIDEO)


