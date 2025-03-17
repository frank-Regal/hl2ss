import pyaudio
import queue
import threading
import wave
import os
import sys
import time
import multiprocessing as mp
import numpy as np
from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from viewer import hl2ss_mp
from viewer import hl2ss_lnm
from viewer import hl2ss
from viewer import hl2ss_utilities
from viewer import hl2ss_3dcv
from datetime import datetime

class FRAMESTAMP:
    CURRENT = None
    PREVIOUS = None

class HoloLensMultiSensorStreamer():
    def __init__(self):

        # Initialize ROS node
        rospy.init_node('hololens_multi_sensor_streamer_node', anonymous=True)

        # Get host parameter from ROS parameter server, default to localhost if not set
        self.host =               rospy.get_param('~host', '192.168.11.22')
        self.ag_n =               rospy.get_param('~ag_n', '0')
        self.dataset_path =       rospy.get_param('~dataset_path', '/project/ws_dev/src/hl2ss/hl2ss_ros/dataset') # no trailing slash
        self.write_data_to_file = rospy.get_param('~write_data_to_file', False)
        print(f"Connecting client to HoloLens at: '{self.host}'")

        # Initialize producer
        self.producer = hl2ss_mp.producer()

        # Initialize bridge
        self.bridge = CvBridge()

        # Initialize Remote Configuration Client
        self.client_rc = hl2ss_lnm.ipc_rc(self.host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
        self.start_configuration_client()

        # Initialize consumer
        self.consumer = hl2ss_mp.consumer()

        # Initialize sync frame stamp
        self.sync_timestamp = None

        # Define Stream Ports
        self.ports = [
            # hl2ss.StreamPort.RM_VLC_LEFTLEFT,
            hl2ss.StreamPort.RM_VLC_LEFTFRONT,
            hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
            # hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
        ]

        if self.write_data_to_file:
            self.dir_map = self.create_output_directories(self.dataset_path)
            print(f"Dataset will be saved to: '{self.dataset_path}'")

        # Define VLC Stream Settings --------------------------------------------------------------------
        if hl2ss.StreamPort.RM_VLC_LEFTLEFT in self.ports or \
            hl2ss.StreamPort.RM_VLC_RIGHTRIGHT in self.ports or \
            hl2ss.StreamPort.RM_VLC_LEFTFRONT in self.ports or \
            hl2ss.StreamPort.RM_VLC_RIGHTFRONT in self.ports:

            # Configure VLC Stream Settings
            self.vlc_mode = hl2ss.StreamMode.MODE_0             # Mode 0 (Video Only)
            self.vlc_profile = hl2ss.VideoProfile.H264_BASE     # H.264 Base Profile
            self.vlc_level = hl2ss.H26xLevel.H264_3             # H.264 Level 3
            self.vlc_bitrate = 2*1024*1024                      # 2 Mbps
            self.vlc_fps = hl2ss.Parameters_RM_VLC.FPS          # 30 FPS
            self.vlc_width = hl2ss.Parameters_RM_VLC.WIDTH      # 640
            self.vlc_height = hl2ss.Parameters_RM_VLC.HEIGHT    # 480

        # Define Microphone Settings
        if hl2ss.StreamPort.MICROPHONE in self.ports:
            self.mic_profile = hl2ss.AudioProfile.RAW  # uint16
            self.mic_format = pyaudio.paInt16 if (self.mic_profile == hl2ss.AudioProfile.RAW) else pyaudio.paFloat32 # RAW format is s16 packed, AAC decoded format is f32 planar
            self.mic_channels = hl2ss.Parameters_MICROPHONE.CHANNELS
            self.mic_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
            self.mic_frames = []

        # Configure Streams
        self.configure_streams()

        # Initialize VLC Writers
        if self.write_data_to_file:
            self.vlc_writers = self.create_vlc_writers()

        # Initialize Utilities --------------------------------------------------------------------------
        self.buffer_elements = 150    # Maximum number of frames in buffer
        self.manager = mp.Manager()   # Manager for shared memory
        self.sinks = {}               # Sinks for each port
        self.enable = True
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Initialize Frame Stamps and Writer Maps
        self.frame_stamp = {}
        self.writer_map = {}
        self.image_pub = {}
        for port in self.ports:
            self.frame_stamp[port] = FRAMESTAMP()
            if port == hl2ss.StreamPort.MICROPHONE:
                self.writer_map[port] = lambda port, payload: self.process_mic(port, payload)
            else:
                if self.write_data_to_file:
                    self.writer_map[port] = lambda port, payload: self.process_vlc_writer(port, payload, self.vlc_writers.get(port))
                else:
                    self.writer_map[port] = lambda port, payload: self.process_vlc(port, payload)
                    self.image_pub[port] = rospy.Publisher(f'hololens_ag{self.ag_n}/{hl2ss.get_port_name(port)}/image_raw', Image, queue_size=10)

        # Initialize Audio Queue and Thread
        if hl2ss.StreamPort.MICROPHONE in self.ports:
            self.audio_queue = queue.Queue()
            self.thread = threading.Thread(target=self.audio_worker, args=(self.audio_queue,))
            self.thread.start()

        # Initialize Streams
        self.initialize_streams()

    """
    Configure Streams --------------------------------------------------------------------------------
    """
    def configure_streams(self):
        if hl2ss.StreamPort.RM_VLC_LEFTLEFT in self.ports:
            self.producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT, \
                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_LEFTLEFT, mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))
        if hl2ss.StreamPort.RM_VLC_LEFTFRONT in self.ports:
            self.producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT, \
                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_LEFTFRONT, mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))
        if hl2ss.StreamPort.RM_VLC_RIGHTFRONT in self.ports:
            self.producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, \
                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT, mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))
        if hl2ss.StreamPort.RM_VLC_RIGHTRIGHT in self.ports:
            self.producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, \
                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))
        if hl2ss.StreamPort.MICROPHONE in self.ports:
            self.producer.configure(hl2ss.StreamPort.MICROPHONE, \
                hl2ss_lnm.rx_microphone(self.host, hl2ss.StreamPort.MICROPHONE, profile=self.mic_profile))

    """
    Start Configuration Client --------------------------------------------------------------------------
    """
    def start_configuration_client(self):
        try:
            self.client_rc.open()
            self.client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_LEFTFRONT, hl2ss.InterfacePriority.NORMAL)
            self.client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_LEFTLEFT, hl2ss.InterfacePriority.NORMAL)
            self.client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_RIGHTFRONT, hl2ss.InterfacePriority.NORMAL)
            self.client_rc.set_interface_priority(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT, hl2ss.InterfacePriority.NORMAL)
            self.client_rc.set_interface_priority(hl2ss.StreamPort.MICROPHONE, hl2ss.InterfacePriority.NORMAL)
            self.client_rc.close()
        except Exception as e:
            print(f"Cannot Connect to HoloLens! ({e})")
            sys.exit(1)

    """
    Initialize Streams --------------------------------------------------------------------------------
    """
    def initialize_streams(self):
        for port in self.ports:
            self.producer.initialize(port, self.buffer_elements)
            self.producer.start(port)
            self.sinks[port] = self.consumer.create_sink(self.producer, port, self.manager, None)
            self.sinks[port].get_attach_response()
            while (self.sinks[port].get_buffered_frame(0)[0] != 0):
                pass
            print(f"Started streaming data from port '{port}' for '{hl2ss.get_port_name(port)}'")

    """
    Audio Worker ----------------------------------------------------------------------------------------
    """
    def audio_worker(self, audio_queue):
        while (self.enable):
            self.mic_frames.append(audio_queue.get())
            print(f"Got audio frame")

    """
    Create Output Directory ----------------------------------------------------------------------------
    """
    def create_output_directories(self, output_dir):
        dir_map = {}
        for port in self.ports:
            port_name = hl2ss.get_port_name(port)
            directory = os.path.join(output_dir, port_name)
            if not os.path.exists(directory):
                os.makedirs(directory)
            dir_map[port] = directory
        return dir_map

    """
    Create VLC Writers --------------------------------------------------------------------------------
    """
    def create_vlc_writers(self):
        writers = {}
        self.set_timestamp() # call this here because it is the first instance of the timestamp

        for port, directory in self.dir_map.items():
            if (port != hl2ss.StreamPort.MICROPHONE):
                # Configure file name with timestamp
                filename = f'{directory}/{self.timestamp}.mp4'

                # Configure video writer based on stream type
                writers[port] = cv2.VideoWriter(filename,
                    cv2.VideoWriter_fourcc('m','p','4','v'), # MPEG-4 codec
                    self.vlc_fps,                            # 30 FPS
                    (self.vlc_height, self.vlc_width))       # 480, 640

                print(f"Created video writer for '{port}'")
            else:
                print(f"No video writers created for '{port}'")

        return writers

    """
    Write Audio ----------------------------------------------------------------------------------------
    """
    def write_audio(self):
        # Save the recorded data as a WAV file
        for port, directory in self.dir_map.items():
            if (port == hl2ss.StreamPort.MICROPHONE):
                filename = f'{directory}/{self.timestamp}.wav'
                with wave.open(filename, 'wb') as wave_file:
                    wave_file.setnchannels(self.mic_channels)
                    wave_file.setsampwidth(2 if self.mic_format == pyaudio.paInt16 else 4)
                    wave_file.setframerate(self.mic_sample_rate)
                    wave_file.writeframes(b''.join(self.mic_frames))
                    print(f"Audio saved to {filename}")

    def set_timestamp(self):
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    """
    Process Streams -----------------------------------------------------------------------------------
    """
    def process_streams(self):
        try:
            while not rospy.is_shutdown():
                for port in self.ports:

                    # Get most recent frame for the first port in the list and sync all other ports to this frame
                    if port == self.ports[0]:
                        self.frame_stamp[port].CURRENT, data = self.sinks[port].get_most_recent_frame()
                        self.sync_timestamp = data.timestamp
                    else:
                        self.frame_stamp[port].CURRENT, data = self.sinks[port].get_nearest(self.sync_timestamp)

                    # Process frame if it is not None and the frame stamp has changed
                    if (data is not None and self.frame_stamp[port].CURRENT != self.frame_stamp[port].PREVIOUS):
                        self.writer_map[port](port, data.payload)
                        self.frame_stamp[port].PREVIOUS = self.frame_stamp[port].CURRENT

        finally:
            # Write audio
            if self.write_data_to_file:
                self.write_audio()

                # Cleanup ----------------------------------------------------------
                # Release video writers
                for writer in self.vlc_writers.values():
                    if writer is not None:
                        writer.release()
                print("Released all video writers")

                # Signal audio worker to terminate
                self.audio_queue.put(b'')

                # Wait for audio worker to terminate
                self.thread.join()

            # Stop streams
            for port in self.ports:
                self.sinks[port].detach()
                self.producer.stop(port)
                print(f'Stopped {port}')

    """
    Process VLC ----------------------------------------------------------------------------------------
    """
    def process_vlc_writer(self, port, payload, writer):
        if (payload.image is not None and payload.image.size > 0):
            if self.write_data_to_file and writer is not None:
                try:
                    # Rotate image for correct orientation based on camera port
                    image_corrected = cv2.rotate(payload.image, hl2ss_3dcv.rm_vlc_get_rotation(port))

                    # Convert image to RGB
                    frame = hl2ss_3dcv.rm_vlc_to_rgb(image_corrected)

                    # Write frame to writer
                    writer.write(frame)

                except Exception as e:
                    print(f"Error writing VLC frame for '{port}'! Details: {e}")
    """
    Process VLC ----------------------------------------------------------------------------------------
    """
    def process_vlc(self, port, payload):
        if (payload.image is not None and payload.image.size > 0):
            try:
                # Rotate image for correct orientation based on camera port
                image_corrected = cv2.rotate(payload.image, hl2ss_3dcv.rm_vlc_get_rotation(port))
                img_msg = self.bridge.cv2_to_imgmsg(image_corrected, encoding="mono8")

                # Publish frame
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = hl2ss.get_port_name(port)
                self.image_pub[port].publish(img_msg)

            except Exception as e:
                print(f"Error writing VLC frame for '{port}'! Details: {e}")


    def process_mic(self, port, payload):
        audio_bits = hl2ss_utilities.microphone_planar_to_packed(payload) if (self.mic_profile != hl2ss.AudioProfile.RAW) else payload
        self.audio_queue.put(audio_bits.tobytes())


def main():
    # Initialize Streamer
    streamer = HoloLensMultiSensorStreamer()

    # Process Streams
    streamer.process_streams()

if __name__ == '__main__':
    main()