
import pyaudio
import queue
import threading
import wave
import os
import sys
import time
import multiprocessing as mp
import numpy as np
import cv2
import rospy
from std_msgs.msg import Empty, String

from viewer import hl2ss_mp
from viewer import hl2ss_lnm
from viewer import hl2ss
from viewer import hl2ss_utilities
from viewer import hl2ss_3dcv
from datetime import datetime

# Class to track frame timestamps
class FRAMESTAMP:
    CURRENT = None
    PREVIOUS = None

class CACTI_DATASET_STREAM_WRITER():
    def __init__(self, host, output_dir,
                 setting="", condition="", experiment="", participant="", classname=""):

        # Initialize producer
        self.producer = hl2ss_mp.producer()

        # Initialize Remote Configuration Client
        self.client_rc = hl2ss_lnm.ipc_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
        self.start_configuration_client()

        # Initialize consumer
        self.consumer = hl2ss_mp.consumer()

        # Define HoloLens IP Address
        self.host = host

        # Define Stream Ports
        self.ports = [
            hl2ss.StreamPort.RM_VLC_LEFTFRONT,
            hl2ss.StreamPort.RM_VLC_LEFTLEFT,
            hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
            hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
            hl2ss.StreamPort.MICROPHONE
        ]

        # Create Output Directory
        # self.dir_map = self.create_output_directories(output_dir)
        self.output_dir = output_dir
        self.setting = setting
        self.condition = condition
        self.experiment = experiment
        self.participant = f"P00{participant}"
        self.classname = classname

        # Define VLC Stream Settings --------------------------------------------------------------------
        self.vlc_mode = hl2ss.StreamMode.MODE_0             # Mode 0 (Video Only)
        self.vlc_profile = hl2ss.VideoProfile.H264_BASE     # H.264 Base Profile
        self.vlc_level = hl2ss.H26xLevel.H264_3             # H.264 Level 3
        self.vlc_bitrate = 2*1024*1024                      # 2 Mbps
        self.vlc_fps = hl2ss.Parameters_RM_VLC.FPS          # 30 FPS
        self.vlc_width = hl2ss.Parameters_RM_VLC.WIDTH      # 640
        self.vlc_height = hl2ss.Parameters_RM_VLC.HEIGHT    # 480

        # Define Microphone Settings
        self.mic_profile = hl2ss.AudioProfile.RAW           # uint16
        self.mic_format = pyaudio.paInt16 if (self.mic_profile == hl2ss.AudioProfile.RAW) else pyaudio.paFloat32 # RAW format is s16 packed, AAC decoded format is f32 planar
        self.mic_channels = hl2ss.Parameters_MICROPHONE.CHANNELS
        self.mic_sample_rate = hl2ss.Parameters_MICROPHONE.SAMPLE_RATE
        self.audio_buffer = []

        # Configure Streams
        self.configure_streams()

        # Initialize Utilities --------------------------------------------------------------------------
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.buffer_elements = 300    # Maximum number of frames in buffer
        self.manager = mp.Manager()   # Manager for shared memory
        self.sinks = {}               # Sinks for each port
        self.wrote_audio = True       # This is used to check if audio has been written to disk (needs to start true)
        self.record = False           # This is used to check if recording is active
        self.vlc_writers = {}         # Preconfigured VLC VideoWriters for each port
        self.sync_timestamp = None    # This is used to sync the timestamps of the VLC streams

        # Initialize Frame Stamp Map to ensure no duplicate frames are written to disk
        self.frame_stamp = {
            hl2ss.StreamPort.RM_VLC_LEFTFRONT     : FRAMESTAMP(),
            hl2ss.StreamPort.RM_VLC_LEFTLEFT      : FRAMESTAMP(),
            hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : FRAMESTAMP(),
            hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : FRAMESTAMP(),
            hl2ss.StreamPort.MICROPHONE           : FRAMESTAMP()
        }

        # Initialize all writers for each port
        self.writer_map = {
            hl2ss.StreamPort.RM_VLC_LEFTFRONT     : lambda port, payload: self.process_vlc(port, payload, self.vlc_writers.get(port)),
            hl2ss.StreamPort.RM_VLC_LEFTLEFT      : lambda port, payload: self.process_vlc(port, payload, self.vlc_writers.get(port)),
            hl2ss.StreamPort.RM_VLC_RIGHTFRONT    : lambda port, payload: self.process_vlc(port, payload, self.vlc_writers.get(port)),
            hl2ss.StreamPort.RM_VLC_RIGHTRIGHT    : lambda port, payload: self.process_vlc(port, payload, self.vlc_writers.get(port)),
            hl2ss.StreamPort.MICROPHONE           : lambda port, payload: self.process_mic(port, payload)
        }

        # Initialize audio queue and start audio worker thread
        self.audio_queue = queue.Queue()
        self.thread = threading.Thread(target=self.audio_worker, args=(self.audio_queue,))
        try:
            self.thread.start()
        except Exception as e:
            print(f"Error starting audio worker thread. Details: {e}")
            sys.exit(1)

        # Finally initialize streams
        self.initialize_streams()

    """
    Configure Streams --------------------------------------------------------------------------------
    """
    def configure_streams(self):
        self.producer.configure(hl2ss.StreamPort.RM_VLC_LEFTFRONT,
                                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_LEFTFRONT,
                                mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))

        self.producer.configure(hl2ss.StreamPort.RM_VLC_LEFTLEFT,
                                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_LEFTLEFT,
                                mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))

        self.producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
                                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_RIGHTFRONT,
                                mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))

        self.producer.configure(hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
                                hl2ss_lnm.rx_rm_vlc(self.host, hl2ss.StreamPort.RM_VLC_RIGHTRIGHT,
                                mode=self.vlc_mode, profile=self.vlc_profile, level=self.vlc_level, bitrate=self.vlc_bitrate))

        self.producer.configure(hl2ss.StreamPort.MICROPHONE,
                                hl2ss_lnm.rx_microphone(self.host, hl2ss.StreamPort.MICROPHONE,
                                profile=self.mic_profile))

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
            print(f"Initialized port '{port}' to stream '{hl2ss.get_port_name(port)}' sensor data.")

    """
    Create Filename --------------------------------------------------------------------------------
    """
    def create_filename(self, port, filetype=""):
        # Start with empty filename
        filename = ""

        if port == "rm_vlc_leftfront":
            port = "LF"
        if port == "rm_vlc_leftleft":
            port = "LL"
        if port == "rm_vlc_rightfront":
            port = "RF"
        if port == "rm_vlc_rightright":
            port = "RR"
        if port == "microphone":
            port = "MIC"


        filename = f"{self.setting}_{self.condition}_{self.experiment}_{self.participant}_{self.classname}_{port}_"

        # Add timestamp
        filename += self.timestamp

        # Add filetype if it exists
        if filetype:
            # Add dot only if filetype doesn't already have one
            if not filetype.startswith('.'):
                filename += "."
            filename += filetype

        # Join with directory

        condition = 'No_People' if self.condition == 'NP' else 'People'
        experiment = 'No_Sleeve' if self.experiment == 'NS' else 'Sleeve'

        full_path = os.path.join(self.output_dir, self.setting, condition, experiment, self.classname, filename)

        # Ensure directory exists
        os.makedirs(os.path.dirname(full_path), exist_ok=True)

        return full_path

    """
    Create VLC Writers --------------------------------------------------------------------------------
    """
    def create_vlc_writers(self):
        writers = {}
        self.set_timestamp() # call this here because it is the first instance of the timestamp

        # for port, directory in self.dir_map.items():
        for port in self.ports:
            if (port != hl2ss.StreamPort.MICROPHONE):
                # Configure file name with timestamp
                filename = self.create_filename(hl2ss.get_port_name(port), filetype=".mp4")

                # Configure video writer based on stream type
                writers[port] = cv2.VideoWriter(filename,
                    cv2.VideoWriter_fourcc('m','p','4','v'), # MPEG-4 codec
                    self.vlc_fps,                            # 30 FPS
                    (self.vlc_height, self.vlc_width))       # 480, 640 (this is the order of the dimensions and the frame is rotated before written)
        return writers

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

        except Exception as e:
            print(f"Cannot Connect to HoloLens! ({e})")
            sys.exit(1)

    """
    Audio Worker ----------------------------------------------------------------------------------------
    """
    def audio_worker(self, audio_queue):
        while True:
            try:
                data = audio_queue.get(timeout=5)  # Get data from queue with timeout
                if data == b'': break              # Check for empty bytes as termination signal
                self.audio_buffer.append(data)
            except queue.Empty:
                pass                               # No data received, continue and wait for next frame

    """
    Write Audio ----------------------------------------------------------------------------------------
    """
    def write_audio(self):
        try:
            # Save the recorded data as a WAV file
            # for port, directory in self.dir_map.items():
            for port in self.ports:
                if (port == hl2ss.StreamPort.MICROPHONE):
                    filename = self.create_filename(hl2ss.get_port_name(port), filetype=".wav")
                    with wave.open(filename, 'wb') as wave_file:
                        wave_file.setnchannels(self.mic_channels)
                        wave_file.setsampwidth(2 if self.mic_format == pyaudio.paInt16 else 4)
                        wave_file.setframerate(self.mic_sample_rate)
                        wave_file.writeframes(b''.join(self.audio_buffer))

            # Clear the audio frames
            self.audio_buffer.clear()
            self.wrote_audio = True
        except Exception as e:
            print(f"Error writing audio! Details: {e}")


    def set_timestamp(self):
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    """
    Process VLC ----------------------------------------------------------------------------------------
    """
    def process_vlc(self, port, payload, writer):

        if (payload.image is not None and payload.image.size > 0):
            if writer is not None:
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
    Process Microphone --------------------------------------------------------------------------------
    """
    def process_mic(self, port, payload):
        try:
            audio_bits = hl2ss_utilities.microphone_planar_to_packed(payload) if (self.mic_profile != hl2ss.AudioProfile.RAW) else payload
            self.audio_queue.put(audio_bits.tobytes())
        except Exception as e:
            print(f"Error processing microphone data! Details: {e}")

    """
    Process Streams -----------------------------------------------------------------------------------
    """
    def process_streams(self):
        try:
            while not rospy.is_shutdown():
                if (self.record):
                    # Initialize VLC Writers
                    if (len(self.vlc_writers) == 0):
                        self.vlc_writers = self.create_vlc_writers()
                        self.wrote_audio = False

                    # Grab Data
                    for port in self.ports:

                        # Sync timestamps based on the first port in the list
                        if port == self.ports[0] and port != hl2ss.StreamPort.MICROPHONE:
                            self.frame_stamp[port].CURRENT, data = self.sinks[port].get_most_recent_frame()
                            self.sync_timestamp = data.timestamp
                        elif port != hl2ss.StreamPort.MICROPHONE:
                            self.frame_stamp[port].CURRENT, data = self.sinks[port].get_nearest(self.sync_timestamp)
                        else:
                            self.frame_stamp[port].CURRENT, data = self.sinks[port].get_most_recent_frame()    
                        
                        # Process frame if it is not None and the frame stamp has changed
                        if (data is not None and self.frame_stamp[port].CURRENT != self.frame_stamp[port].PREVIOUS):
                            self.writer_map[port](port, data.payload)
                            self.frame_stamp[port].PREVIOUS = self.frame_stamp[port].CURRENT
                else:
                    # Write audio
                    if (not self.wrote_audio):
                        self.write_audio()

                    # Release video writers
                    if (len(self.vlc_writers) > 0):
                        for writer in self.vlc_writers.values():
                            if writer is not None:
                                writer.release()
                        self.vlc_writers = {}

        finally:
            # Stop streams
            for port in self.ports:
                self.sinks[port].detach()
                self.producer.stop(port)
                print(f"Closed port '{port}'")

            # Close Remote Configuration Client
            self.client_rc.close()

            # Signal audio worker to terminate
            self.audio_queue.put(b'')

            # Wait for audio worker to terminate
            self.thread.join()

    """
    Start Callback ------------------------------------------------------------------------------------
    """
    def start_callback(self, msg):
        self.record = True
        print("Recording ...")

    """
    Start Callback Class ------------------------------------------------------------------------------
    """
    def start_callback_class(self, msg):
        self.record = True
        self.classname = msg.data
        print("Recording ...")

    """
    Stop Callback -------------------------------------------------------------------------------------
    """

    def stop_callback(self, msg):
        self.record = False
        print("Stopped recording.")


def main():
    # Initialize ROS node
    rospy.init_node('cacti_dataset_streamer')

    # Get host parameter from ROS parameter server, default to localhost if not set
    host = rospy.get_param('~host', '192.168.0.33')
    output_dir = rospy.get_param('~output_dir', '/project/ws_dev/src/hl2ss/hl2ss_ros/dataset') # no trailing slash
    setting = rospy.get_param('~setting', '')
    condition = rospy.get_param('~condition', '')
    experiment = rospy.get_param('~experiment', '')
    participant = rospy.get_param('~participant', '')
    classname = rospy.get_param('~classname', '')

    # Print out status
    print(f"Connecting client to HoloLens at: '{host}'")
    print(f"Dataset will be saved to: '{output_dir}'")

    # Initialize Streamer
    stream_writer = CACTI_DATASET_STREAM_WRITER(host,
                                                output_dir,
                                                setting=setting,
                                                condition=condition,
                                                experiment=experiment,
                                                participant=participant,
                                                classname=classname)

    # Create subscribers
    rospy.Subscriber('/hri_cacti/dataset_capture/start', Empty, stream_writer.start_callback)
    rospy.Subscriber('/hri_cacti/dataset_capture/stop', Empty, stream_writer.stop_callback)
    rospy.Subscriber('/hri_cacti/dataset_capture/start/class', String, stream_writer.start_callback_class)

    # Process Streams
    stream_writer.process_streams()

if __name__ == '__main__':
    main()
