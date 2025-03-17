#------------------------------------------------------------------------------
# This script receives microphone audio from the HoloLens and records it for 5 seconds.
# Audio stream configuration is fixed to 2 channels, 48000 Hz.
#------------------------------------------------------------------------------

import hl2ss
import hl2ss_lnm
import hl2ss_utilities
import pyaudio
import queue
import threading
import time
import wave
import os

# Settings --------------------------------------------------------------------

# HoloLens address
host = "192.168.0.22"

# Audio encoding profile
profile = hl2ss.AudioProfile.RAW

#------------------------------------------------------------------------------

# RAW format is s16 packed, AAC decoded format is f32 planar
audio_format = pyaudio.paInt16 if (profile == hl2ss.AudioProfile.RAW) else pyaudio.paFloat32
enable = True
frames = []

def pcmworker(pcmqueue):
    global enable
    global audio_format

    while (enable):
        frames.append(pcmqueue.get())

    # Save the recorded data as a WAV file
    filepath = '/project/ws_dev/src/hl2ss/viewer/recorder_videos/'
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with wave.open(filepath+'recorded_audio.wav', 'wb') as wave_file:
        wave_file.setnchannels(hl2ss.Parameters_MICROPHONE.CHANNELS)
        wave_file.setsampwidth(2 if audio_format == pyaudio.paInt16 else 4)
        wave_file.setframerate(hl2ss.Parameters_MICROPHONE.SAMPLE_RATE)
        wave_file.writeframes(b''.join(frames))

    print(f"Audio saved to {filepath+'recorded_audio.wav'}")

pcmqueue = queue.Queue()
thread = threading.Thread(target=pcmworker, args=(pcmqueue,))
thread.start()

client = hl2ss_lnm.rx_microphone(host, hl2ss.StreamPort.MICROPHONE, profile=profile)
client.open()

start_time = time.time()
recording_duration = 5  # seconds

print("Recording started...")
while (time.time() - start_time < recording_duration):
    remaining = int(recording_duration - (time.time() - start_time))
    print(f'Recording for {remaining} seconds...', end='\r')

    data = client.get_next_packet()
    # RAW format is s16 packed, AAC decoded format is f32 planar
    audio = hl2ss_utilities.microphone_planar_to_packed(data.payload) if (profile != hl2ss.AudioProfile.RAW) else data.payload
    pcmqueue.put(audio.tobytes())

print("\nRecording finished!")
client.close()

enable = False
pcmqueue.put(b'')
thread.join()
