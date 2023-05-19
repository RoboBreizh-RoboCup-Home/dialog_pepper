#!/user/bin/env python
#=====================================================================================================================#
#                                               SOUND PROCESSING MODULE                                               #
#=====================================================================================================================#
import time
import signal
from queue import Queue
import qi
import sys

from dialog_utils.utils import *
import os
import threading

from datetime import datetime
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayDimension


def handler(signum, frame):
    # deallocate memory to avoid segfault
    MySpeechToText.ALAudioDevice.unsubscribe(
        MySpeechToText.module_name)
    exit(1)


signal.signal(signal.SIGINT, handler)


class SoundBufferPub():
    def __init__(self, app):
        app.start()
        session = app.session
        print("Connected to pepper session")
        self.module_name = "SoundBufferPub"

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAudioDevice.enableEnergyComputation()
        self.SampleRate = 48000
        self.Channels = 4

        self.init()

    def init(self):
        """
        Initialize variables for microphones and VOSK Recognizer
        """
        #=============== Lists to save the sound data ================#
        self.queueSize = 7
        self.previous_sound_data = Queue(self.queueSize)
        self.q = Queue()

        self.soundData = []
        self.micData = []

        self.request = True

        self.framerate = 48000  # SampleRate=48000

        rospy.init_node('speech_to_text', anonymous=True)

    def recognize_text(self):
        """
        Thread that reads the queue being filled by processRemote function
        and sends it to the topic robobreizh/voice_buffer
        """
        # initialise ros publisher node
        self.pub = rospy.Publisher('robobreizh/voice_buffer', Int16MultiArray, queue_size=10)

        self.q.queue.clear()
        self.framerate_per_buffer = 1024

        self.timer = rospy.Timer(rospy.Duration(float(self.framerate_per_buffer) / float(self.framerate)),
            self.audio_publisher_timer_callback_)
        while not rospy.is_shutdown():
            continue

        print(
            f"{B}[Robobreizh - Dialog]The speech processing thread just ended =D {W}")

    def audio_publisher_timer_callback_(self, event) -> None:
        data = self.q.get()
        # publish data
        audio = np.frombuffer(data, dtype=np.int16)
        audio_msg = Int16MultiArray()
        audio_msg.data = audio.tolist()
        audio_msg.layout.data_offset = 0
        audio_msg.layout.dim.append(
            MultiArrayDimension(label="audio", size=len(audio), stride=1)
        )
        self.pub.publish(audio_msg)

    def start_sti_srv(self):
        try:
            while True:
                if self.request:
                    self.ALAudioDevice.setClientPreferences(
                        self.module_name, self.SampleRate, self.Channels, 0)
                    self.ALAudioDevice.subscribe(self.module_name)
                    time.sleep(1)
                    self.reco_thread = threading.Thread(
                        target=self.recognize_text)
                    self.reco_thread.start()
                    while self.request:
                        time.sleep(0.1)
                    self.ALAudioDevice.unsubscribe(self.module_name)
                time.sleep(0.2)
                self.request = isBooleanInDBTrue()
        except Exception as e:
            raise e

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        """
        Fills the queue with data from the microphones
        """
        #====== Audio stream callback fills the queue buffer =========#
        self.soundData = convert_str_to_int(inputBuffer)

        pcm_data = float_to_pcm(self.soundData, 'int16')

        self.q.put(bytes(pcm_data))


#=====================================================================================================================#
#                                                       MAIN                                                          #
#=====================================================================================================================#
if __name__ == "__main__":

    #======= Connection and Initialization of qi framework =======#
    try:
        print('-------------------- stw main----------------------------')
        # Initialize qi framework.
        ip = "192.168.50.44"
        app = qi.Application(
            ["SpeechToIntentSrv", f"--qi-url=tcp://{ip}:9559"])
    except RuntimeError:
        print(f"Can't connect to Naoqi {ip}")
        print("Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    #/////////////////////////////////////////////////////////////#

    #============= Running spekaker's recognition ===============#
    MySpeechToText =SoundBufferPub(app)

    app.session.registerService(
        "SoundBufferPub", MySpeechToText)

    MySpeechToText.start_sti_srv()

#=====================================================================================================================#
#=====================================================================================================================#
