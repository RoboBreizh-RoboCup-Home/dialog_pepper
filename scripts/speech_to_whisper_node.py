#!/user/bin/env python
import time
# from queue import Queue
from collections import deque
import numpy as np
import qi
import sys

import whisper
import os

import json
import threading
import actionlib
import rospkg
import rospy
from std_msgs.msg import String

from dialog_utils.utils import *


def get_pkg_path():
    rp = rospkg.RosPack()
    return(rp.get_path('dialog_pepper'))


class SpeechToWhisperNode():
    def __init__(self, app, name: str):
        self.name = name
        app.start()
        session = app.session
        self.session = session
        print("Connected to pepper session")

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAudioDevice.enableEnergyComputation()
        self.SampleRate = 48000
        self.Channels = 4

        #============= Speaker's recognition parameters ==============#
        self.rstCounterSpeech = 5
        self.rstCounterSilence = 8

        self.counterSpeech = self.rstCounterSpeech
        self.counterSilence = self.rstCounterSilence

        self.timeOutInternalCounter = 140
        self.rstTimeOutInternalCounter = 140

        self.FrontMicImportance = 2.0		# Front Left
        self.LeftMicImportance = 0.25		# Rear Left
        self.RightMicImportance = 0.25		# Rear Right
        self.RearMicImportance = 2.0		# Front Right

        self.status = "Silence"

        self.thOffset = 475
        self.threshold = 0
        self.hh = 0.8
        self.ll = 1 - self.hh

        self.recordingInProgress = False
        self.firstTime = True

        self.stopping = False
        self.init()
        rospy.on_shutdown(self.cleanup)

    def initAngle(self):
        """
        This example uses the setAngles method.
        """
        motion_service = self.session.service("ALMotion")

        motion_service.setStiffnesses("Head", 1.0)

        # Example showing how to set angles, using a fraction of max speed
        names = ["HeadYaw", "HeadPitch"]
        angles = [0.0, -0.3]
        fractionMaxSpeed = 0.2
        motion_service.setAngles(names, angles, fractionMaxSpeed)

        time.sleep(3.0)
        motion_service.setStiffnesses("Head", 0.0)

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

        #=============== VOSK_RECO ================#
        self.model = model  # Model(model_path)
        self.framerate = 48000  # SampleRate=48000

        self.isRecognized_partially = False

        #===================== ROS Stuff =====================#
        self.initTextPublisher()

        rospy.loginfo(
            bcolors.CYAN+"[RoboBreizh - Vision] Waiting for action requests ..."+bcolors.ENDC)

        self._feedback = WhisperFeedback()
        self._result = ChatDemoResult()

        self._action_name = self.name
        self._as = actionlib.SimpleActionServer(
            self._action_name, ChatDemoAction, execute_cb=self.speech_recognition_main, auto_start=False)
        self._as.start()

    def initTextPublisher(self):
        self.pub_robot = rospy.Publisher(
            '/robobreizh/user_text', String, queue_size=1)

    def speech_recognition_main(self, goal):
        t_start = time.time()
        t_end = t_start + goal.max_duration.to_sec()

        while not rospy.is_shutdown() and time.time() < t_end:
            self.ALAudioDevice.setClientPreferences(
                self.module_name, self.SampleRate, self.Channels, 0)
            
            self.ALAudioDevice.startMicrophonesRecording("transcript.wav")
            time.sleep(5)
            self.ALAudioDevice.stopMicrophonesRecording()

            model = whisper.load_model("base")
            self.device_ = "cpu" 
            if torch.cuda.is_available():
                self.get_logger().info("CUDA is available. Using GPU.")
                self.device_ = "cuda"
            model.to(self.device_)
            result = model.transcribe("transcript.wav")
            print(result)

            # self.ALAudioDevice.subscribe(self.module_name)
            # self.reco_thread = threading.Thread( target=self.recognize_text)
            # self.reco_thread.start()
            # self.ALAudioDevice.unsubscribe(self.module_name)
            print("unsubscribe : ", time.time())
            if self._as.is_preempt_requested():
                self.cleanup()
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.reco_thread.join()
                return True
            if self.stopping:
                self.cleanup()
                self._result.time_elapsed = self._feedback.time_elapsed
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)
                self.reco_thread.join()
                return True
            time.sleep(0.2)

        self._result.time_elapsed = self._feedback.time_elapsed
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        rospy.loginfo(
            bcolors.CYAN+"[RoboBreizh - Vision] Waiting for action requests ..."+bcolors.ENDC)

    def recognize_text(self):
        """
        Thread that reads the queue being filled by processRemote function
        """

    def cleanup(self):
        self.session.service("ALAudioDevice").unsubscribe(self.name)


#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#
#=====================================================================================================================#
#                                                       MAIN                                                          #
#=====================================================================================================================#
if __name__ == "__main__":
    node_name = 'speech_to_whisper'
    rospy.init_node(node_name, anonymous=True)
    #======= Connection and Initialization of qi framework =======#
    try:
        print('-------------------- stw main----------------------------')
        # Initialize qi framework.
        connection_url = "tcp://192.168.50.44:9559"
        app = qi.Application(
            [node_name, "--qi-url=" + connection_url])
    except RuntimeError:
        print(f"Can't connect to Naoqi {connection_url}")
        print("Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    #/////////////////////////////////////////////////////////////#

    #============= Running spekaker's recognition ===============#
    MySpeechToIntentSrv = SpeechToWhisperNode(app, name=node_name)
    app.session.registerService(
        node_name, MySpeechToIntentSrv)
    rospy.spin()
    rospy.loginfo(bcolors.CYAN+"[RoboBreizh - Vision] Shutting nlp node"+bcolors.ENDC)

#=====================================================================================================================#
#=====================================================================================================================#
