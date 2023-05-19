#!/user/bin/env python
import time
import signal
from queue import Queue
import qi
import sys
from scipy.io import wavfile
from robobreizh_msgs.msg import SpeechToWav
from dialog_utils.utils import *
import os
import threading
import actionlib

import rospy


def handler(signum, frame):
    # deallocate memory to avoid segfault
    MySpeechToText.ALAudioDevice.unsubscribe(
        MySpeechToText.module_name)
    exit(1)


signal.signal(signal.SIGINT, handler)


class SpeechToWavService():
    def __init__(self, app, module_name):
        app.start()
        session = app.session
        print("Connected to pepper session")
        self.module_name = module_name

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")


        rospy.init_node('speech_to_wav_action', anonymous=True)
        self._srv = rospy.Service( f'/robobreizh/dialog_pepper/speech_to_wav_service', 
            SpeechToWav, 
            execute_cb=self.speech_to_wav_cb, 
        )

        rospy.loginfo(f'{self.module_name}: server created'  )


    def speech_to_wav_cb(self, req):

        self._recorder.startMicrophonesRecording("~/.ros/stereofile.wav", "wav", 48000, [1,1,1,1])
        time.sleep(req.timeout)
        self._recorder.stopMicrophonesRecording()

        rospy.loginfo(B+"[Robobreizh - Dialog] Stopping recording ..."+W)

    def cleanup(self):
        self.session.service("ALAudioDevice").unsubscribe(self.name)

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
            ["SpeechToWav", f"--qi-url=tcp://{ip}:9559"])
    except RuntimeError:
        print(f"Can't connect to Naoqi {ip}")
        print("Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    #/////////////////////////////////////////////////////////////#

    #============= Running spekaker's recognition ===============#
    module_name = "SpeechToWav"
    MySpeechToText = SpeechToWavService(app, module_name)

    app.session.registerService(module_name, MySpeechToText)
    rospy.spin()

#=====================================================================================================================#
#=====================================================================================================================#
