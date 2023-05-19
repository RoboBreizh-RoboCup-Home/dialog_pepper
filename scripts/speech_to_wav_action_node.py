#!/user/bin/env python
import time
import signal
from queue import Queue
import qi
import sys
from scipy.io import wavfile
from robobreizh_msgs.msg import StringToWavAction, StringToWavFeedback, StringToWavResult
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


class SpeechToWavActionServer():
    def __init__(self, app, module_name):
        app.start()
        session = app.session
        print("Connected to pepper session")
        self.module_name = module_name

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAudioDevice.enableEnergyComputation()
        self.SampleRate = 48000
        self.Channels = 4

        self.init_microphone_variables()

        rospy.init_node('speech_to_text', anonymous=True)
        self._as = actionlib.SimpleActionServer( 
            f'/robobreizh/dialog_pepper/{self.module_name}', 
            StringToWavAction, 
            execute_cb=self.speech_to_wav_cb, 
            auto_start=False
        )

        rospy.loginfo(f'{self.module_name}: action server created'  )
        # starts the action server
        self._as.start()
        rospy.loginfo(f'{self.module_name}: action server started'  )

    def init_microphone_variables(self):
        """
        Initialize variables for microphones 
        """
        #=============== Lists to save the sound data ================#
        self.queueSize = 7
        self.previous_sound_data = Queue(self.queueSize)
        self.q = Queue()

        self.soundData = []
        self.micData = []

        self.request = True

        self.framerate = 48000  # SampleRate=48000

        self.rstCounterSpeech = 4  # number of iteration before starting recording
        self.rstCounterSilence = 8  # number of iteration before stoping recording

        self.counterSpeech = self.rstCounterSpeech
        self.counterSilence = self.rstCounterSilence

        self.timeOutInternalCounter = 140
        self.rstTimeOutInternalCounter = 140

        self.FrontMicImportance = 1.0       # Front Left
        self.LeftMicImportance = 1.0       # Rear Left
        self.RightMicImportance = 1.0      # Rear Right
        self.RearMicImportance = 1.0        # Front Right
        self.status = "Silence"
        self.thOffset = 450
        self.threshold = 0
        self.hh = 0.8
        self.ll = 1 - self.hh

        self.recordingInProgress = False
        self.firstTime = True
        self.no_sentence = True
        self.sentence = ""

        self.init_time = time.time()
        self.frequency = []
        self.counter = 0


    def speech_to_wav_cb(self,goal):
        rospy.loginfo(f'{self.module_name}: enter cb')
        # Run code until it timesout or is canceled
        timeout = rospy.Duration.from_sec(goal.timeout)
        beginning = rospy.get_rostime()

        self._detection_feedback = StringToWavFeedback()
        self._detection_result = StringToWavResult()

        self.ALAudioDevice.setClientPreferences( self.module_name, self.SampleRate, self.Channels, 0)
        rospy.loginfo(f'{self.module_name}: Subscribing to al audio device')
        self.ALAudioDevice.subscribe(self.module_name)
        rospy.loginfo(f'{self.module_name}: Subscribed to al audio device')

        while rospy.get_rostime() - beginning < timeout or self.no_sentence:
            rospy.loginfo(f'{self.module_name}: within callback loop')

            if self._as.is_preempt_requested():
                self.cleanup()
                rospy.loginfo('%s: Preempted' % self.module_name)
                self._as.set_preempted()
                # force to timeout
                timeout = rospy.Duration.from_sec(0)

        if not self.no_sentence:
            rospy.loginfo(f'{self.module_name}: succeed')
            self._detection_result = self._detection_feedback.state
            self._as.set_succeeded(self._door_detection_result)    

        self.ALAudioDevice.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        """
        Fills the queue with data from the microphones
        """
        self.counter += 1
        #====== Audio stream callback method with simple silence detection =========#
        self.soundData = inputBuffer

        self.energy = (((self.ALAudioDevice.getFrontMicEnergy()*self.FrontMicImportance) +
                        (self.ALAudioDevice.getLeftMicEnergy()*self.LeftMicImportance) +
                        (self.ALAudioDevice.getRightMicEnergy()*self.RightMicImportance) +
                        (self.ALAudioDevice.getRearMicEnergy()*self.RearMicImportance))
                       / 4)

        #============================ First Iteration ==============================#
        if (self.firstTime):
            rospy.loginfo(B+"[Robobreizh - Dialog] Sound detection in progress ..."+W)
            self.ymin_prev = self.energy
            self.ymax_prev = self.energy
            self.ymed_prev = self.energy
            self.firstTime = False
        #///////////////////////////////////////////////////////////////////////////#

        if (self.energy > self.ymax_prev):
            self.ymax = self.energy
        else:
            self.ymax = self.hh * self.ymax_prev + self.ll * self.ymed_prev

        if (self.energy < self.ymin_prev):
            self.ymin = self.energy
        else:
            self.ymin = self.ll * self.ymin_prev + self.hh * self.ymed_prev

        self.ymed = (self.ymin + self.ymax) / 2

        #============================ Possible States ==============================#
        if (self.status == "Silence"):
            if (self.energy > self.ymed_prev + self.thOffset):
                self.status = "possibleSpeech"
                self.threshold = self.ymed_prev + self.thOffset
                self.counterSpeech = self.rstCounterSpeech - 1

        elif (self.status == "possibleSpeech"):
            rospy.loginfo(O+"[Robobreizh - Dialog] Possible speech ..."+W)
            self.counterSpeech -= 1
            if (self.energy > self.threshold and self.energy > self.ymed):
                if (self.counterSpeech <= 0):
                    self.counterSpeech = self.rstCounterSpeech
                    self.status = "Speech"
                    self.start_recording()
                    self.timeOutInternalCounter = self.rstTimeOutInternalCounter - self.rstCounterSpeech
                else:
                    self.status = "possibleSpeech"
            else:
                self.status = "Silence"

        elif (self.status == "Speech"):
            rospy.loginfo(O+"[Robobreizh - Dialog] Speech ..."+W)
            if (self.energy < self.ymed and self.energy < self.threshold):
                self.status = "possibleSilence"
                self.threshold = self.ymed
                self.counterSilence = self.rstCounterSilence - 1
            else:
                self.status = "Speech"

        elif (self.status == "possibleSilence"):
            self.counterSilence -= 1
            if (self.energy > self.threshold):
                self.status = "Speech"
            elif (self.counterSilence == 0):
                self.status = "Silence"
                self.stop_recording()
            else:
                self.status = "possibleSilence"

        else:
            self.status = "Silence"

        #///////////////////////////////////////////////////////////////////////////#

        #=========== Way out in case of spending a lot of time listening ===========#
        if(self.status != "Silence"):
            self.timeOutInternalCounter -= 1

        if(self.timeOutInternalCounter == 0):
            self.status = "Time limit reached"
            self.timeOutInternalCounter = self.rstTimeOutInternalCounter
            self.recordingInProgress = False
            self.micData = []
            self.previous_sound_data = Queue(self.queueSize)
            print("SPEECH IS TAKING MORE TIME THAN EXPECTED")
            text = "Speech is taking more time than expected. Try again, i am listening"
            self.aLTextToSpeech.say(text)
            self.status = "Silence"
        #///////////////////////////////////////////////////////////////////////////#

        self.ymin_prev = self.ymin
        self.ymax_prev = self.ymax
        self.ymed_prev = self.ymed

        if self.recordingInProgress:
            self.micData += self.soundData
        else:
            if self.previous_sound_data.full():
                self.previous_sound_data.get()

            self.previous_sound_data.put(self.soundData)

    def start_recording(self):
        #=================== Retrieve the previous buffer data =====================#
        self.recordingInProgress = True
        self.init_time = time.time()

        while not self.previous_sound_data.empty():
            self.micData += self.previous_sound_data.get()

        rospy.loginfo(B+"[Robobreizh - Dialog] Recording message..."+W)
    
    def stop_recording(self):
        rospy.loginfo(B+"[Robobreizh - Dialog] Stopping recording ..."+W)
        current = time.time() - self.init_time
        #==================== Saves the recording to memory ========================#
        self.micData = self.convert_str_to_int(self.micData)

        wav_filename = "stereofile.wav"
        Data = self.float_to_pcm(self.micData, 'int16')
        wavfile.write(wav_filename, self.SampleRate, Data)

        #==================== Saves the recording to memory ========================#
        self.sentence = self.vosk_reco.speech_to_text(wav_filename)

        print("\n\n------------------------\n\n")
        print("Recognized text: " + self.sentence)

        self.recordingInProgress = False
        self.micData = []
        self.previous_sound_data = Queue(self.queueSize)
        self.no_sentence = False

        freq = current/self.counter
        print("Frequency: ", freq)

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
    MySpeechToText = SpeechToWavActionServer(app, module_name)

    app.session.registerService(module_name, MySpeechToText)
    rospy.spin()

#=====================================================================================================================#
#=====================================================================================================================#
