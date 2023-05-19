#                                               SOUND PROCESSING MODULE                                               #
#=====================================================================================================================#
import time
import signal
from queue import Queue
import numpy
from std_msgs.msg import String
import qi
import sys

# from scipy.io import wavfile

import wave
from dialog_utils.utils import *
import os
import sqlite3


def handler(signum, frame):
    exit(1)


signal.signal(signal.SIGINT, handler)


class SpeechToIntentSrv():
    def __init__(self, app):
        app.start()
        session = app.session
        print("Connected to pepper session")
        self.module_name = "SpeechToIntentSrv"

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAudioDevice.enableEnergyComputation()
        self.SampleRate = 48000
        self.Channels = 4
        self.aLAnimatedSpeech = session.service("ALAnimatedSpeech")
        self.init()

    def init(self):
        """
        Initialize variables for microphones
        """
        #============= Speaker's recognition parameters ==============#
        self.rstCounterSpeech = 3  # number of iteration before starting recording
        self.rstCounterSilence = 8  # number of iteration before stoping recording

        self.counterSpeech = self.rstCounterSpeech
        self.counterSilence = self.rstCounterSilence

        self.timeOutInternalCounter = 140
        self.rstTimeOutInternalCounter = 140

        self.FrontMicImportance = 2.0       # Front Left
        self.LeftMicImportance = 0.25       # Rear Left
        self.RightMicImportance = 0.25      # Rear Right
        self.RearMicImportance = 2.0        # Front Right

        #=============== Lists to save the sound data ================#
        self.queueSize = 7
        self.previous_sound_data = Queue(self.queueSize)
        self.soundData = []
        self.micData = []
        self.status = "Silence"

        self.thOffset = 400
        self.threshold = 0
        self.hh = 0.8
        self.ll = 1 - self.hh

        self.recordingInProgress = False
        self.firstTime = True
        self.request = False


    def start_sti_action(self):
        try:
            while True:
                if self.request:
                    print("got in")
                    self.ALAudioDevice.setClientPreferences(
                        self.module_name, self.SampleRate, self.Channels, 0)
                    print("subscribe : ", time.time())
                    self.ALAudioDevice.subscribe(self.module_name)
                    while self.request:
                        time.sleep(0.1)
                        self.isBooleanInDBTrue()
                    self.ALAudioDevice.unsubscribe(self.module_name)
                    print("unsubscribe : ", time.time())
                time.sleep(0.2)
                self.isBooleanInDBTrue()
        except Exception as e:
            raise e



#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        #====== Audio stream callback method with simple silence detection =========#
        self.soundData = self.convert_str_to_int(inputBuffer)

        self.energy = (((self.ALAudioDevice.getFrontMicEnergy()*self.FrontMicImportance) +
                        (self.ALAudioDevice.getLeftMicEnergy()*self.LeftMicImportance) +
                        (self.ALAudioDevice.getRightMicEnergy()*self.RightMicImportance) +
                        (self.ALAudioDevice.getRearMicEnergy()*self.RearMicImportance))
                    / 4)

        #============================ First Iteration ==============================#
        if (self.firstTime):
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
            print("possibleSpeech")
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
            print("speech")
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

        # self.micData += self.soundData
        # if self.previous_sound_data.full():
        #     self.previous_sound_data.get()
        # self.previous_sound_data.put(self.soundData)
        if self.recordingInProgress:
            self.micData += self.soundData
        else:
            if self.previous_sound_data.full():
                self.previous_sound_data.get()
            self.previous_sound_data.put(self.soundData)

#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def start_recording(self):
        #=================== Retrieve the previous buffer data =====================#
        self.recordingInProgress = True
        while not self.previous_sound_data.empty():
            self.micData += self.previous_sound_data.get()


    def stop_recording(self):
        """
        Saves the recording to disk space
        """
        self.aLAnimatedSpeech.say("Give me some time to understand what you said.")
        Data = self.float_to_pcm(self.micData, 'int16')

        wav_filename = "stereofile.wav"
        wf=wave.open(f=wav_filename,mode='w')
        wf.setnchannels(nchannels=self.Channels)
        wf.setframerate(framerate=self.SampleRate)
        wf.writeframesraw(data=Data)
        # wavfile.write(wav_filename, self.SampleRate, Data)
        self.init()


#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#


#=====================================================================================================================#
#                                                       MAIN                                                          #
#=====================================================================================================================#
if __name__ == "__main__":

    #======= Connection and Initialization of qi framework =======#
    try:
        # Initialize qi framework.
        connection_url = "tcp://127.0.0.1:9559"
        app = qi.Application(
            ["SpeechToIntentSrv", "--qi-url=" + connection_url])
    except RuntimeError:
        print(f"Can't connect to Naoqi {connection_url}")
        print("Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    #/////////////////////////////////////////////////////////////#

    #============= Running spekaker's recognition ===============#
    MySpeechToIntentSrv = SpeechToIntentSrv(app)
    app.session.registerService(
        "SpeechToIntentSrv", MySpeechToIntentSrv)
    MySpeechToIntentSrv.start_sti_action()

    print("Disconnected")
#=====================================================================================================================#
#=====================================================================================================================#
