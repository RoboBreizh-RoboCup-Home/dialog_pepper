#!/user/bin/env python
#=====================================================================================================================#
#                                               SOUND PROCESSING MODULE                                               #
#=====================================================================================================================#
import time
import signal
from queue import Queue
import numpy
import qi
import sys

from scipy.io import wavfile

import sqlite3

from vosk import Model, KaldiRecognizer
import os
import struct

from datetime import datetime
import json
from dialog_utils.utils import *
import threading
import socket


class SpeechToIntentSrv():
    def __init__(self, app):
        app.start()
        session = app.session
        self.session = session
        print("Connected to pepper session")
        self.module_name = "SpeechToIntentSrv"

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAudioDevice.enableEnergyComputation()
        self.SampleRate = 48000
        self.Channels = 4
        self.aLAnimatedSpeech = session.service("ALAnimatedSpeech")
        
        self.aLTextToSpeech = session.service("ALTextToSpeech")
        self.aLTextToSpeech.setLanguage("English")
        self.aLTextToSpeech.setParameter("volume", 100)
        self.aLTextToSpeech.setParameter("pitch", 100)
        self.aLTextToSpeech.setParameter("speed", 68)

        self.init()

        print("######### INIT DONE ###########")

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

        #=============== VOSK_RECO ================#
        model_path = os.path.join(get_pkg_path(), "models/vosk_model")
        model = Model(model_path)
        self.model = model  # Model(model_path)
        self.framerate = 48000  # SampleRate=48000

        self.isRecognized_partially = False
        self.rec = KaldiRecognizer(self.model, self.framerate)

        t1 = datetime.now()
        self.total_t = t1-t1   
        init_persona = "I want you to act as a personal humanoid robot assistant. Your name is Pepper and you are 1.4 metter tall. You have been created by the company Aldebaran in 2012. Now you are part of RoboBreizh, a french-australian robotics team that compet in the RoboCup@Home competition. Your are able to answer all kind of questions and talk about yourself as much as possible. My first question is: Who are you?"
        #self.request_gpt(init_persona)

    def start_sti_srv(self):
        try:
            while True:
                #self.request = isBooleanInDBTrue()
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
                    print("unsubscribe : ", time.time())
                time.sleep(0.2)
        except Exception as e:
            raise e

#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#
    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        """
        Fills the queue with data from the microphones
        """
        #====== Audio stream callback fills the queue buffer =========#
        self.soundData = convert_str_to_int(inputBuffer)

        pcm_data = float_to_pcm(self.soundData, 'int16')

        self.q.put(bytes(pcm_data))

    def recognize_text(self):
        """
        Thread that reads the queue being filled by processRemote function
        """

        #self.gpt = self.session.service("ChatGPTSrv")

        isRecognized = False
        while not isRecognized:
            #not self.q.empty()

            #self.request = isBooleanInDBTrue()
            if not self.request:
                self.rec.Reset()
                with self.q.mutex:
                    self.q.queue.clear()
                print(
                    f"{B}[RBZH - Dialog]The speech processing thread timed out {W}")
                return

            data = self.q.get()
            t_1 = datetime.now()
            # Accept wav == True -> recognized the whole sentence
            if self.rec.AcceptWaveform(bytes(data)):
                t_two = datetime.now()
                # print(f'final infer: {t_two-t_1}')

                self.total_t += t_two-t_1

                # In case of final result
                result = self.rec.FinalResult()

                diction = json.loads(result)
                lentext = len(diction["text"])

                if lentext > 2:
                    self.request = False

                    result_text = diction["text"]
                    isRecognized = True
                    print(f'inference time: {self.total_t}')
                    print(
                        f"{B}[RZH - Dialog] Recognized text: {W}{result_text}")

                    # Stuuf to do
                    self.request_gpt(result_text)

                    #setBooleanInDBFalse()
                    self.request = True
                else:
                    isRecognized = False
                    # Resets current results so the recognition can continue from scratch
                self.rec.Reset()

            else:
                # In case of partial result
                result_partial = self.rec.PartialResult()
                if (len(result_partial) > 15):

                    t_2 = datetime.now()
                    #print('partial inference: ',t_2-t_1)
                    self.total_t += t_2-t_1

                    self.isRecognized_partially = True
                    partial_dict = json.loads(result_partial)
                    partial = partial_dict["partial"]
                else:
                    # does not seem to ever occur
                    print('nothing')

            if self.isRecognized_partially == True:
                # send out the partial text: print(f'partial result: {string[17:len(string)-3]}')
                print(f'{B}[Robobreizh - Dialog][{threading.get_native_id()}] partial : {W}{partial}')
                # reset self.isRecognized_partially =  False
                self.isRecognized_partially = False

        print(
            f"{B}[Robobreizh - Dialog]The speech processing thread just ended =D {W}")


    def request_gpt(self, sentence):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('192.168.50.236', 8080))

        client.send(str.encode(sentence))

        from_server = client.recv(4096)
        print(f'{B}[Robobreizh - Answer from GPT : {W}{str(from_server)}')
        client.close()

        self.aLAnimatedSpeech.say(str(from_server))


    def parsing_intent(self, result_text):
        parser_intent = self.parsing.classifier(result_text)
        if len(parser_intent) == 0:
            raise rospy.ServiceException
        else:
            to_say = "I recognize "
            intents = parser_intent.split("\n")
            i=intents.pop()
            for command in intents:
                command = command.split("'")
                command = command[1::2]
                to_say = to_say+" the action: "+command[1]
                to_say = to_say+" with arguments: "+command[2]+" "+command[3]+"; "
        
            self.aLAnimatedSpeech = self.session.service("ALAnimatedSpeech")

            #===================== Set ALTextToSpeech ====================#

            self.aLTextToSpeech = self.session.service("ALTextToSpeech")
            self.aLTextToSpeech.setLanguage("English")
            self.aLTextToSpeech.setParameter("volume", 100)
            self.aLTextToSpeech.setParameter("pitch", 100)
            self.aLTextToSpeech.setParameter("speed", 68)

            self.aLAnimatedSpeech.say(to_say)

            parser_intent = parser_intent.split("\n")

        return parser_intent

#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

#=====================================================================================================================#
#                                                       MAIN                                                          #
#=====================================================================================================================#
if __name__ == "__main__":

    #======= Connection and Initialization of qi framework =======#
    try:
        print('-------------------- stw main----------------------------')
        # Initialize qi framework.
        connection_url = "tcp://192.168.50.44:9559"
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
    MySpeechToIntentSrv.start_sti_srv()

    print("Disconnected")
#=====================================================================================================================#
#=====================================================================================================================#