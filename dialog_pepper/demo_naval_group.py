#!/user/bin/env python
#=====================================================================================================================#
#                                               SOUND PROCESSING MODULE                                               #
#=====================================================================================================================#
import time
import signal
from queue import Queue
import numpy
from std_msgs.msg import String
import qi
import sys

from scipy.io import wavfile

import wave
from dialog_utils.utils import *
import os
import sqlite3

from vosk import Model, KaldiRecognizer
from dialog_utils.utils import *
import os
import rospy
import struct

from datetime import datetime
import json
import GPSRparser.Parser as Parser

def handler(signum, frame):
    exit(1)


signal.signal(signal.SIGINT, handler)

class VoskReco():
    def __init__(self,model):
        #model_path = os.path.join(get_pkg_path(), "models/vosk_model/")
        self.model = model #Model(model_path)
        self.framerate = 48000 #SampleRate=48000

        self.isRecognized = False
        self.isRecognized_partially = False

    def speech_to_text(self, pcm_data):
        # convert the registered file in a string using vosk model

        t1 = datetime.now()
        rec = KaldiRecognizer(self.model, self.framerate)
        t2 = datetime.now()
        print(f"----------------Kaldi init time {t2-t1}")
        print('len pcm: ', len(pcm_data))
        t1_before_while = datetime.now()

        # counter = 0
        # while True:
        #     counter += 1
        #     print(f'while counter: {counter}')
        #     if len(pcm_data) >99:
        #         print('in 1st if')
        #         data = pcm_data[:100]
        #         pcm_data = pcm_data[100:]
        #     else:
        #         data = pcm_data
        #         pcm_data = None
        #         print('in else')
            
        #     if len(data) == 0:
        #         print('len == 0, break')
        #         break

        t1 = datetime.now()
        print(f'------before accepting wave: {t1}')
        if rec.AcceptWaveform(pcm_data):
            print('if this one is not showing, accept wave takes forever')
            t2 = datetime.now()
            print(f'Accept Wave time take {t2-t1}')


            # In case of final result
            result = rec.FinalResult()

            diction = json.loads(result)
            lentext = len(diction["text"])

            if lentext > 2:
                result_text = diction["text"]
                rospy.loginfo(result_text)
                self.isRecognized = True
                print('Recognized full!')
            else:
                isRecognized = False
                # Resets current results so the recognition can continue from scratch
            rec.Reset()

        else:
            print('if this one is not showing, accept wave takes forever')
            # In case of partial result
            result_partial = rec.PartialResult()
            if (len(result_partial) > 20):

                self.isRecognized_partially = True
                print('Partial!')
                partial_dict = json.loads(result_partial)
                partial = partial_dict["partial"]
                partial = partial[17:len(string)-3]


        if self.isRecognized == True:
            #send out the full text
            print(f'Recognized Full result_text: {result_text}')
            #reset self.isRecognized == False
            self.isRecognized == False
            return result_text

        if self.isRecognized_partially == True:
            #send out the partial text: print(f'partial result: {string[17:len(string)-3]}')
            print(f'Recognized partial text result: {partial}')
            #reset self.isRecognized_partially =  False
            self.isRecognized_partially + False

            return partial
    

    
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
        print('-----check SpeechToIntentSrv init--------------')
        model_path = os.path.join(get_pkg_path(), "models/vosk_model")
        model = Model(model_path)

        self.model = model #Model(model_path)
        self.framerate = 48000 #SampleRate=48000

        self.isRecognized = False
        self.isRecognized_partially = False
        self.parsing = Parser.Parser()
        self.rec = KaldiRecognizer(self.model, self.framerate)

        self.init()

        print("######### INIT DONE ###########")

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

        self.q = Queue()

    def isBooleanInDBTrue(self):
        """
        Get the boolean value of id one to check if there is a request for sound processing
        """
        conn = sqlite3.connect("/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db")
        cur = conn.cursor()
        cur.execute("SELECT run FROM dialog WHERE id = 1")

        rows = cur.fetchall()

        if rows[0][0] == 1:
            self.request = True
        else:
            self.request = False
        conn.close()

    def setBooleanInDBFalse(self):
        """
        set the process as done via the boolean value in the db
        """
        conn = sqlite3.connect("/home/nao/robobreizh_pepper_ws/src/manager_pepper/manager_db/roboBreizhDb.db")
        print("set to false")
        cur = conn.cursor()
        r = cur.execute("update dialog set run = 0 where id = 1")
        conn.commit()
        conn.close()

    def start_sti_srv(self):
        try:
            while True:
                self.isBooleanInDBTrue()
                if self.request:
                    print("got in")
                    self.ALAudioDevice.setClientPreferences(
                        self.module_name, self.SampleRate, self.Channels, 0)
                    print("subscribe : ", time.time())
                    self.ALAudioDevice.subscribe(self.module_name)
                    while self.request:
                        self.isBooleanInDBTrue()
                    self.ALAudioDevice.unsubscribe(self.module_name)
                    print("unsubscribe : ", time.time())
                time.sleep(0.2)
                self.isBooleanInDBTrue()
        except Exception as e:
            raise e


#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def convert_str_to_int(self, data):
        signedData = []
        ind = 0

        for i in range(0, int(len(data)/2)):
            signedData.append(data[ind]+data[ind+1]*256)

            ind = ind + 2

        for i in range(0, int(len(signedData))):
            if signedData[i] >= 32768:
                signedData[i] = signedData[i]-65536

        for i in range(0, int(len(signedData))):
            signedData[i] = signedData[i]/32767.0

        return signedData

    def recognizer(self):
        print("recognizer start")
        data = self.q.get()
        print(data.type())
        if self.rec.AcceptWaveform(data):
            t2 = datetime.now()
            print(f'Accept Wave time take {t2-t1}')

            # In case of final result
            result = self.rec.FinalResult()

            diction = json.loads(result)
            lentext = len(diction["text"])

            if lentext > 2:
                result_text = diction["text"]
                rospy.loginfo(result_text)
                self.isRecognized = True
                print('Recognized full!')
                print("Results: ", result_text)
            else:
                isRecognized = False
                # Resets current results so the recognition can continue from scratch
            self.rec.Reset()
        else:
            result_partial = self.rec.PartialResult()
            if (len(result_partial) > 20):
                diction = json.loads(result_partial)
                lentext = len(diction["text"])

                result_text = diction["text"]
                rospy.loginfo(result_text)
                self.isRecognized = True
                print('Recognized partial full!')
                print("Results: ", result_text)

#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#
    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        #====== Audio stream callback method with simple silence detection =========#
        #self.soundData = self.convert_str_to_int(inputBuffer)

        num_ints = len(inputBuffer) // 2
        print(num_ints)
        fmt = '<' + 'h' * num_ints # "<" means little-endian, "h" means 16-bit int
        data = struct.unpack(fmt, inputBuffer)

        # signedData = []
        # ind = 0

        # for i in range(0, int(len(data)/2)):
        #     signedData.append(data[ind]+data[ind+1]*256)

        #     ind = ind + 2


        # for i in range(0, int(len(signedData))):
        #     if signedData[i] >= 32768:
        #         signedData[i] = signedData[i]-65536

        print(type(data[0]))

        if self.rec.AcceptWaveform(data):
            t2 = datetime.now()
            print('Accept Wave time take')

            # In case of final result
            result = self.rec.FinalResult()

            diction = json.loads(result)
            lentext = len(diction["text"])

            if lentext > 2:
                result_text = diction["text"]
                rospy.loginfo(result_text)
                self.isRecognized = True
                print('Recognized full!')
                print("Results: ", result_text)
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
            else:
                isRecognized = False
                # Resets current results so the recognition can continue from scratch
            self.rec.Reset()
        else:
            result_partial = self.rec.PartialResult()
            if (len(result_partial) > 20):
                diction = json.loads(result_partial)
                lentext = len(diction["text"])

                result_text = diction["text"]
                rospy.loginfo(result_text)
                self.isRecognized = True
                print('Recognized partial full!')
                print("Results: ", result_text)

        self.q.put(bytes(signedData))
        print("put in queue")



    def processRemote2(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        #====== Audio stream callback method with simple silence detection =========#
        print('-------------------process remote---------------------')
        while not rospy.is_shutdown():
            print('in process remote while')
            self.soundData = self.convert_str_to_int(inputBuffer)
            #print(f'soundData: {self.soundData}')
            pcm_data = self.process_micData_stream()
            print('pcm Data: ',pcm_data)
            words_reco = self.vosk_reco.speech_to_text(pcm_data)
            print(f'stw Sentence: {words_reco}')

            self.micData = self.soundData


        # self.energy = (((self.ALAudioDevice.getFrontMicEnergy()*self.FrontMicImportance) +
        #                 (self.ALAudioDevice.getLeftMicEnergy()*self.LeftMicImportance) +
        #                 (self.ALAudioDevice.getRightMicEnergy()*self.RightMicImportance) +
        #                 (self.ALAudioDevice.getRearMicEnergy()*self.RearMicImportance))
        #             / 4)

        # #============================ First Iteration ==============================#
        # if (self.firstTime):
        #     self.ymin_prev = self.energy
        #     self.ymax_prev = self.energy
        #     self.ymed_prev = self.energy
        #     self.firstTime = False
        # #///////////////////////////////////////////////////////////////////////////#

        # if (self.energy > self.ymax_prev):
        #     self.ymax = self.energy
        # else:
        #     self.ymax = self.hh * self.ymax_prev + self.ll * self.ymed_prev

        # if (self.energy < self.ymin_prev):
        #     self.ymin = self.energy
        # else:
        #     self.ymin = self.ll * self.ymin_prev + self.hh * self.ymed_prev

        # self.ymed = (self.ymin + self.ymax) / 2

        # #============================ Possible States ==============================#
        # if (self.status == "Silence"):
        #     if (self.energy > self.ymed_prev + self.thOffset):
        #         self.status = "possibleSpeech"
        #         self.threshold = self.ymed_prev + self.thOffset
        #         self.counterSpeech = self.rstCounterSpeech - 1

        # elif (self.status == "possibleSpeech"):
        #     print("possibleSpeech")
        #     self.counterSpeech -= 1
        #     if (self.energy > self.threshold and self.energy > self.ymed):
        #         if (self.counterSpeech <= 0):
        #             self.counterSpeech = self.rstCounterSpeech
        #             self.status = "Speech"
                    
        #             #self.start_recording():


        #             self.timeOutInternalCounter = self.rstTimeOutInternalCounter - self.rstCounterSpeech
        #         else:
        #             self.status = "possibleSpeech"
        #     else:
        #         self.status = "Silence"

        # elif (self.status == "Speech"):
        #     print("speech")
        #     if (self.energy < self.ymed and self.energy < self.threshold):
        #         self.status = "possibleSilence"
        #         self.threshold = self.ymed
        #         self.counterSilence = self.rstCounterSilence - 1
        #     else:
        #         self.status = "Speech"

        # elif (self.status == "possibleSilence"):
        #     self.counterSilence -= 1
        #     if (self.energy > self.threshold):
        #         self.status = "Speech"
        #     elif (self.counterSilence == 0):
        #         self.status = "Silence"
        #         #self.stop_recording()
        #     else:
        #         self.status = "possibleSilence"

        # else:
        #     self.status = "Silence"

        # #///////////////////////////////////////////////////////////////////////////#

        # #=========== Way out in case of spending a lot of time listening ===========#
        # if(self.status != "Silence"):
        #     self.timeOutInternalCounter -= 1

        # if(self.timeOutInternalCounter == 0):
        #     self.status = "Time limit reached"
        #     self.timeOutInternalCounter = self.rstTimeOutInternalCounter
        #     self.recordingInProgress = False
        #     self.micData = []
        #     self.previous_sound_data = Queue(self.queueSize)
        #     print("SPEECH IS TAKING MORE TIME THAN EXPECTED")
        #     text = "Speech is taking more time than expected. Try again, i am listening"
        #     self.aLTextToSpeech.say(text)
        #     self.status = "Silence"
        # #///////////////////////////////////////////////////////////////////////////#

        # self.ymin_prev = self.ymin
        # self.ymax_prev = self.ymax
        # self.ymed_prev = self.ymed

        # # self.micData += self.soundData
        # # if self.previous_sound_data.full():
        # #     self.previous_sound_data.get()
        # # self.previous_sound_data.put(self.soundData)
        # if self.recordingInProgress:
        #     self.micData += self.soundData
        # else:
        #     if self.previous_sound_data.full():
        #         self.previous_sound_data.get()
        #     self.previous_sound_data.put(self.soundData)

#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def start_recording(self):
        #=================== Retrieve the previous buffer data =====================#
        print(f'-------Start Recording-------time:{datetime.now()}')
        self.recordingInProgress = True
        while not self.previous_sound_data.empty():
            self.micData += self.previous_sound_data.get()


    def stop_recording(self):
        #==================== Saves the recording to memory ========================#
        
        print(f'-------Stop Recording time:{datetime.now()}')
        self.aLAnimatedSpeech.say("Give me some time to understand what you said.")
        wav_filename = "stereofile.wav"

        t1 = datetime.now()
        Data = self.float_to_pcm(self.micData, 'int16')
        t2 = datetime.now()
        print(f'-------Float-to-PCM time:{t2-t1}')
        
        t1 = datetime.now()
        
        wavfile.write(wav_filename, self.SampleRate, Data)
        t2 = datetime.now()
        print(f'-------Writing Wave File time:{t2-t1}')
        print(f'-------Finish Writing time point: {t2}')

        self.init()
        self.setBooleanInDBFalse()

    def process_micData_stream(self):
        print(f'-------process_micData_stream-------time:{datetime.now()}')
        self.recordingInProgress = True
        #while not self.previous_sound_data.empty():
        #self.micData = self.soundData.copy()

        print(f'----soundData[0]: {self.soundData[0]}')

        pcm_data = self.float_to_pcm(self.soundData, 'int16')
        #self.init()
        self.soundData = []
        #self.setBooleanInDBFalse()
        
        return pcm_data


    def float_to_pcm(self, myrecording, dtype):
        myrecording = numpy.asarray(myrecording)
        i = numpy.iinfo(dtype)
        abs_max = 2 ** (i.bits - 1)
        offset = i.min + abs_max
        return (myrecording * abs_max + offset).clip(i.min, i.max).astype(dtype)

#/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#


#=====================================================================================================================#
#                                                       MAIN                                                          #
#=====================================================================================================================#
if __name__ == "__main__":

    #======= Connection and Initialization of qi framework =======#
    try:
        print('-------------------- stw main----------------------------')
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
    MySpeechToIntentSrv.start_sti_srv()

    print("Disconnected")
#=====================================================================================================================#
#=====================================================================================================================#
