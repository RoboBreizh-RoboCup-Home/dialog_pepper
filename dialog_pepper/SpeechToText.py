#!/user/bin/env python
#=====================================================================================================================#
#                                               SOUND PROCESSING MODULE                                               #
#=====================================================================================================================#
import time
import signal
from queue import Queue
import qi
import sys
from vosk import Model, KaldiRecognizer

from .dialog_utils.utils import *
import os
import threading

from datetime import datetime
import json

from rclpy.node import Node
from std_msgs.msg import String
import rclpy

global START_TIME 
START_TIME = time.time()
class SpeechToText(Node):
    def __init__(self, app):
        super().__init__('stt_node')

        app.start()
        session = app.session
        print("Connected to pepper session")
        self.module_name = "SpeechToIntentSrv"

        #===================== Set ALAudioDevice =====================#
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAudioDevice.enableEnergyComputation()
        self.SampleRate = 48000
        self.Channels = 4

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

        # ROS publisher for the speech to text
        self.pub = self.create_publisher(String, 'speech_to_text', 10)

    def recognize_text(self, time_out=15):
        """
        Thread that reads the queue being filled by processRemote function
        """
        self.q.queue.clear()
        isRecognized = False
        while not isRecognized:
            #not self.q.empty()

            # self.request = isBooleanInDBTrue()
            current_time = time.time()
            global START_TIME
            if current_time - START_TIME > time_out:
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

                    result_text = diction["text"]
                    isRecognized = True
                    print(f'inference time: {self.total_t}')
                    print(
                        f"{B}[RZH - Dialog] Recognized text: {W}{result_text}")
                    self.pub.publish(String(data=result_text))
                    # Write value inside database
                    writeValue(result_text)
                    # setBooleanInDBFalse()
                    self.request = False
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

    def start_sti_srv(self):
        removeValue()
        input("**************** Press Enter to start the speech to text service... ****************")
        # setBooleanInDBTrue()
        global START_TIME 
        START_TIME = time.time()

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
                # self.request = isBooleanInDBTrue()
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

def main():
    
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

    rclpy.init()

    MySpeechToText = SpeechToText(app) 
    app.session.registerService(
        "SpeechToIntentSrv", MySpeechToText)
    
    def handler(signum, frame):
        # deallocate memory to avoid segfault
        MySpeechToText.ALAudioDevice.unsubscribe(
            MySpeechToText.module_name)
        exit(1)

    signal.signal(signal.SIGINT, handler)

    MySpeechToText.start_sti_srv()

if __name__ == "__main__":
    main()


#=====================================================================================================================#
#=====================================================================================================================#
