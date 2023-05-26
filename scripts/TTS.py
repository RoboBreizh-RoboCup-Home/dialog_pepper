import qi
import rospy
import sys
from robobreizh_msgs.srv import Msg, MsgResponse
from dialog_utils.utils import *


class TTS():
    def __init__(self):
        self.first_time = True
        rospy.init_node('tts_srv', anonymous=True)
        session = qi.Session()
        try:
            session.connect("tcp://localhost:9559")
        except RuntimeError:
            print("Can't connect to Naoqi")
            sys.exit(1)

        self.aLAnimatedSpeech = session.service("ALAnimatedSpeech")

        #===================== Set ALTextToSpeech ====================#
        self.aLTextToSpeech = session.service("ALTextToSpeech")
        self.aLTextToSpeech.setLanguage("English")
        self.aLTextToSpeech.setParameter("volume", 100)
        self.aLTextToSpeech.setParameter("pitch", 100)
        self.aLTextToSpeech.setParameter("speed",120)

        self.tts_srv = rospy.Service(
            '/robobreizh/dialog_pepper/text_to_speech', Msg, self.speakCb)
        rospy.loginfo(
            B+"[Robobreizh - Dialog] Text to speech server started"+W)
        rospy.Rate(5)
        rospy.spin()

    def speakCb(self, req):
        val: bool = False
        if req.mode == 0:
            self.aLAnimatedSpeech.say(req.sentence)
            val = True
        elif req.mode == 1:
            self.aLTextToSpeech.say(req.sentence)
            val = True
        else:
            rospy.logerr("[Text To Speech ] - TTS Mode not recognized")
            val = False
        return MsgResponse(val)


if __name__ == "__main__":
    tts = TTS()
