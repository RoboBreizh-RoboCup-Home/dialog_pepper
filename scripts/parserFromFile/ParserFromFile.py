import rospy
from dialog_pepper.srv import *
from dialog_utils.utils import *
from parserFromFile.data.common import *

class ParserFromFile():
    def __init__(self):
        self.module_name = "Transcript contains srv"

    def callback(self,req):
        try:

            print(f'{B}[Robobreizh - Dialog] transcript received: {W}{req.transcript}')

            if len(req.transcript) == 0:
                raise rospy.ServiceException

            result = ""
            for word in common_label[req.topic_label]:
                if word.lower() in req.transcript.lower():
                    result = word

            return TranscriptContainsResponse(result.lower())
        except Exception as e:
            raise e

    def start_srv(self):
        rospy.init_node(self.module_name,anonymous=True)
        rospy.Service('/robobreizh/dialog_pepper/transcript_contains_srv', TranscriptContains, self.callback)
        rospy.loginfo(B+"[Robobreizh - Dialog] 'Transcript contains' started"+W)
        rospy.spin()

if __name__ == "__main__":
    dff = ParserFromFile()
    dff.start_srv()
