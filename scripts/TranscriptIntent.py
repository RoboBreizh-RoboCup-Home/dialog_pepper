#!/user/bin/env python
import rospy

from dialog_utils.utils import *
import os
from robobreizh_msgs.srv import *
# from dialog_pepper.srv import *
from predict_robot import CommandProcessor
import spacy



class Intent():
    def __init__(self):
        model_name='distil_bert'
        model_path = os.path.join(get_pkg_path(), 'scripts/quantized_models/distil_bert.onnx')
        slot_classifier_path = os.path.join(get_pkg_path(), 'scripts/numpy_para/slot_classifier')
        intent_token_classifier_path = os.path.join(
            get_pkg_path(), 'scripts/numpy_para/intent_token_classifier')
        pro_classifier_path = os.path.join(get_pkg_path(), 'scripts/numpy_para/pro_classifier')
        self.parser = CommandProcessor(model_path=model_path, slot_classifier_path=slot_classifier_path,
                                       intent_token_classifier_path=intent_token_classifier_path,
                                       pro_classifier_path=pro_classifier_path,quantized = False, gpu = False, model_name=model_name)
        self.module_name = "TranscriptIntent"
        self.spacy_descr = spacy.load('en_core_web_sm')

    def intent_callback(self, req):
        # sti ros service callback
        try:
            rospy.loginfo(B+"[Robobreizh - Dialog] Parsing intent..."+W)

            # # ------ test descr ------
            # req.transcript = 'Find the person wearing a red shirt' #####################
            # # -------------------------
            parser_intent = self.parser.predict(req.transcript.replace(", "," , ").split())
            print(parser_intent)
            rospy.loginfo(B+"[Robobreizh - Dialog] Parsing Done..."+W)
            rospy.loginfo(
                B+"[Robobreizh - Dialog] Recognized text: " + W + parser_intent)

            if len(parser_intent) == 0:
                raise rospy.ServiceException

            parser_intent = parser_intent.split("\n")

            return TranscriptIntentResponse(parser_intent)
        except Exception as e:
            raise e
     
    def sendReady(self):
        """
        TO DO : set this function in the appropriate process or find a way for the manager to tell when everything is loaded
        This process used to be the one to load the slowest thus it would return say that the manager is ready
        """
        pass
        # rospy.wait_for_service('/robobreizh/dialog_pepper/text_to_speech')
        # try:
        #     tts = rospy.ServiceProxy(
        #         '/robobreizh/dialog_pepper/text_to_speech', Msg)
        #     tts("Everything is loaded, you can now send the plan")
        #     return
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s" % e)

    def start_wti_srv(self):
        # starts ros service node

        rospy.init_node(self.module_name, anonymous=True)
        rospy.Service('/robobreizh/dialog_pepper/transcript_intent',
                      TranscriptIntent, self.intent_callback)
        rospy.loginfo(B+"[Robobreizh - Dialog] Wav to intent server started"+W)
        rospy.spin()


if __name__ == "__main__":
    # pass
    transcipt_intent = Intent()
    transcipt_intent.start_wti_srv()
