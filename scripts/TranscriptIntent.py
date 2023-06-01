#!/user/bin python3
import rospy

from dialog_utils.utils import *
import os
from robobreizh_msgs.srv import *
# from dialog_pepper.srv import *
from predict_robot import CommandProcessor
import spacy
import json 
import re
import ast

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
            parser_intent = self.parser.predict(req.transcript.replace(", "," , ").split())
            print(parser_intent)


            parser_intent = re.sub("''","'",parser_intent)
            begin_lst = [m.start() for m in re.finditer('{',parser_intent)]
            end_lst = [m.start() for m in re.finditer('}',parser_intent)]

            task_lst = [parser_intent[b:e+1] for b,e in zip(begin_lst,end_lst)]
            task_descr_lst = task_lst.copy()
            for i,task in enumerate(task_lst):
                task_dict = ast.literal_eval(task)
                task_dict_copy = task_dict.copy()
                for k in task_dict.keys():
                    if k == 'intent':
                        continue
                    words = task_dict[k]

                    if 'room' in words: # don't need to parse the room
                        continue

                    if 'table' in words and len(words.split())>1: # for cases like 'end table', etc
                        continue

                    if len(words.split()) > 1:
                        doc = self.spacy_descr(words)
                        dep_lst = [token.dep_ for token in doc]
                        task_dict_copy[k] = words.split()[dep_lst.index('ROOT')]

                        if 'left most' or 'right most' in words:
                            task_dict_copy.update({k+'_descr' : ' '.join(words.split()[:2])})
                            task_dict_copy.update({k : ' '.join(words.split()[2:])})

                        if len(words.split()) >= 3:
                            if dep_lst[1] == 'prep' and dep_lst[2] == 'pobj':
                                task_dict_copy.update({k : words.split()[dep_lst.index('ROOT')]})
                                task_dict_copy.update({k+'_position' : words.split()[1]})
                                task_dict_copy.update({k+'_position_obj' : words.split()[2]})

                        if dep_lst[0] == 'amod' and dep_lst[1] == 'ROOT':
                            task_dict_copy.update({k+'_descr' : words.split()[0]})

                        if dep_lst[0] == 'ROOT' and dep_lst[1] == 'acl':
                            descr = ' '.join(words.split()[1:])
                            # task_dict_copy.update({k+'_descr' : descr})
                            task_dict_copy.update({k+'_descr_verb' : words.split()[1]})
                            if 'amod' in dep_lst:
                                task_dict_copy.update({k+'_descr_adj' : words.split()[dep_lst.index('amod')]})
                            if 'dobj' in dep_lst:
                                task_dict_copy.update({k+'_descr_key' : words.split()[dep_lst.index('dobj')]})

                        task_descr_lst[i] = str(task_dict_copy)

            parser_intent = '\n'.join(task_descr_lst)
            # print('\n'.join(task_descr_lst))






            # parser_intent = re.sub("''","'",parser_intent)
            # task_lst = parser_intent.split('{')
            # print(f'task_lst: {task_lst}')
            # task_descr_lst = task_lst.copy()
            # print(task_descr_lst)
            # for i,task in enumerate(task_lst):
            #     task_dict = json.loads(task)
            #     for k in task_dict.keys():
            #         if k == 'intent':
            #             continue
            #         words = task_dict[k]
            #         if len(words.split()) > 1:
            #             doc = self.spacy_descr(words)
            #             dep_lst = [token.dep_ for token in doc]
            #             task_dict[k] = words.split()[dep_lst.index('ROOT')]

            #             if dep_lst[0] == 'amod' and dep_lst[1] == 'ROOT':
            #                 task_dict[k+'_descr'] = words.split()[0]
                            
            #             if dep_lst[0] == 'ROOT' and dep_lst[1] == 'acl':
            #                 task_dict[k+'_descr'] = words.split()[1:]

            #             task_descr_lst[i] = [str(task_dict)]
            
            # print(f'!! task_descr_lst: {task_descr_lst}')
                            


              

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
