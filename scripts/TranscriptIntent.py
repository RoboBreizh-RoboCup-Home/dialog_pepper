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

            raw_request = req.transcript.split()

            parser_intent = self.parser.predict(req.transcript.replace(", "," , ").split())
            # print(f'model output: {parser_intent}')


            new_names_2023 = ['Adel', 'Angel', 'Axel', 'Charlie', 'Jane', 'Jules', 'Morgan', 'Paris', 'Robin', 'Simone']

            name_lst = ['Alex', 'Charlie', 'Elizabeth', 'Francis', 'Jennifer', 'Linda', 'Mary', 'Patricia', 'Robin', 'Skyler', 'Alex', 'Charlie', 'Francis', 'James', 'John', 'Michael', 'Robert', 'Skyler', 'William', 'everyone'] + new_names_2023

            last_person = None


            # name_lst = ['Alex', 'Charlie', 'Elizabeth', 'Francis', 'Jennifer', 'Linda', 'Mary', 'Patricia', 'Robin', 'Skyler', 'Alex', 'Charlie', 'Francis', 'James', 'John', 'Michael', 'Robert', 'Skyler', 'William', 'everyone']

            # last_person = None


            parser_intent = re.sub("''","'",parser_intent)
            begin_lst = [m.start() for m in re.finditer('{',parser_intent)]
            end_lst = [m.start() for m in re.finditer('}',parser_intent)]

            task_lst = [parser_intent[b:e+1] for b,e in zip(begin_lst,end_lst)]
            task_descr_lst = task_lst.copy()


            for i,task in enumerate(task_lst):
                task_dict = ast.literal_eval(task)
                task_dict_copy = task_dict.copy()


                for k in task_dict.keys():

                    words = task_dict[k]

                    if k == 'intent':
                        continue

                    if k == 'per':
                        name_values = words.split()
                        if len(name_values) == 2:
                            if name_values[0] in name_lst and name_values[1] in name_lst:
                                task_dict_copy.update({k : name_values[0]})
                                task_dict_copy.update({'dest_per' : name_values[1]})

                                # skip the rest
                                task_descr_lst[i] = task_dict_copy_string
                                continue

                    if k == 'dest':
                        if words.split()[0] in name_lst:
                            task_dict_copy.update({k : ' '.join(words.split()[1:])})
                            task_dict_copy.update({k + '_per' : words.split()[0]})
                            if last_person is not None:
                                task_dict_copy.update({'per': last_person})
                        elif 'all the' in words:
                            # all the elders, women, man, people, children
                            task_dict_copy.update({k : ' '.join(words.split()[3:])})
                            task_dict_copy.update({k + '_per' : ' '.join(words.split()[:3])})

                    # ----------- haven't test this yet  ----------------------------------
                    # print('dic len : ',len(task_dict_copy))
                    # print('dic: ',task_dict_copy)
                    # if len(task_dict_copy) == 1: # if there is only one element (eg; no slots detected -> skip)
                    #     continue
                    if task_dict['intent'] == 'guide' and 'leave' in raw_request: # only ask sb to leave has "leave"
                        task_dict_copy['intent'] = 'tell'
                        task_dict_copy['dest'] =  task_dict_copy['per']
                        task_dict_copy.pop('per')
                        task_dict_copy['what'] = "ask leave"

                        task_descr_lst[i] = str(task_dict_copy)

                        if task_descr_lst[-1] == "{'intent': 'take'}": # fix a weird
                            task_descr_lst.pop()
                        continue


                    if task_dict['intent'] == 'guide' and raw_request[-1] == 'back':
                        task_dict_copy['dest'] = "back"
                    # ----------- haven't test this yet  ----------------------------------

                    if task_dict['intent'] == 'tell' and ' '.join(words.split()[:3]) == 'how many people':
                        task_dict_copy['intent'] = 'count'
                        word_lst = words.split()
                        are_idx = word_lst.index('are')
                        the_idx = word_lst.index('the')
                        task_dict_copy['dest'] =  ' '.join(word_lst[the_idx+1:are_idx])
                        task_dict_copy['what'] = ' '.join(word_lst[are_idx+1:])
                        if task_dict_copy['what'] == 'girls':
                            task_dict_copy['what'] = 'female'
                        elif task_dict_copy['what'] == 'boys':
                            task_dict_copy['what'] = 'male'
                        task_descr_lst[i] = str(task_dict_copy)
                        continue

                    if k == 'what' and task_dict['intent'] == 'tell':
                        if words.split()[0] == 'the':
                            words = ' '.join(words.split()[1:])
                        word_lst = words.split()
                        if word_lst[1] == 'person':
                            words = ' '.join([word_lst[0]] + ['of the'] + word_lst[1:])
                        elif word_lst[2] == 'person':
                            words = ' '.join(word_lst[:2] + ['the'] + word_lst[2:])

                        # print('words:', words)
                        doc = self.spacy_descr(words)
                        dep_lst = [token.dep_ for token in doc]
                        # print(dep_lst)
                        if ' '.join(dep_lst[:6]) == 'ROOT prep det pobj prep det':
                            task_dict_copy.update({k : words.split()[dep_lst.index('ROOT')]})
                            if dep_lst[-2] != 'det':
                                task_dict_copy.update({'dest' : ' '.join(words.split()[-2:])})
                            else:
                                task_dict_copy.update({'dest' : words.split()[-1]})
                        # print(task_dict_copy)
                        task_descr_lst[i] = str(task_dict_copy)
                        continue

                    if task_dict['intent'] == 'tell' and ' '.join(words.split()[:3]) == 'how many people':
                        task_dict_copy['intent'] = 'count'
                        word_lst = words.split()
                        are_idx = word_lst.index('are')
                        the_idx = word_lst.index('the')
                        task_dict_copy['dest'] =  ' '.join(word_lst[the_idx+1:are_idx])
                        task_dict_copy['what'] = ' '.join(word_lst[are_idx+1:])
                        if task_dict_copy['what'] == 'girls':
                            task_dict_copy['what'] = 'female'
                        elif task_dict_copy['what'] == 'boys':
                            task_dict_copy['what'] = 'male'
                        task_descr_lst[i] = str(task_dict_copy)
                        continue

                    if k == 'what' and task_dict['intent'] == 'tell':
                        if words.split()[0] == 'the':
                            words = ' '.join(words.split()[1:])
                        word_lst = words.split()
                        if word_lst[1] == 'person':
                            words = ' '.join([word_lst[0]] + ['of the'] + word_lst[1:])
                        elif word_lst[2] == 'person':
                            words = ' '.join(word_lst[:2] + ['the'] + word_lst[2:])

                        print('words:', words)
                        doc = self.spacy_descr(words)
                        dep_lst = [token.dep_ for token in doc]
                        print(dep_lst)
                        if ' '.join(dep_lst[:6]) == 'ROOT prep det pobj prep det':
                            task_dict_copy.update({k : words.split()[dep_lst.index('ROOT')]})
                            if dep_lst[-2] != 'det':
                                task_dict_copy.update({'dest' : ' '.join(words.split()[-2:])})
                            else:
                                task_dict_copy.update({'dest' : words.split()[-1]})
                        print(task_dict_copy)
                        task_descr_lst[i] = str(task_dict_copy)
                        continue

                    else:

                        if k == 'per':
                            last_person = words.split()[-1]

                        if 'room' in words and len(words.split())==2: # don't need to parse the room
                            continue

                        if 'table' in words and len(words.split())==2: # for cases like 'end table', etc
                            continue

                        if len(words.split()) > 1:
                            doc = self.spacy_descr(words)
                            dep_lst = [token.dep_ for token in doc]
                            task_dict_copy[k] = words.split()[dep_lst.index('ROOT')]

                            if 'left most' in words or 'right most' in words:
                                task_dict_copy.update({k+'_descr' : ' '.join(words.split()[:2])})
                                task_dict_copy.update({k : ' '.join(words.split()[2:])})

                            if len(words.split()) >= 3:
                                if 'prep' in dep_lst and 'pobj' in dep_lst:
                                    task_dict_copy.update({k : words.split()[dep_lst.index('ROOT')]})
                                    task_dict_copy.update({k+'_position' : words.split()[dep_lst.index('prep')]})
                                    task_dict_copy.update({k+'_position_obj' : words.split()[dep_lst.index('pobj')]})

                            if dep_lst[0] == 'amod' and dep_lst[1] == 'ROOT':
                                task_dict_copy.update({k+'_descr' : words.split()[dep_lst.index('amod')]})

                            if dep_lst[0] == 'ROOT' and dep_lst[1] == 'acl':
                                descr = ' '.join(words.split()[1:])
                                # task_dict_copy.update({k+'_descr' : descr})
                                task_dict_copy.update({k+'_descr_verb' : words.split()[1]})
                                if 'amod' in dep_lst:
                                    task_dict_copy.update({k+'_descr_adj' : words.split()[dep_lst.index('amod')]})
                                if 'dobj' in dep_lst:
                                    task_dict_copy.update({k+'_descr_key' : words.split()[dep_lst.index('dobj')]})

                    # for greet and introduce
                    task_dict_copy_string = str(task_dict_copy)
                    if "and" in raw_request:
                        if i == 0:
                            raw_request_current = raw_request[0:raw_request.index('and')]
                        else:
                            raw_request_current = raw_request[raw_request.index('and')+1:]
                    # if 'greet' in task_dict_copy_string and 'per' in task_dict_copy_string and 'dest_per' in task_dict_copy_string:
                    if 'greet' in task_dict_copy_string and 'per' in task_dict_copy_string and 'introduce' in raw_request_current:
                        task_dict_copy_string = task_dict_copy_string.replace("greet", "introduce")

                    # print('----------------')
                    # print(task_dict_copy_string)
                    # print(len(task_dict_copy_string))
                    # print(task_dict_copy)
                    # print('----------------')
                    # if len(task_dict_copy) == 1:
                    #     continue # if there is only one element (eg; no slots detected -> skip)
                    # else:
                    task_descr_lst[i] = task_dict_copy_string
            parser_intent = '\n'.join(task_descr_lst)
            
            say_dict_lst = [ast.literal_eval(task) for task in parser_intent.split('\n')]

            to_say = ''
            if len(say_dict_lst) == 1:
                dic = say_dict_lst[0]
                to_say = f"I am going to do the task of {dic['intent']}"
                dic.pop('intent')
                for k,v in dic.items():
                    if k == 'dest':
                        key = 'destination'
                    elif k == 'per':
                        key = 'person'
                    elif k == 'sour':
                        key = 'source'
                    elif k == 'what':
                        key = 'what to say'
                    elif k == 'obj':
                        key = 'object'
                    to_say += f' and the {key} is {v}'

            else:
                for i,dic in enumerate(say_dict_lst):
                    if i == 0:
                        to_say = f"I am going to do the task of {dic['intent']} in the first task"
                        dic.pop('intent')
                        for k,v in dic.items():
                            if k == 'dest':
                                key = 'destination'
                            elif k == 'per':
                                key = 'person'
                            elif k == 'sour':
                                key = 'source'
                            elif k == 'what':
                                key = 'what to say'
                            elif k == 'obj':
                                key = 'object'
                            to_say += f' and the {key} is {v}'
                    if i == 1:
                        to_say += f" I am going to do the task of {dic['intent']} in the second task"
                        dic.pop('intent')
                        for k,v in dic.items():
                            if k == 'dest':
                                key = 'destination'
                            elif k == 'per':
                                key = 'person'
                            elif k == 'sour':
                                key = 'source'
                            elif k == 'what':
                                key = 'what to say'
                            elif k == 'obj':
                                key = 'object'
                            to_say += f' and the {key} is {v}'

                    if i == 2:
                        to_say += f" I am going to do the task of {dic['intent']} in the third task"
                        dic.pop('intent')
                        for k,v in dic.items():
                            if k == 'dest':
                                key = 'destination'
                            elif k == 'per':
                                key = 'person'
                            elif k == 'sour':
                                key = 'source'
                            elif k == 'what':
                                key = 'what to say'
                            elif k == 'obj':
                                key = 'object'
                            to_say += f' and the {key} is {v}'

            
            
            
           

            print(f'to say: {to_say}')


            # print(parser_intent)
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
