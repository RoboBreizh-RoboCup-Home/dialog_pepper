# ================================================================================================================================== #
#                                                              Parser                                                                #
# ================================================================================================================================== #
from math import fabs
import spacy
from dialog_utils.words import (
    objects_list, destination_list, name_list, sinonyms_list)

import sys
import json


class Parser():

    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")

        # ================== Possible object ================== #
        self.objects = objects_list()

        # ================ Possible destination =============== #
        self.destinations = destination_list()

        # ==================== Possible names ================== #
        self.persons = name_list()

        # ==================== Possible verbs ================== #
        self.verbs = {"go": [], "find": [], "follow": [],
                      "take": [], "place": [], "say": []}
        self.verbs["go"], self.verbs["find"], self.verbs["follow"], self.verbs[
            "take"], self.verbs["place"], self.verbs["say"] = sinonyms_list()

    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def classifier(self, string):

        string = string.lower()
        print("Recognize -------------> ", string)
        self.doc = self.nlp(string)

        action = []
        actions_num = 0
        index = []

        for word in self.doc:
            if word.pos_ == "VERB" and word.text in self.verbs.keys():
                action.append(word.text)
                actions_num += 1
                index.append(list(self.doc).index(word))

            # forced search to save some cases where Spacy does't recognize the verb as "VERB"
            elif (word.text in (self.verbs["go"] + self.verbs["find"] + self.verbs["follow"] + self.verbs["take"] + self.verbs["place"] + self.verbs["say"])):
                action.append(word.text)
                actions_num += 1
                index.append(list(self.doc).index(word))

        self.previousPerson = self.previousObject = self.previousDestination = ""
        self.dic_res = ""
        self.dic = {"intent": "", "person": "", "object": "",
                    "destination": "", "who": "", "what": ""}

        for it in range(actions_num):

            flag_go_add = False
            flag_to = False
            self.index_adp = -1
            self.index_go_add = -1

            if actions_num > 1:
                if it < actions_num - 1:
                    doc = self.doc[index[it]:index[it+1]]
                else:
                    doc = self.doc[index[it]:]
            else:
                doc = self.doc[index[0]:]

            # specials cases for FIND and TAKE verbs
            for word in doc:
                if word.text == "to" and not flag_to:
                    self.index_adp = list(doc).index(word)
                    flag_to = True

                if (word.text in self.destinations[0] and not word.text == "to"):
                    self.index_go_add = list(doc).index(word)
                    flag_go_add = True

            # ================== GO --> place ==================== #
            if action[it] in self.verbs["go"]:
                self.dic["intent"] = self.verbs["go"][0]
                self.dic["destination"] = self.find(doc, "destination")

            # ======= FIND --> object or person (and place) ====== #
            elif action[it] in self.verbs["find"]:

                if flag_go_add:
                    flag_go_add = False

                    action.append("go")
                    self.dic["intent"] = "go"

                    self.dic["destination"] = self.find(doc, "destination")

                    doc = doc[0:self.index_go_add]

                    self.add_action()

                self.dic["person"] = self.find(doc, "person")
                if len(self.dic["person"]) > 0:
                    self.dic["intent"] = "to find someone"

                if not (len(self.dic["person"]) > 0):
                    del self.dic["person"]
                    self.dic["object"] = self.find(doc, "object")
                    self.dic["intent"] = "to find something"

            # ================ FOLLOW --> person ================= #
            elif action[it] in self.verbs["follow"]:
                self.dic["intent"] = self.verbs["follow"][0]
                self.dic["person"] = self.find(doc, "person")

            # =========== TAKE --> object (and place) ============ #
            elif action[it] in self.verbs["take"]:

                if (flag_go_add and not flag_to) or ((flag_go_add and flag_to) and self.index_adp - self.index_go_add < 0):
                    flag_go_add = False

                    action.append("go")
                    self.dic["intent"] = "go"

                    self.dic["destination"] = self.find(doc, "destination")

                    doc = doc[0:self.index_go_add]

                    self.add_action()

                elif (flag_go_add and flag_to) and self.index_adp - self.index_go_add > 0:
                    flag_go_add = False

                    action.append("go")
                    self.dic["intent"] = "go"

                    docd = doc[self.index_go_add:self.index_adp]
                    self.dic["destination"] = self.find(docd, "destination")

                    doc = [*doc[:self.index_go_add], *doc[self.index_adp:]]
                    doc = str(doc)
                    doc = doc.replace("[", "").replace("]",
                                                       "").replace(",", "")
                    doc = self.nlp(doc)

                    self.index_adp -= (self.index_adp - self.index_go_add)
                    self.add_action()

                self.dic["intent"] = self.verbs["take"][0]

                pos = ["back", "there"]
                index_pos = -1
                for word in doc:
                    if word.text in pos:
                        index_pos = pos.index(word.text)
                        break

                if index_pos != -1:

                    if self.previousDestination != "":
                        self.dic["destination"] = self.previousDestination
                    elif index_pos == 0:
                        self.dic["destination"] = '"STARTING POINT"'

                    self.dic["person"] = self.find(doc, "person")
                    if len(self.dic["person"]) == 0:
                        self.dic["object"] = self.find(doc, "object")

                else:

                    if self.index_adp != -1 and self.index_adp <= 2:
                        self.dic["person"] = self.find(doc, "person")
                    else:
                        self.dic["object"] = self.find(doc, "object")

                    for k in self.destinations[0]:
                        if k in doc.text:
                            self.dic["destination"] = self.find(
                                doc, "destination")
                            break
                    if self.dic["destination"] == "":
                        self.dic["destination"] = self.find(doc, "person")

            # ========== PLACE --> object (and place) =========== #
            elif action[it] in self.verbs["place"]:
                self.dic["intent"] = self.verbs["place"][0]
                self.dic["object"] = self.find(doc, "object")
                for k in self.destinations[0]:
                    if k in doc.text:
                        self.dic["destination"] = self.find(doc, "destination")
                        break

            # ================= SAY --> person =================== #
            elif action[it] in self.verbs["say"]:
                self.dic["intent"] = "say"

                whats = ["what", "whats"]
                wh = ["which", "who", "whos", "whose",
                      "hows", "how", "that", "where", "wheres"]

                index_what = -1
                it_what = 0
                it_s = 0
                for word in doc:
                    if word.text in whats:
                        index_what = list(doc).index(word)

                    if word.text in ["'s", " 's"] and doc[it_what-1].text in whats:
                        it_s = 1
                    it_what += 1

                index_wh = -1
                if index_what != -1:
                    for word in doc:
                        if word.text in wh:
                            index_wh = list(doc).index(word)
                            break

                if not action[it] == "ask":
                    if flag_to:
                        if self.index_adp - index[it] == 1:

                            if index_what != -1:
                                self.dic["what"] = doc[index_what+it_s+1:]
                                self.dic["who"] = doc[self.index_adp +
                                                      1: index_what]

                            elif index_wh != -1:

                                self.dic["what"] = doc[index_wh:]
                                self.dic["who"] = doc[self.index_adp + 1: index_wh]

                        elif self.index_adp - index[it] > 1:
                            self.dic["what"] = doc[1: self.index_adp]
                            self.dic["who"] = doc[self.index_adp + 1:]

                    else:
                        if index_what != -1:
                            if index_what == 1:
                                self.dic["what"] = doc[index_what+it_s+1:]
                            else:
                                self.dic["what"] = doc[index_what+it_s+1:]
                                self.dic["who"] = doc[1: index_what]

                        elif index_wh != -1:

                            if index_wh == 1:
                                self.dic["what"] = doc[index_wh:]
                            else:
                                self.dic["what"] = doc[index_wh:]
                                self.dic["who"] = doc[1: index_wh]

                        else:
                            if (doc[1].pos_ in ["PROPN", "PRON"] and doc[1].dep_ != "poss") or doc[1].text in self.persons:
                                self.dic["what"] = doc[2:]
                                self.dic["who"] = doc[1]
                            else:
                                self.dic["what"] = doc[1:]

                else:

                    if str(doc[1]) in ["him", "her"] and self.previousPerson != "":
                        self.dic["who"] = self.previousPerson
                    else:
                        self.dic["who"] = doc[1]

                    if flag_to:
                        self.dic["what"] = doc[3:]

                    else:
                        self.dic["what"] = doc[2:]

                self.dic["what"] = str(self.dic["what"]).rstrip()
                self.dic["what"] = str(self.dic["what"]).lstrip()
                self.dic["who"] = str(self.dic["who"])

            # ================ add a new action ================ #
            self.add_action()

        return self.dic_res

    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def find(self, spacy_doc, category):

        try:

            # Utils for category "Person"
            if category == "person":
                verbs = []
                for v in self.verbs.keys():
                    for w in self.verbs[v]:
                        verbs.append(w)

            # Utils for category "Object"
            if category == "object":
                article_index = -1

                two_verbs = -1
                for word in spacy_doc[1:]:
                    if word.pos_ == "VERB" and not word.dep_ in ["xcomp", "compound"]:
                        two_verbs = list(spacy_doc).index(word)

            dic = ""

            it = 0
            for word in spacy_doc:

                # ======================== Persons ============================ #
                if category == "person":
                    if (((word.pos_ in ["PROPN", "PRON"] or word.text in self.persons) and word.text not in verbs) and not word.text in ["him", "her"]) and word.dep_ != "poss":
                        dic = word.text
                        self.previousPerson = word.text
                        break

                    elif word.text in ["him", "her"]:
                        if self.previousPerson:
                            dic = self.previousPerson
                        else:
                            dic = word.text
                        break

                # ======================== Objects ============================ #
                elif category == "object":

                    if not word.text in ["it"]:

                        if word.text in self.objects.keys():
                            article_index = list(spacy_doc).index(word)

                        if article_index != -1:

                            if two_verbs != -1:
                                w_it = 0
                                for w in spacy_doc[article_index+1:]:
                                    if (w.text in self.destinations[0] and w.text != "to") or (w.text == "to" and ((article_index + 1) + w_it) - two_verbs != 1) or (w.dep_ == "cc"):
                                        break
                                    else:
                                        dic += w.text + " "
                                        w_it += 1
                                self.previousObject = dic
                                break

                            if (word.text in self.objects[spacy_doc[article_index].text]) or (word.dep_ in ["pobj", "dobj"]):
                                if self.index_adp != -1:
                                    dic = spacy_doc[article_index +
                                                    1: self.index_adp].text
                                elif self.index_go_add != -1:
                                    dic = spacy_doc[article_index +
                                                    1: self.index_go_add].text
                                else:
                                    dic = spacy_doc[article_index +
                                                    1: list(spacy_doc).index(word) + 1].text

                                self.previousObject = dic
                                break

                        # forced search when there is no article or expression introduccing the object
                        elif word.dep_ in ["pobj", "dobj"]:
                            if self.index_adp != -1:
                                dic = spacy_doc[1: self.index_adp].text
                            elif self.index_go_add != -1:
                                dic = spacy_doc[1: self.index_go_add].text
                            else:
                                dic = spacy_doc[1: list(
                                    spacy_doc).index(word) + 1].text

                            self.previousObject = dic
                            break

                    else:
                        if self.previousObject:
                            dic = self.previousObject
                        else:
                            dic = word.text
                        break

                # ====================== Destinations ========================= #
                elif category == "destination":

                    if word.text in self.destinations[1]:
                        dic = word.text
                        self.previousDestination = dic
                        break

                    if self.index_adp < self.index_go_add:
                        if word.pos_ == "ADP" and (word.text in self.destinations[0] and word.text != "to"):
                            for w in spacy_doc[it+2:]:
                                if (w.dep_ == "cc"):
                                    break
                                else:
                                    dic += w.text + " "
                            self.previousDestination = dic
                            break

                    else:
                        if word.pos_ == "ADP" and word.text in self.destinations[0]:
                            for w in spacy_doc[it+2:]:
                                if (w.dep_ == "cc"):
                                    break
                                else:
                                    dic += w.text + " "
                            self.previousDestination = dic
                            break

                it += 1

            return dic

        except not (category in ["person", "object", "destination"]):
            print("Error: category not found")
            return ""

    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

    def add_action(self):

        keys = list(self.dic.keys()).copy()
        for k in keys:
            self.dic[k] = self.dic[k].replace(",", "").rstrip().lstrip()
            if len(self.dic[k]) == 0:
                del self.dic[k]

        if len(self.dic.keys()) > 0:
            self.dic_res += str(self.dic) + '\n'

            self.dic_res = self.dic_res.replace("[", "").replace("]", "")

        self.dic = {"intent": "", "person": "", "object": "",
                    "destination": "", "who": "", "what": ""}

    #////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////#


if __name__ == "__main__":
    f = open('./data/Commands.json')

    data = json.load(f)

    f.close()
    com_lst = data['Commands'].values()
    nlp = Parser()

    try:
        #label_dic  = {"intent":[],"person" : [], "object" : [], "destination" : [], "who" : [], "what" : []}
        place_holder_lst = ['intent', 'person',
                            'object', 'destination', 'who', 'what']
        for i, com in enumerate(com_lst):

            answer = nlp.classifier(com)

            #print("\n---> " + answer + "\n\n-----------------\n")
            print(answer)
            with open('./data/'+str(i)+'.json', 'w') as f:
                json.dump(answer, f)

            print('len answer: ', len(answer), ', type answer: ', type(answer))
            print('----------------------')

    except KeyboardInterrupt:
        sys.exit(1)
