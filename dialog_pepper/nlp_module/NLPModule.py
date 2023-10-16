import spacy
import wikipedia
from transformers import pipeline
import re

#=======================================================================================================#  
#                                              NLP MODULE                                               #
#=======================================================================================================# 
class NLPmodule():

    def __init__(self):
        self.nlp = spacy.load("en_core_web_sm")
        self.categories = ['how', 'when', 'who', 'what', 'where', 'which']
        self.error_massage = "I'm not able to asnswer that kind of question"
        self.error_massage2 = "I can't find an answer to that"

#///////////////////////////////////////////////////////////////////////////////////////////////////////#

    def question_filter(self, text):

        self.doc = self.nlp(text)

        dic = {"category":[], "question_search_doc":""}
        flag = True
        sentence = ""
        for token in self.doc:
            if (token.lemma_ in self.categories and flag):
                
                flag = False

                if token.lemma_ == "how": dic["category"] = "how"
                elif token.lemma_ == "when": dic["category"] = "when"
                elif token.lemma_ == "who": dic["category"] = "who"
                elif token.lemma_ == "what": dic["category"] = "what"
                elif token.lemma_ == "where": dic["category"] = "where"
                elif token.lemma_ == "which": dic["category"] = "which" 

            if dic["category"] == "how" and token.lemma_ == "many": dic["category"] += " " + "many"
            elif dic["category"] == "how" and token.lemma_ == "much": dic["category"] += " " + "much"

            if token.pos_ == "NOUN" or token.pos_ == "PROPN" or token.pos_ == "ADJ":
                sentence += token.text + " "

        if (flag): 
            return self.error_massage
        else:
            dic["question_search_doc"] = sentence.rstrip()
            return dic

#///////////////////////////////////////////////////////////////////////////////////////////////////////#

    def search(self, question, questionType, questionSearch):

        try:

            summary = wikipedia.summary(wikipedia.search(questionSearch, 1)[0], auto_suggest=False)

            summary = list(summary)
            flag = False
            for i in range(len(summary)):
                if (summary[i]=='('): flag = True
                if (flag):
                    if (summary[i]==')'): flag = False
                    summary[i] = ""
            summary = "".join(summary)
            summary = re.sub(r'\W', ' ', summary)

            model_name = "bert-large-cased-whole-word-masking-finetuned-squad"
            nlp_qa = pipeline(task="question-answering", model=model_name, tokenizer=model_name, framework="pt", device=-1)

            while (True):
                result = nlp_qa(question=question, context=summary)
                answer = result["answer"]
                
                answer_match = self.verification(questionType, answer)
                if answer_match: break

                summary = re.sub(answer, " ", summary)

                if (len(answer) == 0): return self.error_massage2
                
            return answer

        except wikipedia.exceptions.DisambiguationError as e:
            print(e) 
            return "Disambiguation Error ocurred"
        except wikipedia.exceptions.HTTPTimeoutError as e:
            print(e) 
            return "HTTP Timeout Error ocurred"
        except wikipedia.exceptions.PageError as e:
            print(e) 
            return "Page Error ocurred"
        except wikipedia.exceptions.RedirectError as e:
            print(e) 
            return "Redirect Error ocurred"
        except wikipedia.exceptions.WikipediaException as e:
            print(e) 
            return "Wikipedia Exception ocurred"

#///////////////////////////////////////////////////////////////////////////////////////////////////////#

    def verification(self, questionType, answer):

        doc = self.nlp(answer)
        
        if questionType == "who":
            for ent in doc.ents:
                if ent.label_ == "PERSON": return True

        elif questionType == "where":
            for ent in doc.ents:
                if ent.label_ == "GPE" or ent.label_ == "LOC": return True

        elif questionType == "when":
            for ent in doc.ents:
                if ent.label_ == "DATE" or ent.label_ == "TIME": return True
        
        elif questionType == "how many" or questionType == "how much":
            for words in doc:
                if words.pos_ == "MONEY" or words.pos_ == "QUANTITY" or words.pos_ == "PERCENT": return True

        elif questionType == "what" or questionType == "how" or questionType == "which":
            for words in doc:
                if words.pos_ == "PROPN" or words.pos_ == "NOUN": return True
            
        return False

#///////////////////////////////////////////////////////////////////////////////////////////////////////#