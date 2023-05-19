# Dialog Package

## 1. Requirements

- Code for Python 3

- A Vosk model for speech recognition, preferably "vosk-model-small-en-us-0.15". This is in the github repository as model folder


## 2. Description

The code is made to work on the Pepper robot.

The main.py module initializes the SoundProcessingModule, which is what detects and records speech using dinamically adjusted parameters (so that it doesn't record whatever noise is present, only human voice speaking to it).

The SoundProcessingModule recognizes four states: Silence, PossibleSpeech, Speech, and PossibleSilence. It starts recording when it reaches the Speech state (and also saves a pre-recorded buffer so that the first words spoken, in the PossibleSpeech state, aren't lost) and stops when it reaches the Silence state. It then creates a wav file, which is then turned into text by the SpeechToText module using the vosk library. This may take a few seconds, depending on the CPU. The result is saved into a text file. Then the NLPModule takes this text and processes it using methods from the Spacy library. It finds out what type of question is asked (who, where, when, what, how, which) and the information that the speaker wants to konw. Wikipedia.search and Wikipedia.summary from the Wikipedia library are used to look up the question and try to find a correct answer. A string with this answer is what NLPModule returns, or otherwise a string saying that it can't answer the question or if an error occurs, a string explaining what kind of error was.

### 2.1 Services
```
/robobreizh/dialog_pepper/text_to_speech_srv (Msg)
/robobreizh/dialog_pepper/speech_to_intent (Action)
```

### 2.2 Messages

## 3. Usage
The services can be started using 

    rosrun dialog_pepper [service_name].py 

## 4. Documentation
The different intent should be shaped as such :
