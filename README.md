# RoboBreizh Dialog Package


This package contains models and ROS2 nodes to handle Speech-to-Text and Natural Language Understanding with the Pepper robot in the RoboCup@Home competition. This repository uses the Pepper robot speech processing (through Naoqi) as a backend but it can be easily customized to any robot with a different API.

## 1. Requirements

We use [the VOSK model](https://github.com/alphacep/vosk-api) with [the Kaldi backend](https://github.com/kaldi-asr/kaldi) for efficient Speech-to-Text on the edge. Please make sure you installed Kaldi before running this package. Kaldi and Vosk are already installed in our latest [Gentoo build for the Pepper robot](https://github.com/RoboBreizh-RoboCup-Home/pepper_os_humble).


## 2. Description

The code is made to work on the Pepper robot.

The SpeechToText.py node initializes the SoundProcessingModule, which is what detects and records speech using dynamically adjusted parameters (so that it doesn't record whatever noise is present, only the human voice speaking to it).

The SoundProcessingModule recognizes four states: Silence, PossibleSpeech, Speech, and PossibleSilence. It starts recording when it reaches the Speech state (and also saves a pre-recorded buffer so that the first words spoken, in the PossibleSpeech state, aren't lost) and stops when it reaches the Silence state. The recording is saved continuously into a buffer and then fed to the Speech-to-Text model. The Vosk model performs speech recognition in a stream-like manner, making it possible to start decoding the speech before the person ends their talk, which saves a considerable amount of time and makes the process almost real-time. The final transcript is then sent to a ROS topic.


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
