from collections import deque

import numpy as np
import rospy
import torch
import whisper
from std_msgs.msg import Int16MultiArray, String


class WhisperInferenceNode():
    def __init__(self, node_name: str) -> None:
        print(whisper.available_models())
        self.whisper_model_ = whisper.load_model("base")
        self.whisper_options_ = whisper.DecodingOptions(language="english", fp16=False)

        self.inference_period_ = 6.0
        self.audio_activation_threshold_ = 0.3

        # Subscribe to ros topic /robobreizh/voice_buffer
        self.sub = rospy.Subscriber("/robobreizh/voice_buffer", Int16MultiArray, self.audio_buffer_callback_)

        self.audio_buffer_ = deque( maxlen=int(48000.0 / 1024 * self.inference_period_))  # buffer length to record self.inference_period_ seconds

        self.text_pub = rospy.Publisher('/robobreizh/whisper_text',String, queue_size=3)

        self.device_ = "cpu"
        if torch.cuda.is_available():
            self.get_logger().info("CUDA is available. Using GPU.")
            self.device_ = "cuda"
        self.whisper_model_ = self.whisper_model_.to(self.device_)


        self.rate = rospy.Rate(1.0/self.inference_period_)
        while not rospy.is_shutdown():
            self.whisper_timer_callback_()
            self.rate.sleep()

    def audio_buffer_callback_(self, audio_msg: Int16MultiArray) -> None:
        self.audio_buffer_.append(audio_msg.data)

    def whisper_timer_callback_(self):
        if len(self.audio_buffer_) == self.audio_buffer_.maxlen:
            audio = (
                np.concatenate(self.audio_buffer_) / 32768.0
            )  # normalization in whisper https://github.com/openai/whisper/blob/0f39c89d9212e4d0c64b915cf7ba3c1f0b59fecc/whisper/audio.py#L49
            audio = torch.from_numpy(audio).float()
            if audio.abs().max() < self.audio_activation_threshold_:
                return
            audio = whisper.pad_or_trim(audio)
            mel = whisper.log_mel_spectrogram(audio).to(self.device_)
            result = whisper.decode(self.whisper_model_, mel, self.whisper_options_)
            print(result.text)
            text_msg = String(data=result.text)
            self.text_pub.publish(text_msg)


def main():
    rospy.init_node('whisper',anonymous=True)
    WhisperInferenceNode("whisper_inference_node")
    rospy.spin()
    rospy.shutdown()


if __name__ == "__main__":
    main()