#!/usr/bin/env python3
import rospy, os
import openai, config

from std_msgs.msg import Bool, String

SAMPLE_COUNT = 640

class TextPrompt():

    def __init__(self):
        # openai load api key
        openai.api_key = config.load_api_key()

        # init node
        rospy.init_node("text_prompt")
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            self.topic_base_name + "/gpt_speech/process_whisper", Bool, self.check_cb, tcp_nodelay=True
        )
        self.publisher = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/text_prompt", String, queue_size=0
        )
        self.message = String()
        self.process_whisper = False

    def check_cb(self, msg):
        self.process_whisper = msg.data

    def text_to_speech(self):
        outfilename = 'audio_files/testing.wav'
        while not rospy.core.is_shutdown():
            if os.path.exists(outfilename) and self.process_whisper is True:
                rospy.sleep(0.5)
                audio_file= open(outfilename, "rb")
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
                self.message.data = transcript.text
                print(self.message.data)
                self.publisher.publish(self.message.data)
                os.remove(outfilename)
                rospy.sleep(0.5)

if __name__ == "__main__":
    main = TextPrompt()
    main.text_to_speech()


    # def text_to_speech(self):
    #     audio_file = open("audio_files/Television.wav", "rb")
    #     print(audio_file)
    #     transcript = openai.Audio.transcribe("whisper-1", audio_file)
    #     print(transcript)