#!/usr/bin/env python3
import wave, struct, rospy
import os

from std_msgs.msg import Int16MultiArray # ROS message for mics
from std_msgs.msg import Bool

SAMPLE_COUNT = 640

class SaveAudio():

    def __init__(self):
        # init node
        rospy.init_node("save_audio")
        self.audio_sample = None
        self.check_audio = None
        self.check_data = Bool()
        self.check_data.data = False
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.pub_whisper = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/process_whisper", Bool, queue_size=0
        )
        self.audio_subscriber = rospy.Subscriber(
            self.topic_base_name + "/gpt_speech/process_speech", Int16MultiArray, self.audio_cb, tcp_nodelay=True
        )
        self.check_subscriber = rospy.Subscriber(
            self.topic_base_name + "/gpt_speech/check_speech", Bool, self.check_cb, tcp_nodelay=True
        )
    
    def audio_cb(self, msg):
        self.audio_sample = msg.data

    def check_cb(self, msg):
        self.check_audio = msg.data

    def save_audio_file(self):
        outfilename = 'audio_files/testing.wav'
        while not rospy.core.is_shutdown():
            if self.check_audio is True:
                # start saving file
                file = wave.open(outfilename, 'wb')
                file.setframerate(20000)
                file.setsampwidth(2)
                print("Starting to write/re-write new audio file")
                file.setnchannels(1)
                for s in self.audio_sample:
                    file.writeframes(struct.pack('<h', s))
                file.close()
                print("Done")
                # publish start processing whisper
                self.check_data.data = True
            
            if not os.path.exists(outfilename):
                self.check_data.data = False
            self.pub_whisper.publish(self.check_data)

if __name__ == "__main__":
    main = SaveAudio()
    main.save_audio_file()