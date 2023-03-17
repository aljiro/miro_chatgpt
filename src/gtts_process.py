from gtts import gTTS
import rospy
import os
import std_msgs.msg
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray, Int16MultiArray
from io import BytesIO
from pydub import AudioSegment
import numpy as np

class gTTSNode:

    def __init__(self):
        rospy.init_node("gTTS_process", anonymous=True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Prompt subscriber
        self.sub_prompt = rospy.Subscriber(topic_base_name + "/gpt_speech/prompt_response",
                          std_msgs.msg.String, self.response_callback, queue_size=1, tcp_nodelay=True)
        # Control
        self.pub_stream = rospy.Publisher(topic_base_name + "/control/stream", Int16MultiArray, queue_size=0)

        self.data = []

    def response_callback( self, msg ):
        print("Processing response: ", msg)
        mp3_fp = BytesIO()
        res = gTTS(text = msg.data, lang = 'en', slow = False )
        res.save("response.mp3")
        # res.write_to_fp(mp3_fp)

        seg=AudioSegment.from_mp3("response.mp3")
        seg=seg.set_frame_rate(8000)
        seg=seg.set_channels(1)
        wavIO=BytesIO()
        seg.export("response.wav", format="wav")
        # Decoding wav
        # print("Res: ", wavIO.getvalue())
        with open("response.wav", 'rb') as f:
            dat = f.read()
        self.data_r = 0

        # convert to numpy array
        dat = np.fromstring(dat, dtype='int16').astype(np.int32)

        # normalise wav
        dat = dat.astype(float)
        sc = 32767.0 / np.max(np.abs(dat))
        dat *= sc
        dat = dat.astype(np.int16).tolist()
        
        self.data = dat
        self.d = 0

        # TO-DO: DEcode directly to the stream

    
    def loop(self):
        rate = rospy.Rate(10)
        self.d = 0
        while not rospy.core.is_shutdown():
            outfilename = 'audio_files/testing.wav'
            if self.d < len(self.data):
                msg = Int16MultiArray(data = self.data[self.d:self.d + 1000])
                self.d += 1000
                self.pub_stream.publish(msg)
                if os.path.exists(outfilename):
                    rospy.sleep(2)
                    os.remove(outfilename)
            rate.sleep()

if __name__ == "__main__":    
    main = gTTSNode()
    main.loop()