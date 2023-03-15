#!/user/bin/env python3
import numpy as np
import os
import rospy # ROS Python interface
from std_msgs.msg import Int16MultiArray # ROS message for mics
from std_msgs.msg import Bool

SAMPLE_COUNT = 640
SAMPLING_TIME = 5 # in terms of seconds

class ProcessAudio(object):

    # update the detected words with the messages being published
    def __init__(self):
        rospy.init_node("process_audio")
        self.mic_data = np.zeros((0, 4), 'uint16')
        self.micbuf = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        self.startCheck = False
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            self.topic_base_name + "/sensors/mics", Int16MultiArray, self.audio_cb, tcp_nodelay=True
        )
        self.audio_publisher = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/process_speech", Int16MultiArray, queue_size=0
        )
        self.check_publisher = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/check_speech", Bool, queue_size=0
        )
        self.audio_store = Int16MultiArray()
        self.check_audio = Bool()

        # processed sound
        self.detected_sound = np.zeros((0,1), 'uint16')
        self.start_processing = False   # check whether a new audio sample is to be processed

    def audio_cb(self, msg):
		# reshape into 4 x 500 array
        data = np.asarray(msg.data)
        self.mic_data = np.transpose(data.reshape((4, 500)))
        self.check_audio.data = False
        if not self.micbuf is None:

			# append mic data to store
            self.micbuf = np.concatenate((self.micbuf, self.mic_data))

			# finished recording?
            if self.micbuf.shape[0] >= SAMPLE_COUNT:

				# start updating records
                self.outbuf = self.micbuf[:SAMPLE_COUNT]
                self.micbuf = self.micbuf[SAMPLE_COUNT:]
                self.startCheck = True 	# check if micbuf is full since the used audio will be deleted from mic buf
            
            if self.startCheck is True and not self.outbuf is None:
                # get audio from left ear of MiRo
                detect_sound = np.reshape(self.outbuf[:,[1]], (-1))

                # process audio
                outbuf = np.zeros((int(SAMPLE_COUNT), 0), 'uint16')
                i = np.arange(0, SAMPLE_COUNT, 1)
                j = np.arange(0, SAMPLE_COUNT)
                x = np.interp(i, j, detect_sound[:])
                outbuf = np.concatenate((outbuf, x[:, np.newaxis]), axis=1)
                outbuf = outbuf.astype(int)
                outbuf = np.reshape(outbuf[:, [0]], (-1))
                self.startCheck = False
                self.detected_sound = np.append(self.detected_sound, outbuf)
            
            # renew data
            if len(self.detected_sound) >= 20000 * SAMPLING_TIME:
                self.audio_store.data = self.detected_sound
                self.detected_sound = np.zeros((0,1), 'uint16')
                print(self.start_processing)
                self.check_audio.data = True
            self.audio_publisher.publish(self.audio_store)
            self.check_publisher.publish(self.check_audio)

if __name__ == "__main__":
    main = ProcessAudio()
    while not rospy.core.is_shutdown():
        print(main.check_audio.data)