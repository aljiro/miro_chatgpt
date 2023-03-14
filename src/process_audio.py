#!/user/bin/env python3
import numpy as np
import os
import rospy # ROS Python interface
from std_msgs.msg import Int16MultiArray # ROS message for mics

SAMPLE_COUNT = 640

class ProcessAudio(object):

    # update the detected words with the messages being published
    def __init__(self):
        self.mic_data = np.zeros((0, 4), 'uint16')
        self.micbuf = np.zeros((0, 4), 'uint16')
        self.outbuf = None
        self.startCheck = False
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            self.topic_base_name + "/sensors/mics", Int16MultiArray, self.audio_cb, tcp_nodelay=True
        )

    def audio_cb(self, msg):
		# reshape into 4 x 500 array
        data = np.asarray(msg.data)
        self.mic_data = np.transpose(data.reshape((4, 500)))

        if not self.micbuf is None:

			# append mic data to store
            self.micbuf = np.concatenate((self.micbuf, self.mic_data))

			# finished recording?
            if self.micbuf.shape[0] >= SAMPLE_COUNT:

				# start updating records
                self.outbuf = self.micbuf[:SAMPLE_COUNT]
                self.micbuf = self.micbuf[SAMPLE_COUNT:]
                self.startCheck = True 	# check if micbuf is full since the used audio will be deleted from mic buf