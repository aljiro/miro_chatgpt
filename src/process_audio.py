#!/user/bin/env python3
import numpy as np
import os
import rospy # ROS Python interface
from std_msgs.msg import Int16MultiArray # ROS message for mics
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

# testing purposes
import wave, struct

SAMPLE_COUNT = 640
SAMPLING_TIME = 29 # in terms of seconds

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
        # self.audio_publisher = rospy.Publisher(
        #     self.topic_base_name + "/gpt_speech/process_speech", Int16MultiArray, queue_size=0
        # )
        # self.check_publisher = rospy.Publisher(
        #     self.topic_base_name + "/gpt_speech/check_speech", Bool, queue_size=0
        # )
        self.audio_store = Int16MultiArray()
        self.check_audio = Bool()

        # processed sound
        self.detected_sound = np.zeros((0,1), 'uint16')
        self.zcr_frame = np.zeros((0,1), 'uint16')
        self.to_transcribe = np.zeros((0,1), 'uint16')
        self.start_processing = False   # check whether a new audio sample is to be processed
        self.record = False
        self.stop_record = 0
        self.processing = False

    def calc_ZCR(self, signal):
        ZCR = 0
        for k in range(1, len(signal)):
            ZCR += 0.5 * abs(np.sign(signal[k]) - np.sign(signal[k - 1]))
        return ZCR
    
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
                self.zcr_frame = np.append(self.zcr_frame, outbuf)

            if len(self.zcr_frame) >= 10000:
                zcr_value = self.calc_ZCR(self.zcr_frame)
                self.zcr_frame = np.zeros((0,1), 'uint16')
                if zcr_value >= 100 and zcr_value <= 1000:
                    self.record = True
                    self.stop_record = 0
                    print("RECORDING NOW")
                else:
                    self.stop_record += 1
            
            # reset if false
            if (self.record == False) and (len(self.detected_sound) > 10000):
                self.detected_sound = self.detected_sound[len(self.detected_sound) - 10000:len(self.detected_sound)]

            # renew data
            if ((len(self.detected_sound) >= (20000 * SAMPLING_TIME)) or (self.stop_record >= 4)) and (self.record == True):
                if self.processing == False:
                    self.to_transcribe = self.detected_sound
                    self.processing = True
                self.check_audio.data = True

                # testing
                self.record = False
                self.detected_sound = np.zeros((0,1), 'uint16')

            # self.check_publisher.publish(self.check_audio)
            
    def record_audio(self):
        outfilename = 'audio_files/testing.wav'
        if self.processing is True and (not os.path.exists(outfilename)):
        # if self.processing is True:
            # self.audio_store.data = self.to_transcribe
            # self.audio_publisher.publish(self.audio_store)
            # print("SENDING")
            
            outfilename = 'audio_files/testing.wav'
            file = wave.open(outfilename, 'wb')
            file.setframerate(20000)
            file.setsampwidth(2)
            print("Starting to write/re-write new audio file of sample length " + str(len(self.to_transcribe)))
            file.setnchannels(1)
            for s in self.to_transcribe:
                file.writeframes(struct.pack('<h', s))
            file.close()
            print("DONE")
            self.processing = False

if __name__ == "__main__":
    main = ProcessAudio()
    i = 0
    while not rospy.core.is_shutdown():
        main.record_audio()