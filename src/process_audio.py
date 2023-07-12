#!/user/bin/env python3
import numpy as np
import os
import rospy # ROS Python interface
from std_msgs.msg import Int16MultiArray, Bool, String # ROS message for mics
import time
import pvporcupine

# testing purposes
import wave, struct

SAMPLE_COUNT = 640
SAMPLING_TIME = 10 # in terms of seconds

class ProcessAudio(object):

    # update the detected words with the messages being published
    def __init__(self):
        
        # init for the processing audio
        rospy.init_node("process_audio")
        self.mic_data = np.zeros((0, 4), 'uint16')      # the raw sound data obtained from the MiRo's message.
        self.micbuf = np.zeros((0, 4), 'uint16')        # the raw sound data that is has been recorded for use. This has been recorded and passes to be process when it has enough sample count.
        self.detected_sound = np.zeros((0,1), 'uint16') # the detected sound throughout.
        self.recorded_sound = np.zeros((0,1), 'uint16') # to store the first few seconds of sound to be init when hearing a response.
        self.to_record = np.zeros((0,1), 'uint16')      # the frame which is recorded for use in chatgpt.
        self.zcr_frame = np.zeros((0,1), 'uint16')      # zero crossing rate frame.
        self.process_whisper_msg = Bool()               # message used to let whisper know when to start processing.
        self.gesture_msg = String()                     # gestures message
        self.stop_record = time.time() - 5              # the time when the robot should stop recording. The robot will stop recording 3 seconds after it hears "Hey MiRo" and the user stops speaking after.
        self.start_check_time = time.time() + 1.5       # the time when the robot itself is speaking.

        # porcupine access
        self.access_key = "B+WsqPgG7cJtnh7HwOCw4S264Vx0JncHljCKCbp+euikQIW3Ufqhag=="
        # new_path = "../pkgs/mdk-210921/catkin_ws/src/speech_recognition_porcupine/src"
        # os.chdir(new_path)
        self.handle = pvporcupine.create(access_key=self.access_key, 
									keywords=['hey google'],
									keyword_paths=['processed_data/Hey-Miro_en_linux_v2_2_0.ppn'])

        # ros subcribers and publishers to be used
        self.topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.subscriber = rospy.Subscriber(
            self.topic_base_name + "/sensors/mics", Int16MultiArray, self.audio_cb, tcp_nodelay=True
        )
        self.whisper_publisher = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/process_whisper", Bool, queue_size=0
        )
        self.subscriber_check_gtts = rospy.Subscriber(
            self.topic_base_name + "/control/stream", Int16MultiArray, self.check_gtts_cb, tcp_nodelay=True
        )
        self.pub_gestures = rospy.Publisher(
            self.topic_base_name + "/gpt_speech/actions", String, queue_size=0
        )
        self.pub_response = rospy.Publisher(self.topic_base_name + "/gpt_speech/prompt_response", String, queue_size=0)

    # callback function that updates the time when it sees a message be published
    def check_gtts_cb(self, msg):
        self.gesture_msg.data = "normal"
        self.pub_gestures.publish(self.gesture_msg)
        self.start_check_time = time.time()

    # zero crossing rate calculation
    def calc_ZCR(self, signal):
        ZCR = 0
        for k in range(1, len(signal)):
            ZCR += 0.5 * abs(np.sign(signal[k]) - np.sign(signal[k - 1]))
        return ZCR
    
    # process audio for wake word and the recording to be sent for speech to text
    def audio_cb(self, msg):
        # start recording only if the miro is not speaking
        if time.time() - self.start_check_time > 1.5:
            # reshape into 4 x 500 array
            data = np.asarray(msg.data)
            self.mic_data = np.transpose(data.reshape((4, 500)))
            self.record = False

            if not self.micbuf is None:
                self.micbuf = np.concatenate((self.micbuf, self.mic_data))
                
                if self.micbuf.shape[0] >= SAMPLE_COUNT:
                    outbuf = self.micbuf[:SAMPLE_COUNT]
                    self.micbuf = self.micbuf[SAMPLE_COUNT:]
                    
                    # get audio from left ear of Miro
                    detect_sound = np.reshape(outbuf[:, [1]], (-1))
                    for i in range(1,3):
                        detect_sound = np.add(detect_sound, np.reshape(outbuf[:,[i]], (-1)))

                    # downsample to sampling rate accepted by picovoice
                    outbuf_dwnsample = np.zeros((int(SAMPLE_COUNT / 1.25), 0), 'uint16')
                    i = np.arange(0, SAMPLE_COUNT, 1.25)
                    j = np.arange(0, SAMPLE_COUNT)
                    x = np.interp(i, j, detect_sound[:])
                    outbuf_dwnsample = np.concatenate((outbuf_dwnsample, x[:, np.newaxis]), axis=1)
                    outbuf_dwnsample = outbuf_dwnsample.astype(int)
                    outbuf_dwnsample = np.reshape(outbuf_dwnsample[:, [0]], (-1))
                    if len(self.recorded_sound) < 20000:
                        self.recorded_sound = np.append(self.recorded_sound, detect_sound)
                    else:
                        self.recorded_sound = np.append(self.recorded_sound[20000:], detect_sound)
                    
                    # check for any wake words
                    keyword_index = self.handle.process(outbuf_dwnsample)

                    self.print = keyword_index
                    # if the wake word is "Hey MiRo" start recording
                    if keyword_index != -1:
                        print("Detected: Hey Miro")
                        self.gesture_msg.data = "notice"
                        self.pub_gestures.publish(self.gesture_msg)
                        rmsg = String()
                        rmsg.data = "Hey!"
                        self.pub_response.publish(rmsg)
                        self.stop_record = time.time()
                    if (time.time() - self.stop_record < SAMPLING_TIME):
                        self.gesture_msg.data = "listening"
                        self.pub_gestures.publish(self.gesture_msg)
        
                        self.record = True
                        self.detected_sound = np.append(self.detected_sound, self.recorded_sound)
                        self.recorded_sound = np.zeros((0,1), 'uint16')
                    else:
                        if len(self.detected_sound) > 0:
                            self.to_record = self.detected_sound
                            print(len(self.to_record))
                            self.detected_sound = np.zeros((0,1), 'uint16')

    # to be looped. the method is used for saving audio files and publishing message to whisper that it is ready to be processed.
    # To-Do: write to Bytes.io
    def record_audio(self):
        outfilename = 'audio_files/testing.wav'
        if len(self.to_record) > 0:
            outfilename = 'audio_files/testing.wav'
            with wave.open(outfilename, 'wb') as file:
                file.setframerate(20000)
                file.setsampwidth(2)
                print("Starting to write/re-write new audio file of sample length " + str(len(self.to_record)))
                file.setnchannels(1)
                for s in self.to_record:
                    try:
                        file.writeframes(struct.pack('<h', s))
                    except struct.error as err:
                        print(err)
                file.close()
            print("Writing Complete!")
            self.to_record = np.zeros((0,1), 'uint16')
        if os.path.exists(outfilename):
            self.gesture_msg.data = "working"
            self.pub_gestures.publish(self.gesture_msg)
            self.process_whisper_msg.data = True
        else:
            self.process_whisper_msg.data = False
        self.whisper_publisher.publish(self.process_whisper_msg)

if __name__ == "__main__":
    main = ProcessAudio()
    while not rospy.core.is_shutdown():
        main.record_audio()