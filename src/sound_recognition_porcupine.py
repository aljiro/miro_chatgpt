#!/usr/bin/env python3
#
# The following code has been adapted from client_audio.py provided 
# by CONSEQUENTIAL ROBOTICS for detecting audio
#
# The following code uses porcupine to detect for wake words that is
# "Hey Miro", "No Miro" and "Good Boy" after it will be published

import rospy

import os
import numpy as np
import wave, struct
import pvporcupine
from speech_recognition_porcupine.msg import Audio
from process_audio import ProcessAudio

# sample count
SAMPLE_COUNT = 640

class ros_porcupine():

	def loop(self):
		
		# loop
		detected_sound = np.zeros((0, 1), 'uint16')

		while not rospy.core.is_shutdown():

			self.micbuf = self.mic_data.micbuf
			self.outbuf = self.mic_data.outbuf
			self.startCheck = self.mic_data.startCheck

			if self.startCheck is True:
				
				# get audio from left ear of Miro
				detect_sound = np.reshape(self.outbuf[:, [1]], (-1))

				# downsample for playback
				outbuf = np.zeros((int(SAMPLE_COUNT / 1.25), 0), 'uint16')
				i = np.arange(0, SAMPLE_COUNT, 1.25)
				j = np.arange(0, SAMPLE_COUNT)
				x = np.interp(i, j, detect_sound[:])
				outbuf = np.concatenate((outbuf, x[:, np.newaxis]), axis=1)
				outbuf = outbuf.astype(int)
				outbuf = np.reshape(outbuf[:, [0]], (-1))

				# check for any wake words
				keyword_index = self.handle.process(outbuf)

				# if the miro is doing actions for hey miro then it will continue to loop
				if (self.keyword_detected >= 0 and keyword_index == -1):
					self.keyword_detected = self.keyword_detected
				else:
					self.keyword_detected = keyword_index

				# if wake word is "Good Boy"
				if self.keyword_detected == 0:
					self.audio.detected_words = "Good Boy"
				# if wake word is "Hey Miro"
				elif self.keyword_detected == 1:
					self.audio.detected_words = "Hey Miro"
					
				# if wake word is "No Miro"
				elif self.keyword_detected == 2:
					self.audio.detected_words = "No Miro"
				
				# if nothing has been detected before
				elif self.keyword_detected == -1:
					self.audio.detected_words = ""
				
				# publish the detected words
				self.audio_pub.publish(self.audio.detected_words)

				# collect data from the micbuf for making audio file for re-listening later
				detected_sound = np.append(detected_sound, outbuf)
				print(len(detect_sound))
				self.mic_data.startCheck = False # to show micbuf data has been processed

		# save audio file throughout the whole running process
		outfilename = 'tmp/client_audio.wav'	# audio file location
		file = wave.open(outfilename, 'wb')
		file.setframerate(16000)
		file.setsampwidth(2)
		
		print("writing one CENTRE channel to file with sample length " + str(len(detected_sound)))
		file.setnchannels(1)
		for s in detected_sound:
			file.writeframes(struct.pack('<h', s))

		file.close()
	
	def __init__(self):

		# Initialise node and required objects
		rospy.init_node("porcupine_sound_recognition")
		self.mic_data = ProcessAudio()

		# state
		self.micbuf = self.mic_data.micbuf
		self.outbuf = self.mic_data.outbuf
		self.buffer_stuff = 0
		self.playchan = 0
		self.playsamp = 0
		self.startCheck = self.mic_data.startCheck
		self.keyword_detected = -1

		# porcupine access
		self.access_key = "B+WsqPgG7cJtnh7HwOCw4S264Vx0JncHljCKCbp+euikQIW3Ufqhag=="
		new_path = "../pkgs/mdk-210921/catkin_ws/src/speech_recognition_porcupine/src"
		os.chdir(new_path)
		self.handle = pvporcupine.create(access_key=self.access_key, 
									keywords=['hey google'],
									keyword_paths=['processed_data/Good-boy_en_linux_v2_1_0.ppn',
									'processed_data/Hey-Miro_en_linux_v2_1_0.ppn',
									'processed_data/No-Miro_en_linux_v2_1_0.ppn'])

		# publish processed words from audio
		self.topic_base_name = "/"  + os.getenv("MIRO_ROBOT_NAME")
		self.audio_pub = rospy.Publisher(
			self.topic_base_name + "/audio/detected_words", Audio, queue_size=0
		)
		self.audio = Audio()

		# report
		print ("starting to record audio from the right ear of MiRo")

if __name__ == "__main__":
	main = ros_porcupine()
	main.	loop()
