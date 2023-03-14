#!/usr/bin/env python2
import numpy as np
import wave, struct, rospy
# import openai, config

from process_audio import ProcessAudio

class TextPrompt():

    def __init__(self):
        # openai load api key
        # openai.api_key = config.load_api_key()

        # init node
        rospy.init_node("text_prompt")
        self.mic_data = ProcessAudio()

        # state
        self.outbuf = self.mic_data.outbuf
        self.buffer_stuff = 0
        self.startCheck = self.mic_data.startCheck
    
    def text_to_speech(self):
        detected_sound = np.zeroes((0,1), 'uint16')

        # while not rospy.core.is_shutdown():
        while len(detected_sound) < 100000:

            self.outbuf = self.mic_data.outbuf
            self.startCheck = self.mic_data.startCheck

            if self.startCheck is True:
                # get audio from left ear of MiRo
                detect_sound = np.reshape(self.outbuf[:,[1]], (-1))

                # downsample for playback
                detected_sound = np.append(detected_sound, detect_sound)
            
        outfilename = 'audio_files/testing.wav'
        file = wave.open(outfilename, 'wb')
        file.setframerate(20000)
        file.setsampwidth(2)

        print("writing")
        file.setnchannels(1)
        for s in detected_sound:
            file.writeframes(struct.pack('<h', s))
        file.close()

if __name__ == "__main__":
    main = TextPrompt()
    main.text_to_speech()


    # def text_to_speech(self):
    #     audio_file = open("audio_files/Television.wav", "rb")
    #     print(audio_file)
    #     transcript = openai.Audio.transcribe("whisper-1", audio_file)
    #     print(transcript)