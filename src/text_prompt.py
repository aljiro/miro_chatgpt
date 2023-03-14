#!/usr/bin/env python3
import numpy as np
import wave, struct, rospy
import openai, config

from process_audio import ProcessAudio

SAMPLE_COUNT = 640

class TextPrompt():

    def __init__(self):
        # openai load api key
        openai.api_key = config.load_api_key()

        # init node
        rospy.init_node("text_prompt")
        self.mic_data = ProcessAudio()

        # state
        self.outbuf = self.mic_data.outbuf
        self.buffer_stuff = 0
        self.startCheck = self.mic_data.startCheck
    
    def text_to_speech(self):
        detected_sound = np.zeros((0,1), 'uint16')

        # while not rospy.core.is_shutdown():
        while len(detected_sound) < 60000:

            self.outbuf = self.mic_data.outbuf
            self.startCheck = self.mic_data.startCheck

            if self.startCheck is True:
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
                self.mic_data.startCheck = False
                detected_sound = np.append(detected_sound, outbuf)

        outfilename = 'audio_files/testing.wav'
        file = wave.open(outfilename, 'wb')
        file.setframerate(20000)
        file.setsampwidth(2)

        print("writing")
        file.setnchannels(1)
        for s in detected_sound:
            file.writeframes(struct.pack('<h', s))
        file.close()
        audio_file= open(outfilename, "rb")
        transcript = openai.Audio.transcribe("whisper-1", audio_file)
        print(transcript)
if __name__ == "__main__":
    main = TextPrompt()
    main.text_to_speech()


    # def text_to_speech(self):
    #     audio_file = open("audio_files/Television.wav", "rb")
    #     print(audio_file)
    #     transcript = openai.Audio.transcribe("whisper-1", audio_file)
    #     print(transcript)