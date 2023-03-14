import openai
import config

openai.api_key = config.API_KEY("aung")

audio_file = open("audio_files/Television.wav", "rb")
transcript = openai.Audio.transcribe("whisper-1", audio_file)
print(transcript)