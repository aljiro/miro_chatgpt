import openai
import config

openai.api_key = config.API_KEY

chat = openai.ChatCompletion.create(
  model="gpt-3.5-turbo",
  messages=[
        {"role": "system", "content": "You are a helpful assistant."},
        {"role": "user", "content": "What is the best beer?"}
    ]
)

reply = chat['choices'][0]['message']['content']

print(reply)


