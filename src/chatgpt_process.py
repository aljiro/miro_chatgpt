import openai
import rospy
import config
import os

openai.api_key = config.load_api_key()

class ChatGPTNode:
    def __init__(self):
        rospy.init_node("chatgpt_process", anonymous=True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

        self.model = "gpt-3.5-turbo"
        self.system_message = {"role": "system", "content": "You are a helpful assistant."}

    def prompt_callback( self, msg ):
        chat = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": "What is the best beer?"}
                ]
        )

        reply = chat['choices'][0]['message']['content']
        # Publish to the corresponding topic

    def run(self):
        None


config.load_api_key()


