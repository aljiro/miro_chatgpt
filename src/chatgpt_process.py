import openai
import rospy
import config
import os
import std_msgs.msg

openai.api_key = config.load_api_key()

class ChatGPTNode:
    def __init__(self):
        rospy.init_node("chatgpt_process", anonymous=True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        self.model = "gpt-3.5-turbo"
        self.system_message = {"role": "system", "content": "You are a friendly robot assistant called MiRo. You are assisting the University of Sheffield booth at the Living Machines 2023 conference."}

        # Prompt subscriber
        self.sub_prompt = rospy.Subscriber(topic_base_name + "/gpt_speech/text_prompt",
                          std_msgs.msg.String, self.prompt_callback, queue_size=1, tcp_nodelay=True)
        # Response publisher
        self.pub_response = rospy.Publisher(topic_base_name + "/gpt_speech/prompt_response", std_msgs.msg.String, queue_size=0)
                                

    def prompt_callback( self, msg ):
        print("Processing prompt: ", msg)
        chat = openai.ChatCompletion.create(
            model=self.model,
            messages=[
                    self.system_message,
                    {"role": "user", "content": msg.data}
                ]
        )# TO_DO: Concatenate all the previous messages

        reply = chat['choices'][0]['message']['content']
        rmsg = std_msgs.msg.String()
        rmsg.data = reply

        print(rmsg)
        self.pub_response.publish(rmsg)


    def loop(self):
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    
    main = ChatGPTNode()
    main.loop()
