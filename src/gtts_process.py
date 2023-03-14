from gtts import gTTS
import rospy
import os
import std_msgs.msg

class gTTSNode:

    def __init__(self):
        rospy.init_node("gTTS_process", anonymous=True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Prompt subscriber
        self.sub_prompt = rospy.Subscriber(topic_base_name + "/prompt_response",
                          std_msgs.msg.String, self.response_callback, queue_size=1, tcp_nodelay=True)

    def response_callback( self, msg ):
        print("Processing response: ", msg)
        res = gTTS(text = msg.data, lang = 'en', slow = False )
        res.save("response.mp3")

    
    def loop(self):
        rate = rospy.Rate(1)

        while not rospy.core.is_shutdown():
            rate.sleep()

if __name__ == "__main__":    
    main = gTTSNode()
    main.loop()