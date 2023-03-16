import rospy
import os
import std_msgs.msg

class GesturesNode:

    def __init__(self):
        rospy.init_node("chatgpt_process", anonymous=True)
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        
        # Prompt subscriber
        self.sub_prompt = rospy.Subscriber(topic_base_name + "/text_prompt",
                          std_msgs.msg.String, self.prompt_callback, queue_size=1, tcp_nodelay=True)

    def prompt_callback( self, msg ):
        None

    def loop( self ):
        rate = rospy.Rate(10)

        while not rospy.core.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    
    main = GesturesNode()
    main.loop()
