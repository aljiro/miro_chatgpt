import rospy
import time
import numpy as np
from std_msgs.msg import Float32MultiArray, UInt32MultiArray, String
from sensor_msgs.msg import JointState

class Gestures:

    def __init__(self):
        rospy.init_node("testing")
        self.topic_base_name = "/miro"

        self.subscriber = rospy.Subscriber(
                self.topic_base_name + "/gpt_speech/actions", String, self.action_cb, tcp_nodelay=True
        )
        # movement for either tilt, lift, yaw or pitch
        self.pub_kinematic = rospy.Publisher(
            self.topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )

        # movement for the tail, eye lid, ears
        self.pub_cosmetic = rospy.Publisher(
            self.topic_base_name + "/control/cosmetic_joints", Float32MultiArray, queue_size=0
        )

        # color of lights on MiRo (r,g,b)
        self.pub_illumination = rospy.Publisher(
            self.topic_base_name + "/control/illum", UInt32MultiArray, queue_size=0
        )
        self.current_time = time.time()
        self.msg = "normal"
        self.cos_cmd = Float32MultiArray()
        self.cos_cmd.data = [0,0,0,0,0,0]
        self.joint_cmd = JointState()
        self.joint_cmd.position = [0,0,0,0]
        self.color_change = UInt32MultiArray()
    
    def set_move_cosmetic(self, tail_pitch = 0, tail_yaw = 0, left_eye = 0, right_eye = 0, left_ear = 0, right_ear = 0):
        self.cos_cmd.data = [tail_pitch,tail_yaw,left_eye,right_eye,left_ear,right_ear]

    def set_move_kinematic(self, tilt = 0, lift = 0, yaw = 0, pitch = 0):
        self.joint_cmd.position = [tilt, lift, yaw, pitch]
    
    # set color
    def get_illumination(self, red = 0, green = 0, blue = 0):
        # changing the rgb format into android format to be published in MiRo message
        color_detail = (int(red), int(green), int(blue))
        color = '0xFF%02x%02x%02x'%color_detail
        color = int(color, 16)
        return color

    def blink(self):
        if self.current_time < time.time():
            self.current_time = time.time() + 0.03
            blink_pos = np.abs(np.sin(self.current_time))
            self.set_move_cosmetic(tail_pitch=blink_pos, left_eye=blink_pos, right_eye=blink_pos)

    def look_up(self):
        self.set_move_kinematic(lift=0.399, pitch=-0.21)
    
    def look_normal(self):
        self.set_move_kinematic(lift=0.59)

    def loading(self):
        current_time = time.time()
        color_condition = int(current_time % 2)
        if color_condition == 0:
            color = self.get_illumination()
        elif color_condition == 1:
            color = self.get_illumination(red=150)
        # six seperate leds in the miro
        self.color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]

    def done(self):
        color = self.get_illumination(green = 150)
        self.color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]

    def no_process(self):
        color = self.get_illumination()
        self.color_change.data = [
            color,
            color,
            color,
            color,
            color,
            color
        ]
    
    def listening(self):
        self.set_move_kinematic(lift=0.399, pitch=-0.21)
        self.done()
    
    def loop(self):
        while not rospy.core.is_shutdown():
            print(self.msg)
            end_time = time.time() + 1
            if self.msg == "listening":
                while time.time() < end_time:
                    self.set_move_kinematic(lift=0.399, pitch=-0.21)
                    self.done()
            elif self.msg == "notice":
                while time.time() < end_time:
                    self.set_move_kinematic(lift=0.399, pitch=-0.21)
            elif self.msg == "working":
                while time.time() < end_time:
                    self.set_move_kinematic(lift=0.59)
                    self.loading()
            elif self.msg =="normal":
                while time.time() < end_time:
                    self.set_move_kinematic(lift=0.59)
                    self.no_process()
            self.pub_cosmetic.publish(self.cos_cmd)
            self.pub_kinematic.publish(self.joint_cmd)
            self.pub_illumination.publish(self.color_change)
            rospy.sleep(0.3)
        
    def action_cb(self, msg):
        self.msg = msg.data

testing = Gestures()
testing.loop()