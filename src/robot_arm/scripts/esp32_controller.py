#!/usr/bin/env python

import rospy
import math

from robot_arm.msg import servo_pos, servo_feedback, servo_command
from std_msgs.msg import UInt8

class ServoMotor():

    def __init__(self, min, max):
        self.min = min - 2
        self.max = max + 2

class Controller():

    def __init__(self):

        # Servos
        self.servos = []
        servo0 = ServoMotor(0, 1002)
        servo1 = ServoMotor(370, 837)
        servo2 = ServoMotor(0, 850)
        self.servos.append(servo0)
        self.servos.append(servo1)
        self.servos.append(servo2)

        # ROS Topics
        self.pub_servos_feedback = None
        self.pub_esp32_cmd = None
        self.pub_esp32_btn = None
        self.init_publishers()
        self.init_listeners()

        rospy.loginfo("Initialized the ESP32 Controller")

    def init_listeners(self):
        rospy.Subscriber("/esp32/servos/feedback", servo_feedback, self.listen_callback_servo_feedback) # Feedback in
        rospy.Subscriber("/esp32_ctrl/cmd", servo_command, self.listen_callback_esp32_ctrl_cmd)         # Axis command in
        rospy.Subscriber("/esp32_ctrl/btn", UInt8, self.listen_callback_esp32_ctrl_btn)                 # Button command in

        rospy.loginfo("Initialized the listeners (ESP32 Controller)")

    def init_publishers(self):
        self.pub_servo_feedback = rospy.Publisher("/servos/feedback", servo_feedback, queue_size=1)     # Feedback out 
        self.pub_esp32_cmd = rospy.Publisher("/robot_arm/cmd", servo_command, queue_size=1)             # Axis command out
        self.pub_esp32_btn = rospy.Publisher("/robot_arm/btn", UInt8, queue_size=1)                     # Button command out

        rospy.loginfo("Initialized the publishers (ESP32 Controller)")

    def control_servo(self, id, pos):
        # if pos >= self.servos[id].min and pos <= self.servos[id].max:
        #     return True
        # return False
        pass

    def listen_callback_servo_feedback(self, data):
        if self.control_servo(data.servo0, 0) and self.control_servo(data.servo1, 1) and self.control_servo(data.servo2, 2):
            self.pub_servo_feedback.publish(data)

    def listen_callback_esp32_ctrl_cmd(self, data):
        self.pub_esp32_cmd.publish(data)

    def listen_callback_esp32_ctrl_btn(self, data):
        self.pub_esp32_btn.publish(data)


if __name__ == "__main__":
    rospy.init_node('esp32_ctrl', anonymous=True)
    node = Controller()
    rospy.spin()

