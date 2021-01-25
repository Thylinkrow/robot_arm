#!/usr/bin/env python

import rospy
import math
import json
import os.path

from std_msgs.msg import UInt8, Bool
from robot_arm.msg import servo_pos, servo_feedback, servo_command

class Routine():

    def __init__(self):

        self.status = False
        self.link = "../new_robot_arm/src/robot_arm/routines/"
        self.file_name = None
        self.file = None

        # ROS Topics
        self.pub_cmd = None
        self.pub_btn = None
        self.pub_status = None
        self.init_publishers()
        self.init_listeners()

        rospy.loginfo('Successful init')

    def init_publishers(self):
        self.pub_cmd = rospy.Publisher("/esp32_ctrl/cmd", servo_command, queue_size=1)
        self.pub_btn = rospy.Publisher("/esp32_ctrl/btn", UInt8, queue_size=1)
        self.pub_status = rospy.Publisher("/esp32_ctrl/routine/status", Bool, queue_size=1)

        rospy.loginfo("Initialized the publishers")

    def init_listeners(self):
        rospy.Subscriber("/esp32_ctrl/btn", UInt8, self.listen_callback_btn)

        rospy.loginfo("Initialized the listeners")

    def listen_callback_btn(self, data):
        if not self.status:
            self.status = True
            self.file_name = "routine_"+str(data.data)+".json"
            self.pub_status.publish(self.status)

    def listen_callback_status(self, data):
        self.status = data.data

    def read_file(self):
        f = open(self.link+self.file_name, 'r')	
        self.file = json.loads(f.read())
        f.close()

    def execute_gripper_cmd(self, cmd):
        action = cmd["action"]
        if action["gripper"] == 0:
            # Closing the gripper
            self.pub_btn.publish(101)
        elif action["gripper"] == 1:
            # Opening the gripper
            self.pub_btn.publish(100)
        # Wait
        rospy.sleep(cmd["wait"] / 1000)

    def execute_axis_cmd(self, cmd):
        action = cmd["action"]
        positions = action["positions"]
        # Changing servos angles
        self.send_to_servos(positions["servo1"], positions["servo2"], positions["servo3"], action["time"])
        # Wait
        rospy.sleep(cmd["wait"] / 1000)

    def send_to_servos(self, s1_pos, s2_pos, s3_pos, time):
        # Send to the servos
        cmd = servo_command()
        cmd.servo0.angle = s1_pos
        cmd.servo0.time = time
        cmd.servo1.angle = s2_pos
        cmd.servo1.time = time
        cmd.servo2.angle = s3_pos
        cmd.servo2.time = time
        self.pub_cmd.publish(cmd)

    def execute_routine(self):
        self.read_file()
        rospy.loginfo("Executing {}".format(self.file["name"]))

        for cmd in self.file["commands"]:
            if cmd["type"] == "gripper":
                self.execute_gripper_cmd(cmd)
            elif cmd["type"] == "axis":
                self.execute_axis_cmd(cmd)

        rospy.loginfo("Done")

    def loop(self):
        while not rospy.is_shutdown():
            if self.status:
                self.execute_routine()
                self.status = False
                self.pub_status.publish(self.status)


if __name__ == '__main__':
    rospy.init_node('routine')
    node = Routine()
    node.loop()
    rospy.spin()
