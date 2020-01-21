import json
import rospy
from std_msgs.msg import String
from Helper import Helper
import os

class DubinsTalker:

    def __init__(self, frames):
        self.frames = frames

    def talk(self):
        publisher = rospy.Publisher('STalkerSteer', String, queue_size=10)
        print("Start talking from DubinsTalker")
        rospy.init_node('talker', anonymous=True)
        print("node initialized")
        rate = rospy.Rate(50) # 10hz
        print("rate set")
        while not rospy.is_shutdown():
            for frame in self.frames:
                # rospy.loginfo(frame)
                publisher.publish(json.dumps(frame))
                rate.sleep()
