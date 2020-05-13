#!/usr/bin/env python
import actionlib
from tts.msg import SpeechAction, SpeechGoal
import rospy
from std_msgs.msg import String

def tts(sentence):
    sentence = sentence.data
    args = '{"text_type":"ssml"}'
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    sentence = "<speak><prosody rate=\"120%\" pitch=\"+25%\">" + sentence + "</prosody></speak>"
    goal.text = sentence
    goal.metadata = args
    client.send_goal(goal)
    client.wait_for_result()


rospy.init_node('tts', anonymous=True)
rospy.Subscriber('tts', String, tts)
rospy.spin()
