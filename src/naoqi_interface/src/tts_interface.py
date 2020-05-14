#!/usr/bin/env python
import actionlib
from tts.msg import SpeechAction, SpeechGoal
import rospy
from std_msgs.msg import String
import math


def calculate_rate(emotion):
    rate = 100
    rate_range = 20
    rate_change = 0
    # One of both positive
    if emotion['valence'] * emotion['arousal'] < 0:
        rate_change = int(emotion['norm'] * rate_range)
        # positive valence negative arousal = chill
        if emotion['arousal'] < 0:
            rate_change *= -1
    return "{}%".format(rate + rate_change)


def calculate_pitch(emotion):
    pitch_range = 20
    pitch_change = 0
    sign = ""
    # Only change if both the same sign
    if emotion['valence'] * emotion['arousal'] > 0:
        pitch_change = int(emotion['norm'] * pitch_range)
        # Both positive => High pitch
        if emotion['valence'] > 0:
            sign = "+"
        else:
            sign = "-"
    return "{}{}%".format(sign, pitch_change)


def calculate_volume(emotion):
    volume_range = 6
    volume_change = 0
    sign = "+"
    # One of both positive
    if emotion['valence'] * emotion['arousal'] < 0:
        volume_change = int(emotion['norm'] * volume_range)
        # positive valence negative arousal = chill
        if emotion['arousal'] < 0:
            sign = "-"
    return "{}{}dB".format(sign, volume_change)


def tts(sentence):
    """
    For the calculation of the changes, view the master thesis
    """
    sentence = sentence.data
    emotion = rospy.get_param('emotion')
    print emotion
    emotion['norm'] = math.sqrt(math.pow(emotion['arousal'], 2) + math.pow(emotion['valence'], 2))
    args = '{"text_type":"ssml"}'
    client = actionlib.SimpleActionClient('tts', SpeechAction)
    client.wait_for_server()
    goal = SpeechGoal()
    sentence = "<speak><prosody volume=\"{}\" rate=\"{}\" pitch=\"{}%\">{}</prosody></speak>".format(
        calculate_volume(emotion), calculate_rate(emotion), calculate_pitch(emotion), sentence
    )
    goal.text = sentence
    goal.metadata = args
    client.send_goal(goal)
    client.wait_for_result()


rospy.init_node('tts', anonymous=True)
rospy.Subscriber('tts', String, tts)
rospy.spin()
