#!/usr/bin/env python
import scipy.io.wavfile as wav
import rospy
from std_msgs.msg import String
import numpy as np

class SoundVolume:
    def __init__(self):
        rospy.loginfo("created Soundvolume")
        rospy.init_node('sound_volume', anonymous=True)
        # create the publisher
        self.pub = rospy.Publisher('sound_volume', String, queue_size=10)
        # create the subscriber on audio files
        rospy.Subscriber('sound_recorder', String, self.callback)
        # Create the voice emotion recogniser
        rospy.spin()

    def callback(self, string):
        rospy.loginfo("File: " + string.data)
        self.measure_mean(string.data)

    def measure_mean(self, file):
        sample_rate, data = wav.read(file)
        d = data.astype(np.float)
        rms = np.sqrt((d * d).sum() / len(d))
        self.pub.publish(String(str(rms)))


SoundVolume()
