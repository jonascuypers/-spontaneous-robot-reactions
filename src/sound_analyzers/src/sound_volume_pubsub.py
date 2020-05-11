#!/usr/bin/env python
import scipy.io.wavfile as wav
import rospy
from std_msgs.msg import String
import numpy as np


class SoundVolume:
    def __init__(self):
        """
        This class listens to incoming audio files, reads them in, and publishes the RMS
        """
        rospy.loginfo("created Soundvolume")
        rospy.init_node('sound_volume', anonymous=True)
        # create the publisher
        self.pub = rospy.Publisher('sound_volume', String, queue_size=10)
        # create the subscriber on audio files
        rospy.Subscriber('sound_recorder', String, self.measure_mean_volume)
        # Create the voice emotion recogniser
        rospy.spin()

    def measure_mean_volume(self, audio_file):
        """
        Receives a String containing the file location of the wav file and publishes the RMS of the sound
        @param audio_file : The location of the audio file
        """
        audio_file = audio_file.data
        sample_rate, data = wav.read(audio_file)
        d = data.astype(np.float)
        rms = 20 * np.log10(np.sqrt((d * d).sum() / len(d)))
        self.pub.publish(String(str(rms)))


SoundVolume()
