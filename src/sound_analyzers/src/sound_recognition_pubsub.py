#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_recognition.sound_recogniser_dcase import SoundRecogniser
from sound_analyzers.msg import RecognisedSounds
from sound_analyzers.msg import RecognisedSoundProbability


class SoundRecognition_PubSub:
    def __init__(self):
        """
        This node will receive a wav file and will recognise sounds within that file
        """
        rospy.init_node('sound_recognition_dcase', anonymous=True)
        rospy.set_param('recognisable_sounds', SoundRecogniser.class_list)
        self.sound_recogniser = SoundRecogniser()
        # RecognisedSounds are mappings of a heard sound, to a probability of hearing that sound
        # The messages are used for creating the emotional model
        self.pub = rospy.Publisher('sound_recognised', RecognisedSounds, queue_size=10)
        # create the subscriber on audio files
        rospy.Subscriber('sound_recorder', String, self.callback)
        rospy.spin()
    
    def callback(self, data):
        """
        When a new recording is available, try to recognise its sounds
        """
        self.sound_recognise(data)

    def sound_recognise(self, recording_path):
        """
        Recognise the sounds inside recording_path and publish the sounds.
        @param recording_path: The path where to find the wav file
        """
        threshold = 0.60
        rospy.loginfo("path:" + recording_path.data)
        # Recognize the sounds
        recognised_sounds = self.sound_recogniser.recognise_sound(recording_path.data, threshold)
        # Publish to the emotional reasoner
        self.publish_recognised_sounds(recognised_sounds)

    def publish_recognised_sounds(self, recognised_sounds):
        """
        Convert the recognised sounds to a RecognisedSounds message
        Then publish this message
        @param recognised_sounds: A list of recognised sounds together with the certainty the sound was heard
        """
        sound_probabilities = []
        for elem in recognised_sounds:
            sound_prob = RecognisedSoundProbability()
            sound_prob.sound = elem[0]
            sound_prob.probability = round(elem[1], 4)
            sound_probabilities.append(sound_prob)
        sounds = RecognisedSounds()
        sounds.sounds = sound_probabilities
        self.pub.publish(sounds)


SoundRecognition_PubSub()
