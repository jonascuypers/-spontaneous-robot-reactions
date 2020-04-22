#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_recognition.sound_recogniser_dcase import SoundRecogniser
from sound_analyzers.msg import RecognisedSounds
from sound_analyzers.msg import RecognisedSoundProbability
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils
import numpy as np


class SoundRecognition_PubSub:
    def __init__(self):
        rospy.init_node('sound_recognition_dcase', anonymous=True)
        rospy.set_param('recognisable_sounds', SoundRecogniser.class_list)
        self.sound_recogniser = SoundRecogniser()

        # We keep a matrix to know when to set an event in the knowledge base
        # If in memory_length times recognizing at least minimum_for_recognized are a certain sound
        # This sound will be published in the knowledge base
        # This makes up for instabilities in recognition
        nr_of_recognizable_sounds = len(self.sound_recogniser.class_list)
        memory_length = 7
        self.minimum_for_recognized = 3
        self.recognised_sounds_matrix = np.zeros((nr_of_recognizable_sounds, memory_length))
        self.i = 1
        # create the publisher
        # VoiceEmotions are mappings of a heard sound, to a probability of hearing that sound
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
        The sounds are published to the knowledge base and to the emotional reasoner.
        """
        threshold = 0.6
        rospy.loginfo("path:" + recording_path.data)
        # Recognize the sounds
        recognised_sounds = self.sound_recogniser.recognise_sound(recording_path.data, threshold)
        # Publish to the emotional reasoner
        self.publish_recognised_sounds(recognised_sounds)

    def publish_recognised_sounds(self, recognised_sounds):
        # Convert the sounds with the recognition probability to VoiceEmotions
        sound_probabilities = []
        for elem in recognised_sounds:
            sound_prob = RecognisedSoundProbability()
            sound_prob.sound = elem[0]
            sound_prob.probability = round(elem[1], 4)
            sound_probabilities.append(sound_prob)
        sounds = RecognisedSounds()
        sounds.sounds = sound_probabilities
        self.pub.publish(sounds)


if __name__ == '__main__':
    try:
        SoundRecognition_PubSub()
    except rospy.ROSInterruptException:
        pass
