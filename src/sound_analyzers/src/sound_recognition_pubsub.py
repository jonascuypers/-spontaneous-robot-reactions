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
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('sound_recognition_dcase', anonymous=True)
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
        # Create the sound recogniser
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
        self.publish_voice_emotions(recognised_sounds)
        # Publish to the knowledge base
        self.publish_to_knowledge_base(recognised_sounds)

    def publish_voice_emotions(self, recognised_sounds):
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

    def publish_to_knowledge_base(self, recognised_sounds):
        # Shift the rows one column to the left
        self.recognised_sounds_matrix = np.roll(self.recognised_sounds_matrix, -1)
        recognised_sounds_set = {sound_w_probability[0] for sound_w_probability in recognised_sounds}
        current_recognitions = [1 if sound in recognised_sounds_set else 0 for sound in self.sound_recogniser.class_list]
        rospy.loginfo(" ".join(recognised_sounds_set))
        # replace the last column by the new observations
        self.recognised_sounds_matrix[:, -1] = current_recognitions
        counts_of_recognitions = np.count_nonzero(self.recognised_sounds_matrix, axis=1)
        for count, sound in zip(counts_of_recognitions, self.sound_recogniser.class_list):
            if count >= self.minimum_for_recognized:
                self.publish_single_sound(sound, True)
            else:
                self.publish_single_sound(sound, False)
    def publish_single_sound(self, sound, sound_recognised):
        """
        Publish a single heard sound to the knowledge base

        """
        # Maybe only when not dispatching?
        if sound == "a Dog":
            kb.add_predicate(pytools_utils.predicate_maker("dog-barking", "dog", "brucedog",
                                                           is_negative=(not sound_recognised)))
            kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", "brucedog",
                                                           is_negative=sound_recognised))
        elif sound == "Speech":
            kb.add_predicate(pytools_utils.predicate_maker("human-talking", "Person", "personbert",
                                                           is_negative=(not sound_recognised)))
            kb.add_predicate(pytools_utils.predicate_maker("human-silent", "Person", "personbert",
                                                           is_negative=sound_recognised))
if __name__ == '__main__':
    try:
        SoundRecognition_PubSub()
    except rospy.ROSInterruptException:
        pass
