#!/usr/bin/env python
import rospy
import rosplan_pytools.controller.knowledge_base as kb
import numpy as np
from sound_analyzers.msg import RecognisedSounds
import rosplan_pytools.common.utils as pytools_utils


class SoundRecognitionKnowledgeDeriving:
    def __init__(self):
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('sound_recognition_dcase', anonymous=True)
        # We keep a matrix to know when to set an event in the knowledge base
        # If in memory_length times recognizing at least minimum_for_recognized are a certain sound
        # This sound will be published in the knowledge base
        # This makes up for instabilities in recognition
        # Sleep to make sure parameter is already pushed
        rospy.sleep(5.)
        self.recognisable_sounds = rospy.get_param('recognisable_sounds')
        self.recognisable_sounds.append("kitchen_noise")
        nr_of_recognizable_sounds = len(self.recognisable_sounds)
        memory_length = 7
        self.minimum_for_recognized = 3
        self.recognised_sounds_matrix = np.zeros((nr_of_recognizable_sounds, memory_length))
        self.i = 1
        self.sub = rospy.Subscriber('sound_recognised', RecognisedSounds, self.update_model)
        rospy.spin()

    def update_model(self, recognised_sounds):
        # Shift the rows one column to the left
        self.recognised_sounds_matrix = np.roll(self.recognised_sounds_matrix, -1)
        recognised_sounds = recognised_sounds.sounds
        recognised_sounds_set = {sound_w_probability.sound for sound_w_probability in recognised_sounds}
        if 'a Blender' in recognised_sounds_set or 'Dishes' in recognised_sounds_set \
                or 'Frying' in recognised_sounds_set or 'Running_water' in recognised_sounds_set:
            recognised_sounds_set.add('kitchen_noise')
        current_recognitions = [1 if sound in recognised_sounds_set else 0 for sound in self.recognisable_sounds]
        rospy.loginfo(" ".join(recognised_sounds_set))
        # replace the last column by the new observations
        self.recognised_sounds_matrix[:, -1] = current_recognitions
        counts_of_recognitions = np.count_nonzero(self.recognised_sounds_matrix, axis=1)
        for count, sound in zip(counts_of_recognitions, self.recognisable_sounds):
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
                                                           is_negative=not sound_recognised))
            kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", "brucedog",
                                                           is_negative=sound_recognised))
        elif sound == "Speech":
            kb.add_predicate(pytools_utils.predicate_maker("human-talking", "Person", "personbert",
                                                           is_negative=not sound_recognised))
            kb.add_predicate(pytools_utils.predicate_maker("human-silent", "Person", "personbert",
                                                           is_negative=sound_recognised))
        elif sound == "kitchen_noise":
            kb.add_predicate(pytools_utils.predicate_maker("person-cooking", "Person", "personbert",
                                                           is_negative=not sound_recognised))

SoundRecognitionKnowledgeDeriving()