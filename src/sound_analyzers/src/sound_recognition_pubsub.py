#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sound_recognition.sound_recogniser_dcase import SoundRecogniser
from sound_analyzers.msg import RecognisedSounds
from sound_analyzers.msg import RecognisedSoundProbability
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils


class SoundRecognition_PubSub:
    def __init__(self):
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('sound_recognition_dcase', anonymous=True)
        self.sound_recogniser = SoundRecogniser()
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
        threshold = 0.5
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
        for sound in recognised_sounds:
            if sound[0] == 'a Dog':
                kb.add_predicate(pytools_utils.predicate_maker("dog-barking", "dog", "brucedog"))
                kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", "brucedog", is_negative=True))
            # elif sound[0] == 'Speech':
                # kb.add_predicate(pytools_utils.predicate_maker()
                # kb.add_predicate(pytools_utils.predicate_maker("human-silent", "humanbert")))


if __name__ == '__main__':
    try:
        SoundRecognition_PubSub()
    except rospy.ROSInterruptException:
        pass
