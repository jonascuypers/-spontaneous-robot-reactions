#!/usr/bin/env python
import rospy
from emotional_reasoner.msg import Emotion
from sound_analyzers.msg import RecognisedSounds
from std_msgs.msg import String
from emotional_reasoner.srv import EmotionService, EmotionServiceResponse
import math
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils


class EmotionalReasonerPubSub:
    def __init__(self):
        rospy.init_node('emotional_reasoner', anonymous=True)
        kb.initialize(prefix="/rosplan_knowledge_base")
        self.valence = 0
        self.arousal = 0
        self.publish_as_param()
        self.pub = rospy.Publisher('emotional_reasoner', Emotion, queue_size=10)
        self.service = rospy.Service('emotion_service', EmotionService, self.service_respond_emotion)
        rospy.Subscriber('voice_emotion_recognised', RecognisedSounds, self.adapt_model_speech)
        rospy.Subscriber('sound_recognised', RecognisedSounds, self.adapt_model_sounds)
        self.numbers_to_kb_emotion = [[-1, -1, "lowv-lowa"], [1, -1, "highv-lowa"],
                                      [1, 1, "highv-higha"], [-1, 1, "lowv-higha"]]

        self.map_emotion_to_quantity = {
            "Neutral": {"valence": 0, "arousal": 0},
            "Happy": {"valence": 0.5, "arousal": 0.2},
            "Sad": {"valence": -0.5, "arousal": -0.2},
            "Angry": {"valence": -0.5, "arousal": 0.4},
            "Fear": {"valence": -0.30, "arousal": 0.3},
        }
        self.map_sound_to_quantity = {
            "an Alarm bell ringing": {"valence": -0.295, "arousal": 0.387},
            "Speech": {"valence": 0.0, "arousal": 0.0},
            "a Dog": {"valence": -0.023, "arousal": 0.182},
            "a Cat": {"valence": 0.535, "arousal": 0.067},
            "Dishes": {"valence": -0.192, "arousal": -0.285},
            "Frying": {"valence": 0.0, "arousal": 0.0},
            "an Electric_shaver or a toothbrush": {"valence": -0.437, "arousal": 0.178},
            "a Blender": {"valence": -0.668, "arousal": 0.27},
            "Running_water": {"valence": -0.137, "arousal": -0.045},
            "a Vacuum_cleaner": {"valence": 0, "arousal": 0}
        }

    def adapt_model_speech(self, data):
        """
        Speech with an emotion has been recognised, now the model is adapted
        @param data: The incoming message containing the emotions in speech
        """
        data = data.sounds
        # Find the emotion with the highest expected value
        max_prob, max_emotion = 0, ""
        for emotion_prob in data:
            if emotion_prob.probability > max_prob:
                max_prob = emotion_prob.probability
                max_emotion = emotion_prob.sound

        amounts = self.map_emotion_to_quantity[max_emotion]
        print max_emotion
        self.add_vectors(amounts["valence"], amounts["arousal"])

    def adapt_model_sounds(self, data):
        """
        Sounds have been recognised, now the model is adapted
        @param data: The incoming message containing the recognised sounds
        """
        data = data.sounds
        for elem in data:
            sound_emotion = self.map_sound_to_quantity[elem.sound]
            # adapt the model
            self.add_vectors(sound_emotion["valence"], sound_emotion["arousal"])

    def add_vectors(self, valence, arousal):
        """
        Add the new emotion to the already existing emotion
        @param valence: The valence which must be added
        @param arousal: The arousal which must be added
        """
        self.valence += valence
        self.arousal += arousal
        vector_size = math.sqrt(math.pow(self.arousal, 2) + math.pow(self.valence, 2))
        if vector_size > 1:
            self.valence = self.valence/vector_size
            self.arousal = self.arousal/vector_size

    def service_respond_emotion(self, arg):
        """
        This class is also a ROS Service, so respond with the valence and arousal when necessary
        """
        return EmotionServiceResponse(self.valence, self.arousal)

    def publish_as_param(self):
        """
        Publish the valence and arousal to the parameter server (Used as continuous store of data)
        """
        emotion = {"valence": self.valence, "arousal": self.arousal}
        rospy.set_param('emotion', emotion)

    def publish_as_knowledge(self):
        """
        Publish the quadrant to the knowledge base
        """
        sign = lambda x: -1 if x < 0 else 1
        for emotion in self.numbers_to_kb_emotion:
            if sign(self.valence) == emotion[0] and sign(self.arousal) == emotion[1]:
                pred = pytools_utils.predicate_maker("current-emotion", ["person", "emotionquadrant"],
                                                     ["personbert", emotion[2]])
                kb.add_predicate(pred)
            else:
                pred = pytools_utils.predicate_maker("current-emotion", ["person", "emotionquadrant"],
                                                     ["personbert", emotion[2]], True)
                kb.add_predicate(pred)

    def publisher(self):
        """
        The publisher will publish the valence and arousal each 10 seconds to the:
        Knowledge Base
        Parameter Server
        Normal publish
        """
        rate = rospy.Rate(0.1)  # 0.1hz
        while not rospy.is_shutdown():
            emotion = Emotion()
            # update the emotion
            self.valence = 0.9 * self.valence
            self.arousal = 0.9 * self.arousal
            emotion.arousal = self.arousal
            emotion.valence = self.valence
            # Publish for the visualisation
            self.pub.publish(emotion)
            # Publish for using in knowledge base
            self.publish_as_knowledge()
            # Publish on continuous knowledge base
            self.publish_as_param()
            rate.sleep()


EmotionalReasonerPubSub().publisher()
