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
        self.pub = rospy.Publisher('emotional_reasoner', Emotion, queue_size=10)
        self.service = rospy.Service('emotion_service', EmotionService, self.service_respond_emotion)
        rospy.Subscriber('voice_emotion_recognised', RecognisedSounds, self.adapt_model_speech)
        rospy.Subscriber('sound_recognised', RecognisedSounds, self.adapt_model_sounds)
        self.numbers_to_kb_emotion = [[-1, -1, "lowv-lowa"], [1, -1, "highv-lowa"],
                                      [1, 1, "highv-higha"], [-1, 1, "lowv-higha"]]

        self.map_emotion_to_quantity = {
            "Neutral": {"valence": 0, "arousal": 0},
            "Happy": {"valence": 0.7, "arousal": 0.2},
            "Sad": {"valence": -0.7, "arousal": -0.2},
            "Angry": {"valence": -0.25, "arousal": 0.25},
            "Fear": {"valence": -0.25, "arousal": -0.25},
        }
        self.map_sound_to_quantity = {
            "an Alarm bell ringing": {"valence": 0.5, "arousal": 0.3},
            "Speech": {"valence": 0.0, "arousal": 0.0},
            "a Dog": {"valence": -0.5, "arousal": 0.4},
            "a Cat": {"valence": -0.5, "arousal": -0.4},
            "Dishes": {"valence": -0.2, "arousal": -0.3},
            "Frying": {"valence": 0.5, "arousal": 0.4},
            "an Electric_shaver or a toothbrush": {"valence": 0.2, "arousal": 0.},
            "a Blender": {"valence": -0.3, "arousal": 0.3},
            "Running_water": {"valence": 0, "arousal": 0},
            "a Vacuum_cleaner": {"valence": 0, "arousal": 0}
        }

    def adapt_model_speech(self, data):
        data = data.sounds
        # find max emotion
        max_prob, max_emotion = 0, ""
        for emotion_prob in data:
            if emotion_prob.probability > max_prob:
                max_prob = emotion_prob.probability
                max_emotion = emotion_prob.sound

        amounts = self.map_emotion_to_quantity[max_emotion]
        print max_emotion
        self.add_vectors(amounts["valence"], amounts["arousal"])
        self.pub.publish(Emotion(self.valence, self.arousal))

    def adapt_model_sounds(self, data):
        data = data.sounds
        for elem in data:
            sound_emotion = self.map_sound_to_quantity[elem.sound]
            self.add_vectors(sound_emotion["valence"], sound_emotion["arousal"])
        rospy.Rate(0.5).sleep()
        self.pub.publish(Emotion(self.valence, self.arousal))

    def add_vectors(self, valence, arousal):
        self.valence += valence
        self.arousal += arousal
        vector_size = math.sqrt(math.pow(self.arousal, 2) + math.pow(self.valence, 2))
        if vector_size > 1:
            self.valence = self.valence/vector_size
            self.arousal = self.arousal/vector_size

    def service_respond_emotion(self, arg):
        return EmotionServiceResponse(self.valence, self.arousal)

    def publisher(self):
        rate = rospy.Rate(0.1)  # 0.1hz
        while not rospy.is_shutdown():
            emotion = Emotion()
            self.valence = self.valence - 0.1*self.valence
            self.arousal = self.arousal - 0.1 * self.arousal
            emotion.arousal = self.arousal
            emotion.valence = self.valence
            self.pub.publish(emotion)
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
            rate.sleep()


EmotionalReasonerPubSub().publisher()
