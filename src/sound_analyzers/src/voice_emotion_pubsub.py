#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from voice_emotion_recognition.voice_emotion_recognition import VoiceEmotionRecognition
from sound_analyzers.msg import RecognisedSounds
from sound_analyzers.msg import RecognisedSoundProbability


class VoiceEmotionRecognitionPubSub:
    def __init__(self):
        """
        This node will extract emotions in voice of an audio file
        """
        rospy.init_node('voice_emotion_recognition', anonymous=True)
        self.voice_emotion_recogniser = VoiceEmotionRecognition()
        # create the publisher
        self.pub = rospy.Publisher('voice_emotion_recognised', RecognisedSounds, queue_size=10)
        # create the subscriber on audio files
        rospy.Subscriber('sound_recorder', String, self.emotion_recognise)
        # Create the voice emotion recogniser
        rospy.spin()

    def emotion_recognise(self, string):
        """
        Given a file, check if it contains speech and which emotion the speech has.
        @param string: The String which contains the audio file
        """
        rospy.loginfo("path:" + string.data)
        # Recognize the emotions
        sounds = self.voice_emotion_recogniser.emotions_from_file(string.data)
        # Only publish if speech was recognised
        if sounds:
            voice_emotions = []
            for elem in sounds:
                # Create an object for publishing the emotions
                emotion_prob = RecognisedSoundProbability()
                emotion_prob.sound = elem[0]
                emotion_prob.probability = round(elem[1], 4)
                voice_emotions.append(emotion_prob)
            emotions = RecognisedSounds()
            emotions.sounds = voice_emotions
            # Publish the emotions
            self.pub.publish(emotions)


VoiceEmotionRecognitionPubSub()
