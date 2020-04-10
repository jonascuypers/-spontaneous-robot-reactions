#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from voice_emotion_recognition.voice_emotion_recognition import VoiceEmotionRecognition
from sound_analyzers.msg import RecognisedSounds
from sound_analyzers.msg import RecognisedSoundProbability


class VoiceEmotionRecognitionPubSub:
    def __init__(self):
        rospy.init_node('voice_emotion_recognition', anonymous=True)
        self.voice_emotion_recogniser = VoiceEmotionRecognition()
        # create the publisher
        self.pub = rospy.Publisher('voice_emotion_recognised', RecognisedSounds, queue_size=10)
        # create the subscriber on audio files
        rospy.Subscriber('sound_recorder', String, self.callback)
        # Create the voice emotion recogniser
        rospy.spin()

    def callback(self, data):
        self.emotion_recognise(data)

    def emotion_recognise(self, string):
        rospy.loginfo("path:" + string.data)
        # Recognize the sounds
        sounds = self.voice_emotion_recogniser.emotions_from_file(string.data)
        if sounds:
            voice_emotions = []
            for elem in sounds:
                emotion_prob = RecognisedSoundProbability()
                emotion_prob.sound = elem[0]
                emotion_prob.probability = round(elem[1], 4)
                voice_emotions.append(emotion_prob)
            emotions = RecognisedSounds()
            emotions.sounds = voice_emotions
            self.pub.publish(emotions)


if __name__ == '__main__':
    try:
        VoiceEmotionRecognitionPubSub()
    except rospy.ROSInterruptException:
        pass
