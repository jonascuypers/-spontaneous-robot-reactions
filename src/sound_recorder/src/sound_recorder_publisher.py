#!/usr/bin/env python
from sound_interface.streamgenerators.RobotAudioStream import RobotAudioStream
from sound_interface.streamhandlers.StreamRecorder import StreamRecorder
from naoqi import ALProxy
import rospy

from std_msgs.msg import String
from sound_interface.streamgenerators.MircrophoneStream import MicrophoneStream
import qi
import time

# from audio_analysis.wav_to_sound import Soundrecognizer
robot_ip = "10.10.131.126"
port = 9559


class SoundRecorderPublisher:
    def __init__(self):
        rospy.init_node('sound_recorder', anonymous=True)
        # create the publisher
        self.pub = rospy.Publisher('sound_recorder', String, queue_size=10)
        self.path = "/home/jonas/recorded_sounds/"
        # self.path = rospy.get_param("recording_path")
        # Create the sound recogniser
        rospy.loginfo("Created recorder")

    def startup_procedure(self):
        tts = ALProxy("ALTextToSpeech", robot_ip, port)
        tts.setLanguage("English")
        tts.post.say("Hello, recognise sounds!")
        motion = ALProxy("ALMotion", robot_ip, port)
        motion.moveInit()
        # motion.setStiffnesses("Body", 1.0)
        motion.post.moveTo(1, 0, -0.5)
        # motion.post.setStiffnesses("Body", 0)
        tts.say("I'm walking")


    def rb(self):
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + "tcp://10.10.131.126:9559"])
        app.start()
        session = app.session
        posture_service = session.service("ALRobotPosture")
        posture_service.goToPosture("StandInit", 1.0)

    def start_recording_bot(self):
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + "tcp://10.10.131.126:9559"])
        app.start()
        session = app.session
        peppertalks = PepperTalks(robot_ip, port)
        al_audio_device = session.service("ALAudioDevice")
        with RobotAudioStream(session, al_audio_device) as stream:
            i = 0
            recorder = StreamRecorder(stream, self.path + "output_mic" + str(i) + ".wav")
            while True:
                i += 1
                recorder.start()
                time.sleep(10)
                recorder.stop()
                self.pub.publish(String(recorder.output_file_path))
                recorder.output_file_path = self.path + "output_mic" + str(i) + ".wav"


    def start_recording_pc(self):
        # peppertalks = PepperTalks(robot_ip, port)
        with MicrophoneStream(7,
                              1024) as stream:  # input device index can be determined with the list_input_devices method.
            i = 0
            recorder = StreamRecorder(stream, self.path + "output_mic" + str(i) + ".wav")
            sleep_duration = rospy.Rate(0.1)
            while not rospy.is_shutdown():
                i += 1
                recorder.start()
                sleep_duration.sleep()
                recorder.stop()
                self.pub.publish(String(recorder.output_file_path))
                recorder.output_file_path = self.path + "output_mic" + str(i) + ".wav"



class PepperTalks:
    def __init__(self, robot_ip, port):
        self.tts = ALProxy("ALTextToSpeech", robot_ip, port)
        self.tts.setLanguage("English")

    def say_sound(self, sounds):
        if sounds:
            sentence = "I have heard " + 'and '.join([sound[0] for sound in sounds])
            self.tts.post.say(sentence)


SoundRecorderPublisher().start_recording_pc()


