#!/usr/bin/env python
from sound_interface.streamgenerators.RobotAudioStream import RobotAudioStream
from sound_interface.streamhandlers.StreamRecorder import StreamRecorder
import rospy

from std_msgs.msg import String
from sound_interface.streamgenerators.MircrophoneStream import MicrophoneStream
import qi
import time

robot_ip = "127.0.0.1"
port = 9559


class SoundRecorderPublisher:
    def __init__(self):
        """
        This node will record audio clips of 10 seconds
        """
        rospy.init_node('sound_recorder', anonymous=True)
        # Create the publisher
        self.pub = rospy.Publisher('sound_recorder', String, queue_size=10)
        # The path where to record audio to
        self.path = "/home/jonas/recorded_sounds/"
        # self.path = rospy.get_param("recording_path")
        # Create the sound recogniser
        rospy.loginfo("Created recorder")

    def start_recording_bot(self, ip, port):
        """
        Method for recording from the mic of the robot
        @param ip: ip adress of the robot
        @param port: the port to connect to the robot
        """
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + "tcp://" + ip + ":" + str(port)])
        app.start()
        session = app.session
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
        """
        Start recording using the mic of the PC. Takes samples of 10 seconds
        """
        # Either 8 or 7 depending on config
        with MicrophoneStream(8,
                              1024) as stream:  # input device index can be determined with the list_input_devices method.
            # The amount of recordings before starting back at 1
            max_i = 10
            i = 0
            recorder = StreamRecorder(stream, self.path + "output_mic" + str(i) + ".wav")
            sleep_duration = rospy.Rate(0.1)
            while not rospy.is_shutdown():
                i += 1
                i %= max_i
                recorder.start()
                sleep_duration.sleep()
                recorder.stop()
                self.pub.publish(String(recorder.output_file_path))
                recorder.output_file_path = self.path + "output_mic" + str(i) + ".wav"


SoundRecorderPublisher().start_recording_pc()
