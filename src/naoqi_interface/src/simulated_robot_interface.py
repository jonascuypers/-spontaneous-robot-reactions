#!/usr/bin/env python
import qi
import naoqi
import rospy
import time
import math
from std_msgs.msg import String

# This dictionary maps the movements from a place to another place
movements = {
    "livingroom": {"kitchen": {"names": ["LShoulderPitch", "RShoulderRoll", "HeadYaw"], "angles": [math.pi/2, -math.pi/2, -math.pi/2], "speed": 0.2}},
    "kitchen": {"livingroom": {"names": ["LShoulderPitch", "RShoulderRoll", "HeadYaw"], "angles": [0, 0, 0], "speed": 0.2}}
}


class SimulatedRobotInterface:
    def __init__(self, ip, port):
        rospy.init_node('robot_interface', anonymous=True)
        session = qi.Session()
        try:
            session.connect("tcp://" + ip + ":" + str(port))
        except RuntimeError:
            print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) + ".\n")
        self.motion_service = session.service("ALMotion")
        self.posture_service = session.service("ALRobotPosture")
        self.motion_service.wakeUp()
        self.posture_service.goToPosture("StandInit", 0.5)
        time.sleep(2)
        rospy.Subscriber('robot_command', String, self.handle_robot_command)
        rospy.spin()

    def handle_robot_command(self, arg):
        """
        Converts a string to a command which must be executed on the robot
        It also executes the command
        Current commands: move, grasp
        """
        rospy.loginfo(arg.data)
        command = arg.data
        function = command.split()
        if function[0] == "move":
            self.move_robot(function[1], function[2])
        if function[0] == "grasp":
            if len(function) == 1:
                self.grasp()
            else:
                self.grasp(loosen=True)

    def move_robot(self, from_loc, to_loc):
        """
        Move the robot from the living room to the kitchen
        """
        movement = movements[from_loc][to_loc]
        self.motion_service.setAngles(movement["names"], movement["angles"], movement["speed"])

    def grasp(self, loosen=False):
        angle = int(loosen)
        self.motion_service.setAngles(["LHand"], [angle], 0.2)


robot_ip = rospy.get_param("robot_ip")
port = rospy.get_param("port")
SimulatedRobotInterface(robot_ip, port)
