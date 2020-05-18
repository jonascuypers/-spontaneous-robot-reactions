#!/usr/bin/env python
import rospy
import rosplan_pytools.controller.knowledge_base as kb
import numpy as np
import rosplan_pytools.common.utils as pytools_utils
from std_msgs.msg import String


class LoudSoundsKnowledgeDeriving:
    def __init__(self):
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('loud_sounds_knowledge_deriving', anonymous=True)
        rospy.Subscriber('sound_volume', String, self.sound_volume_detected)
        self.count_high_volume = 0
        self.high_volume_limit = 60
        self.launch_helper_limit = 3
        self.loud_noise = False
        self.noise_place = ""
        rospy.spin()

    def sound_volume_detected(self, volume):
        volume = float(volume.data)
        if volume > self.high_volume_limit:
            self.count_high_volume += 1
        else:
            self.count_high_volume = 0
        if self.count_high_volume >= self.launch_helper_limit and self.loud_noise is False:
            self.loud_noise = True
            self.noise_place = self.get_bot_location()
            kb.add_predicate(pytools_utils.predicate_maker("loud-volume", "Location", self.noise_place))
        elif self.count_high_volume < self.launch_helper_limit and self.loud_noise is True:
            self.loud_noise = False
            kb.add_predicate(pytools_utils.predicate_maker("loud-volume", "Location", self.noise_place, True))

    def get_bot_location(self):
        real_bot_location = ""
        bot_locations = kb.list_predicates("robot-at")
        for bot_location in bot_locations:
            if not bot_location.is_negative:
                real_bot_location = bot_location.values[1].value
        return real_bot_location

LoudSoundsKnowledgeDeriving()