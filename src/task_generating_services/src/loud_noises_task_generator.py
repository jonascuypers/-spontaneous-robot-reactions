#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.controller.planning_system as ps
import rospy
import rosplan_pytools.common.utils as pytools_utils
from std_msgs.msg import String


class LoudNoiseTaskGenerator:
    def __init__(self):
        rospy.init_node("loud_noise_task_generator")
        kb.initialize(prefix="/rosplan_knowledge_base")
        self.count_high_volume = 0
        self.high_volume_limit = 10000
        self.launch_helper_limit = 3
        self.action_done = False
        rospy.Subscriber('sound_volume', String, self.sound_volume_detected)
        rospy.spin()

    def sound_volume_detected(self, volume):
        volume = float(volume.data)
        if volume > self.high_volume_limit:
            self.count_high_volume += 1
        else:
            self.count_high_volume = 0
        if self.count_high_volume >= self.launch_helper_limit and not self.action_done:
            self.action_done = True
            goal = pytools_utils.predicate_maker("asked-all-good", ["person", "subject"], ["personbert", "general"])
            kb.add_goal(goal)
            rospy.loginfo("Goal posted")
            pytools_utils.plan_and_execute()
            print("planning done")
            kb.remove_goal(goal)


LoudNoiseTaskGenerator()
