#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rospy
import rosplan_pytools.common.utils as pytools_utils
from std_msgs.msg import String

if __name__=="__main__":
    rospy.init_node("loud_noise_task_generator")

    kb.initialize(prefix="/rosplan_knowledge_base")
    while not rospy.is_shutdown():
        respond = False
        loud_noise_predicates = kb.list_predicates("loud-volume")
        for loud_noise_pred in loud_noise_predicates:
            if not loud_noise_pred.is_negative:
                respond = True
        if respond and len(kb.list_goals()) == 0:
            goal = pytools_utils.predicate_maker("asked-all-good", ["person", "subject"], ["personbert", "general"])
            kb.add_goal(goal)
            rospy.loginfo("Goal posted")
            pytools_utils.plan_and_execute()
            print("planning done")
            kb.remove_goal(goal)
            rospy.sleep(60*5)
        rospy.sleep(5.)

