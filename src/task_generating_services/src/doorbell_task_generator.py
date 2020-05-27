#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rospy

import rosplan_pytools.common.utils as pytools_utils

if __name__ == "__main__":
    rospy.init_node("doorbell_task_generator")
    # This task generator will set the to open the goal to open the door
    kb.initialize(prefix="/rosplan_knowledge_base")
    while not rospy.is_shutdown():
        respond = False
        dog = ""
        doorbells = kb.list_predicates("doorbell-ringing")
        for doorbell in doorbells:
            if not doorbell.is_negative:
                respond = True
        if respond and len(kb.list_goals()) == 0:
            goal = pytools_utils.predicate_maker("opened-door", "robot", "pepper")
            kb.add_goal(goal)
            rospy.loginfo("Goal posted")
            pytools_utils.plan_and_execute()
            print("planning done")
            kb.remove_goal(goal)
            goal.is_negative = True
            kb.add_predicate(goal)
            rospy.sleep(40)
        rospy.sleep(5.)