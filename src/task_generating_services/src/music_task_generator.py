#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rospy
import rosplan_pytools.common.utils as pytools_utils

if __name__=="__main__":
    rospy.init_node("music_task_generator")

    kb.initialize(prefix="/rosplan_knowledge_base")
    while not rospy.is_shutdown():
        respond = False
        loud_noise_predicates = kb.list_predicates("hectic-environment")
        for loud_noise_pred in loud_noise_predicates:
            if not loud_noise_pred.is_negative:
                respond = True
        if respond and len(kb.list_goals()) == 0:
            goal = pytools_utils.predicate_maker("music-played", ["robot"], ["pepper"])
            kb.add_goal(goal)
            rospy.loginfo("Goal posted")
            pytools_utils.plan_and_execute()
            print("planning done")
            kb.remove_goal(goal)
            goal.is_negative = True
            kb.add_predicate(goal)
            rospy.sleep(60*5)
        rospy.sleep(5.)