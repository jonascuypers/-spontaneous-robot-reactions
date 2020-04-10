#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.controller.planning_system as ps
import rospy

import rosplan_pytools.common.utils as pytools_utils

if __name__=="__main__":
    rospy.init_node("dog_task_generator")

    kb.initialize(prefix="/rosplan_knowledge_base")
    while not rospy.is_shutdown():
        respond = False
        dog = ""
        barking_predicates = kb.list_predicates("dog-barking")
        for barking_predicate in barking_predicates:
            if not barking_predicate.is_negative:
                respond = True
                dog = barking_predicate.values[0].value

        if respond:
            goal = pytools_utils.predicate_maker("dog-silent", "dog", dog)
            kb.add_goal(goal)
            print("planning")
            pytools_utils.plan_and_execute()
            print("planning done")
            kb.remove_goal(goal)
            rospy.sleep(60*5)
        rospy.sleep(5.)