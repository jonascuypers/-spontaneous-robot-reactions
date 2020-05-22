#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rospy
import rosplan_pytools.common.utils as pytools_utils

if __name__ == "__main__":
    rospy.init_node("kitchen_helping_task_generator")

    kb.initialize(prefix="/rosplan_knowledge_base")
    while not rospy.is_shutdown():
        respond = False
        person = ""
        kitchen_predicates = kb.list_predicates("person-cooking")
        for kitchen_person in kitchen_predicates:
            if not kitchen_person.is_negative:
                respond = True
                person = kitchen_person.values[0].value
        if respond:
            goal = pytools_utils.predicate_maker("asked-all-good", ["person", "topic"], [person, "kitchenhelping"])
            kb.add_goal(goal)
            rospy.loginfo("Goal posted")
            pytools_utils.plan_and_execute()
            print("planning done")
            # Reset goal
            kb.remove_goal(goal)
            goal.is_negative = True
            kb.add_predicate(goal)
            rospy.sleep(60*5)
        rospy.sleep(5.)
