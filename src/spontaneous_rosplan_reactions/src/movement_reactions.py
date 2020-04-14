#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosplan_pytools.interfaces.action_interface as act_int
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils


class MoveRobot(act_int.SimpleAction):
    name = "moverobotaction"
    pub = rospy.Publisher('robot_command', String, queue_size=10)

    def _start(self, robot, fromlocation, tolocation):
        rospy.loginfo("MOVING ROBOT")
        # The messages are used for creating the emotional model

        movement = " ".join(["move", fromlocation, tolocation])
        self.pub.publish(String(movement))
        rospy.sleep(2)
        rospy.loginfo("Published :" + movement)
        kb.add_predicate(pytools_utils.predicate_maker("robot-at", ["robot", "location"], [robot, fromlocation], True))
        kb.add_predicate(pytools_utils.predicate_maker("robot-at", ["robot", "location"], [robot, tolocation]))
        rospy.loginfo("Reporting succes")
        super(MoveRobot, self)._report_success()


class PickupTreat(act_int.SimpleAction):
    name = "robottaketreataction"

    def _start(self, robot, treatlocation, treat):
        rospy.loginfo("Picking up treat")
        # The messages are used for creating the emotional model
        self.pub = rospy.Publisher('robot_command', String, queue_size=10)
        self.pub.publish(String("grasp"))
        rospy.sleep(2)
        kb.add_predicate(pytools_utils.predicate_maker("robot-holds", ["robot", "holdingobject"], [robot, treat]))
        super(PickupTreat, self)._report_success()


if __name__=="__main__":
    rospy.init_node("dog_reactor")
    act_int.initialize_actions()
    kb.initialize(prefix="/rosplan_knowledge_base")
    act_int.start_actions("rosplan_plan_dispatcher/action_dispatch", "rosplan_plan_dispatcher/action_feedback")
    rospy.spin()