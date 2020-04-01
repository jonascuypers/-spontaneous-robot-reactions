#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosplan_pytools.interfaces.action_interface as act_int
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils
pub = rospy.Publisher('sound_volume', String, queue_size=10)

class TreatGiving(act_int.SimpleAction):
    name = "givedogtreataction"

    def _start(self, speech, dog, robot, doglocation):
        rospy.loginfo("GIVING FOOD TO DOG")
        kb.add_predicate(pytools_utils.predicate_maker("dog-barking", "dog", dog, True))
        kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", dog))
        super(TreatGiving, self)._report_success()


class DogTalking(act_int.SimpleAction):
    name = "saydogsilentaction"

    def _start(self, speech, dog):
        rospy.loginfo("GIVING FOOD TO DOG")
        kb.add_predicate(pytools_utils.predicate_maker("dog-barking", "dog", dog, True))
        kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", dog))
        super(DogTalking, self)._report_success()


class MakeSilent(act_int.SimpleAction):
    name = "makesilentaction"

    def _start(self, speech, dog):
        rospy.loginfo("MAKING HUMAN SILENT")
        kb.add_predicate(pytools_utils.predicate_maker("human-talking", "speech", speech, True))
        kb.add_predicate(pytools_utils.predicate_maker("human-silent", "speech", speech))
        rospy.loginfo(str(kb.list_goals()))
        super(MakeSilent, self)._report_success()

class MoveRobot(act_int.SimpleAction):
    name = "moverobotaction"

    def _start(self, robot, fromlocation, tolocation):
        rospy.loginfo("MOVING ROBOT")
        kb.add_predicate(pytools_utils.predicate_maker("robot-at", ["robot", "location"], [robot, fromlocation], True))
        kb.add_predicate(pytools_utils.predicate_maker("robot-at", ["robot", "location"], [robot, tolocation]))
        super(MoveRobot, self)._report_success()


if __name__=="__main__":
    rospy.init_node("dog_reactor")
    act_int.initialize_actions()
    kb.initialize(prefix = "/rosplan_knowledge_base")
    act_int.start_actions("rosplan_plan_dispatcher/action_dispatch", "rosplan_plan_dispatcher/action_feedback")
    rospy.spin()
