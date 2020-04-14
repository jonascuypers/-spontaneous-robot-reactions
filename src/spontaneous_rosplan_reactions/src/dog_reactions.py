#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosplan_pytools.interfaces.action_interface as act_int
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils


class TreatGiving(act_int.SimpleAction):
    name = "givedogtreataction"
    pub = rospy.Publisher('robot_command', String, queue_size=10)
    def _start(self, dog, doglocation, robot, human, treat):
        rospy.loginfo("GIVING FOOD TO DOG")
        self.pub.publish(String("grasp off"))
        rospy.sleep(2)
        kb.add_predicate(pytools_utils.predicate_maker("dog-barking", "dog", dog, True))
        kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", dog))
        kb.add_predicate(pytools_utils.predicate_maker("robot-holds", ["robot", "holdingobject"], [robot, treat], True))
        pytools_utils.tts("I will now give food to " + str(dog))
        super(TreatGiving, self)._report_success()


class SayDogSilent(act_int.SimpleAction):
    name = "saydogsilentaction"

    def _start(self, dog, human, robot, doglocation):
        rospy.loginfo("GIVING FOOD TO DOG")
        kb.add_predicate(pytools_utils.predicate_maker("dog-barking", "dog", dog, True))
        kb.add_predicate(pytools_utils.predicate_maker("dog-silent", "dog", dog))
        pytools_utils.tts("Hey " + str(dog) + ", be silent!")
        super(SayDogSilent, self)._report_success()


if __name__=="__main__":
    rospy.init_node("dog_reactor")
    act_int.initialize_actions()
    kb.initialize(prefix="/rosplan_knowledge_base")
    act_int.start_actions("rosplan_plan_dispatcher/action_dispatch", "rosplan_plan_dispatcher/action_feedback")
    rospy.spin()
