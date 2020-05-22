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
        """
        Do the give robot treat action.
        The arguments are all arguments in the PDDL action
        """
        rospy.loginfo("GIVING FOOD TO DOG")
        self.pub.publish(String("grasp off"))
        # Wait until closed
        rospy.sleep(2)
        kb.add_predicate(pytools_utils.predicate_maker("dog-interaction", ["robot", "dog"], [robot,  dog]))
        kb.add_predicate(pytools_utils.predicate_maker("robot-holds", ["robot", "holdingobject"], [robot, treat], True))
        super(TreatGiving, self)._report_success()


class SayDogSilent(act_int.SimpleAction):
    name = "saydogsilentaction"
    pub = rospy.Publisher('tts', String, queue_size=10)

    def _start(self, dog, human, robot, doglocation):
        """
        Tell the dog to be silent
        """
        self.pub.publish(String("Hey " + str(dog) + ", be silent!"))
        kb.add_predicate(pytools_utils.predicate_maker("dog-interaction", ["robot", "dog"], [robot,  dog]))
        super(SayDogSilent, self)._report_success()


if __name__=="__main__":
    rospy.init_node("dog_reactor")
    act_int.initialize_actions()
    kb.initialize(prefix="/rosplan_knowledge_base")
    act_int.start_actions("rosplan_plan_dispatcher/action_dispatch", "rosplan_plan_dispatcher/action_feedback")
    rospy.spin()
