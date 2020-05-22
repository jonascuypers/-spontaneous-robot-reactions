#!/usr/bin/env python
import rospy
import rosplan_pytools.interfaces.action_interface as act_int
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils
from std_msgs.msg import String


class AskAllGood(act_int.SimpleAction):
    name = "askallgoodverboseaction"
    pub = rospy.Publisher('tts', String, queue_size=10)

    def _start(self, robot, person, emotion, location, topic):
        """
        Ask if everything is ok. What is said depends on the topic
        """
        rospy.loginfo("Asking all good on topic: " + topic)

        if str(topic) == "general":
            self.pub.publish(String("Hey " + person + ". Is everything ok or is there something I can do for you?"))
        elif str(topic) == "kitchenhelping":
            self.pub.publish(String("Hey " + person + ". Should I help you with cooking?"))
        kb.add_predicate(pytools_utils.predicate_maker("asked-all-good", ["person", "topic"], [person, topic]))
        super(AskAllGood, self)._report_success()


class PassiveAskAllGood(act_int.SimpleAction):
    name = "askallgoodpassiveaction"
    pub = rospy.Publisher('tts', String, queue_size=10)

    def _start(self, robot, person, emotion, location, topic):
        """
        This action is blank. It displays the possibility to react when having a certain emotion
        """
        # if str(topic) == "general":
        #     self.pub.publish(String("Hey, I will not talk to you"))
        # elif str(topic) == "kitchenhelping":
        #     self.pub.publish(String("Hey kitchen man, I will not talk to help"))
        kb.add_predicate(pytools_utils.predicate_maker("asked-all-good", ["person", "topic"], [person, topic]))
        super(PassiveAskAllGood, self)._report_success()


class MusicPlaying(act_int.SimpleAction):
    name = "startplayingmusicaction"
    music_map = [[-1, -1, "sad"], [1, -1, "chill holiday"], [1, 1, "happy"], [-1, 1, "rock"]]
    pub = rospy.Publisher('tts', String, queue_size=10)

    def _start(self, robot, person, location):
        """
        Suggest to play music. Adapt the music to the emotional state
        """
        emotions = rospy.get_param('emotion')
        sign = lambda x: -1 if x < 0 else 1
        suggested_style = 0
        for style in self.music_map:
            if sign(emotions["valence"]) == style[0] and sign(emotions["arousal"]) == style[1]:
                suggested_style = style[2]

        self.pub.publish(String("Hey {}. Should I play some {} music?".format(person, suggested_style)))
        kb.add_predicate(pytools_utils.predicate_maker("asked-music-play", "robot", robot))
        super(MusicPlaying, self)._report_success()


if __name__ == "__main__":
    rospy.init_node("talking_reactions")
    act_int.initialize_actions()
    kb.initialize(prefix="/rosplan_knowledge_base")
    act_int.start_actions("rosplan_plan_dispatcher/action_dispatch", "rosplan_plan_dispatcher/action_feedback")
    rospy.spin()
