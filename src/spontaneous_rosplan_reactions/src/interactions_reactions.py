#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import rosplan_pytools.interfaces.action_interface as act_int
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils
from emotional_reasoner.srv import EmotionService

class AskAllGood(act_int.SimpleAction):
    name = "askallgoodverboseaction"

    def _start(self, robot, person, emotion, location, topic):
        rospy.loginfo("Asking all good on topic: " + topic)
        if str(topic) == "general":
            pytools_utils.tts("Hey " + person + ". Is everything ok or is there something I can do for you?")
        elif str(topic) == "kitchenhelping":
            pytools_utils.tts("Hey " + person + ". Should I help you with cooking?")
        kb.add_predicate(pytools_utils.predicate_maker("asked-all-good", ["person", "topic"], [person, topic]))
        super(AskAllGood, self)._report_success()

class PassiveAskAllGood(act_int.SimpleAction):
    name = "askallgoodpassiveaction"

    def _start(self, robot, person, emotion, location, topic):
        rospy.loginfo("Asking all good on topic: " + topic)
        if str(topic) == "general":
            pytools_utils.tts("Hey, I will not talk to you")
        elif str(topic) == "kitchenhelping":
            pytools_utils.tts("Hey kitchen man, I will not talk to help")
        kb.add_predicate(pytools_utils.predicate_maker("asked-all-good", ["person", "topic"], [person, topic]))
        super(PassiveAskAllGood, self)._report_success()


class MusicPlaying(act_int.SimpleAction):
    name = "startplayingmusicaction"
    music_map = [[-1, -1, "sad"], [1, -1, "chill holiday"], [1, 1, "happy"], [-1, 1, "rock"]]

    def _start(self, robot, person, location):
        rospy.wait_for_service('emotion_service')
        emotion_service = rospy.ServiceProxy('emotion_service', EmotionService)
        emotions = emotion_service()
        rospy.loginfo("valence:" + str(emotions.valence) + "arousal: " + str(emotions.arousal))
        sign = lambda x: -1 if x < 0 else 1
        suggested_style = 0
        for style in self.music_map:
            if sign(emotions.valence) == style[0] and sign(emotions.arousal) == style[1]:
                suggested_style = style[2]
        pytools_utils.tts("Hey {}. Should I play some {} music?".format(person, suggested_style))
        kb.add_predicate(pytools_utils.predicate_maker("asked-music-play", "robot", robot))
        super(MusicPlaying, self)._report_success()


if __name__=="__main__":
    rospy.init_node("talking_reactions")
    act_int.initialize_actions()
    kb.initialize(prefix="/rosplan_knowledge_base")
    act_int.start_actions("rosplan_plan_dispatcher/action_dispatch", "rosplan_plan_dispatcher/action_feedback")
    rospy.spin()
