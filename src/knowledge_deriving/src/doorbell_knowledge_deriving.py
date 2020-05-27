#!/usr/bin/env python
import rosplan_pytools.controller.knowledge_base as kb
import rospy
from sound_analyzers.msg import RecognisedSounds
import rosplan_pytools.common.utils as pytools_utils


class DoorbellKnowledgeDeriving():
    def __init__(self):
        """
        This will derive knowledge when an alarm bell rings
        """
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('hectic_knowledge_deriving', anonymous=True)
        rospy.Subscriber('sound_recognised', RecognisedSounds, self.sound_recognised)
        self.notified = False
        rospy.spin()

    def sound_recognised(self, sounds):
        """
        When an alarm bell rings, put it into the knowledge base
        """
        sounds = sounds.sounds
        ringing = False
        for elem in sounds:
            if elem.sound == "an Alarm bell ringing":
                ringing = True
                if not self.notified:
                    kb.add_predicate(pytools_utils.predicate_maker("doorbell-ringing", "Location", "kitchen",
                                                               is_negative=False))
                    self.notified = True
        if not ringing and self.notified:
            kb.add_predicate(pytools_utils.predicate_maker("doorbell-ringing", "Location", "kitchen",
                                                           is_negative=True))
            self.notified = False


DoorbellKnowledgeDeriving()
