#!/usr/bin/env python
from sound_analyzers.msg import RecognisedSounds
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils
import rospy

class HecticSoundKnowledgeGenerator:

    def __init__(self):
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('hectic_knowledge_deriving', anonymous=True)
        rospy.Subscriber('sound_recognised', RecognisedSounds, self.sound_recognised)
        # min number of times at least one sound was heard
        self.min_sound_history_required = 7
        # min number of different sounds
        self.min_diff_sounds_required = 4
        # the sliding window
        self.sounds_history = [0 for i in range(14)]
        self.location_bot = ""
        self.published = False
        rospy.spin()

    def sound_recognised(self, sounds):
        sounds = sounds.sounds
        all_sounds = [elem.sound for elem in sounds]
        if len(all_sounds) == 0:
            all_sounds = 0
        self.sounds_history.append(all_sounds)
        self.sounds_history = self.sounds_history[1:]
        # Check if there are at least min_diff_sounds_required different sounds
        # In at least min_required of the len(sounds history)
        count = 0
        nr_different = set()
        for sound_hist in self.sounds_history:
            if sound_hist != 0:
                count += 1
                for sound in sound_hist:
                    nr_different.add(sound)
        if count >= self.min_sound_history_required and len(nr_different) >= self.min_diff_sounds_required:
            if not self.published:
                self.location_bot = self.get_bot_location()
                knowledge = pytools_utils.predicate_maker("hectic-environment", "location", self.location_bot)
                kb.add_predicate(knowledge)
                self.published = True
        else:
            if self.published:
                knowledge = pytools_utils.predicate_maker("hectic-environment", "location", self.location_bot,
                                                          True)
                kb.add_predicate(knowledge)
                self.published = False

    def get_bot_location(self):
        real_bot_location = ""
        bot_locations = kb.list_predicates("robot-at")
        for bot_location in bot_locations:
            if not bot_location.is_negative:
                real_bot_location = bot_location.values[1].value
        return real_bot_location


HecticSoundKnowledgeGenerator()