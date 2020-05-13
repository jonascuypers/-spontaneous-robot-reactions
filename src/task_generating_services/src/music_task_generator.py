#!/usr/bin/env python
from sound_analyzers.msg import RecognisedSounds
import rosplan_pytools.controller.knowledge_base as kb
import rosplan_pytools.common.utils as pytools_utils
import rospy

class MusicTaskGenerator:

    def __init__(self):
        kb.initialize(prefix="/rosplan_knowledge_base")
        rospy.init_node('music_task_generator', anonymous=True)
        rospy.Subscriber('sound_recognised', RecognisedSounds, self.music_playing_task_generator)
        self.min_sound_history_required = 10
        self.min_diff_sounds_required = 3
        self.sounds_history = [0 for i in range(14)]
        self.asked = False
        rospy.spin()

    def music_playing_task_generator(self, sounds):
        sounds = sounds.sounds
        all_sounds = [elem.sound for elem in sounds]
        if len(all_sounds) > 0:
            self.sounds_history.append(all_sounds)
            self.sounds_history = self.sounds_history[1:]
            # Check if there are at least min_diff_sounds_required different sounds
            # In at least min_required of the len(sounds history)
            count = 0
            nr_different = set()
            if not self.asked:
                for elem in self.sounds_history:
                    if elem != 0:
                        count += 1
                        for sound in sounds:
                            nr_different.add(sound)
            if count >= self.min_sound_history_required and len(nr_different) >= self.min_diff_sounds_required:
                goal = pytools_utils.predicate_maker("music-played", "robot", "pepper")
                kb.add_goal(goal)
                self.asked = True
                rospy.loginfo("Goal posted")
                pytools_utils.plan_and_execute()
                print("planning done")
                kb.remove_goal(goal)
        else:
            self.sounds_history.append(0)
            self.sounds_history = self.sounds_history[1:]


MusicTaskGenerator()

