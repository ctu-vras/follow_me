#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from collections import deque
from sound_play.libsoundplay import SoundClient
from diagnostic_msgs.msg import DiagnosticArray

THROTTLE = 5.0


class Logger:
    def __init__(self):
        rospy.init_node("logger")

        self.soundhandle = SoundClient()
        rospy.sleep(1.0)
        self.soundhandle.stopAll()
        rospy.loginfo("Ready to play sound")

        self.message_queue = deque()
        self.message_throttle = {}
        self.playing = False

        self.sub = rospy.Subscriber(
            "/diagnostics", DiagnosticArray, callback=self.callback, queue_size=1
        )
        self.subs = rospy.Subscriber("log_sound", String, self.msg_callback)

        self.tim = rospy.Timer(rospy.Duration(0.1), self.play)

        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.tim.shutdown()
        rospy.sleep(0.2)

    def callback(self, msg):
        for status in msg.status:
            for value in status.values:
                if value.key == "Active sounds" and value.value != "0":
                    self.playing = True
                    return
        self.playing = False

    def msg_callback(self, msg):
        keys = list(self.message_throttle.keys())
        print(keys)
        print(msg.data)
        if msg.data not in keys and msg.data not in self.message_queue:
            print("adding")
            self.message_queue.append(msg.data)
        print()

    def play(self, _):
        # reset too old message throttle
        keys = list(self.message_throttle.keys())
        for k in keys:
            if rospy.Time.now() - self.message_throttle[k] > rospy.Duration(THROTTLE):
                self.message_throttle.pop(k)

        # check if some sound is already playing
        if self.playing:
            return

        # get the first message in queue
        if len(self.message_queue) == 0 or self.playing:
            return
        s = self.message_queue.popleft()

        # play the sound
        sound = self.soundhandle.voiceSound(s)
        sound.play()
        self.playing = True
        self.message_throttle[s] = rospy.Time.now()


if __name__ == "__main__":
    l = Logger()
    rospy.spin()
