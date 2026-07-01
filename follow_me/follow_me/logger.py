#!/usr/bin/env python3

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sound_play_py.libsoundplay import SoundClient

THROTTLE = 5.0


class Logger(Node):
    def __init__(self):
        super().__init__("logger")

        self.soundhandle = SoundClient(self)
        self.soundhandle.stopAll()
        self.get_logger().info("Ready to play sound")

        self.message_queue = deque()
        self.message_throttle = {}
        self.playing = False

        self.sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self.callback, 1
        )
        self.subs = self.create_subscription(
            String, "log_sound", self.msg_callback, 10
        )

        self.tim = self.create_timer(0.1, self.play)

    def callback(self, msg):
        for status in msg.status:
            for value in status.values:
                if value.key == "Active sounds" and value.value != "0":
                    self.playing = True
                    return
        self.playing = False

    def msg_callback(self, msg):
        keys = list(self.message_throttle.keys())
        if msg.data not in keys and msg.data not in self.message_queue:
            self.message_queue.append(msg.data)

    def play(self):
        # reset too old message throttle
        now = self.get_clock().now()
        keys = list(self.message_throttle.keys())
        for k in keys:
            if now - self.message_throttle[k] > Duration(seconds=THROTTLE):
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
        self.message_throttle[s] = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
