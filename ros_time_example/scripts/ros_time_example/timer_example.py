#! /usr/bin/env python3
import rospy


class TimerExample:
    def __init__(self):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        if event.last_real is None:
            return

        actual_time_diff = event.current_real - event.last_real
        expected_time_diff = event.current_expected - event.last_expected

        rospy.loginfo(f'Actual time since last trigger: {actual_time_diff.to_sec()}')
        rospy.loginfo(f'Expected time difference: {expected_time_diff.to_sec()}')


if __name__ == '__main__':
    rospy.init_node('timer_example')
    node_instance = TimerExample()
    rospy.spin()