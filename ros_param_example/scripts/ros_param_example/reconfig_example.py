#! /usr/bin/env python3
import rospy
from dynamic_reconfigure.server import Server
from ros_param_example.cfg import ReconfigExampleConfig


class ReconfigExample:
    def __init__(self):
        self.srv = Server(ReconfigExampleConfig, self.reconfig)

    def reconfig(self, config, level):
        rospy.loginfo('A parameter was changed!')

        if config.enable:
            rospy.loginfo('Enabled')
        else:
            rospy.loginfo('Disabled')

        rospy.loginfo(f'String value: {config.str}')
        rospy.loginfo(f'X value: {config.x}')
        rospy.loginfo(f'Y value: {config.y}')

        if config.list == ReconfigExampleConfig.ReconfigExample_Option_1:
            rospy.loginfo('Option 1 was selected')
        elif config.list == ReconfigExampleConfig.ReconfigExample_Option_2:
            rospy.loginfo('Option 2 was selected')
        elif config.list == ReconfigExampleConfig.ReconfigExample_Option_3:
            rospy.loginfo('Option 3 was selected')
        else:
            pass

        return config


if __name__ == '__main__':
    rospy.init_node('reconfig_example')
    node_instance = ReconfigExample()
    rospy.spin()
