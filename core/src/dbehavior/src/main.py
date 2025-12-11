#!/usr/bin/env python3
import rospy
import sys

from dbehavior.core import DBlackboard


def print_status(blackboard):
    if blackboard.param.print_status:
        rospy.loginfo(
            'target ({:.2f},{:.2f})'.format(
                blackboard.attack_target.x,
                blackboard.attack_target.y))
        rospy.loginfo(
            'robot ({:.2f},{:.2f},{:.2f}), ball({:.2f},{:.2f},{:.2f})'.format(
                blackboard.vision_info.robot_pos.x,
                blackboard.vision_info.robot_pos.y,
                blackboard.vision_info.robot_pos.z,
                blackboard.vision_info.ball_field.x,
                blackboard.vision_info.ball_field.y,
                blackboard.vision_info.ball_field.z))


if __name__ == '__main__':
    # initialize ROS node
    rospy.init_node('behavior_node', log_level=rospy.INFO)
    rate = rospy.Rate(30)

    # initialize behavior
    bb = DBlackboard()
    bb.print_information()
    # bb.set_orientation(bb.start_pos.z)

    # initialize skill
    from dbehavior.util.util import find_behavior

    cls = find_behavior(bb.param.role)
    # cls = Striker(*)
    behavior = cls(bb)
    # behavior = Striker(bb)

    # run ros node
    while not rospy.is_shutdown():
        try:
            # if bb.lower_board_lost:
            if not bb.lower_board_lost:
                # print('2222222222222')
                print_status(bb)
                bb.reset()
                # print('2222222222222')
                behavior.step()
                # print('2222222222222')
                bb.publish()
            rate.sleep()
        except KeyboardInterrupt:
            sys.exit(0)
        # except:
        #     err_type, err_msg, _ = sys.exc_info()
        #     rospy.logerr('[{}] {}'.format(err_type.__name__, err_msg))
        #     rate.sleep()
