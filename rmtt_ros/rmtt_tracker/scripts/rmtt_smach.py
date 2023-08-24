#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import RegionOfInterest as ROI
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8
import smach
import smach_ros
import datetime
import numpy as np


# define state Tag_track
class Tag_track(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global pad_id, mission
        rospy.loginfo('Executing state Tag_track')
        if pad_id == 1:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 2
            return 'next'
        else:
            return 'stay'


# define state Rise
class Rise(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global height, mission
        rospy.loginfo('Executing state Rise')
        if height > 1.5:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 3
            return 'next'
        else:
            vel_rise = Twist()
            vel_rise.linear.z = 0.3
            pub.publish(vel_rise)
            return 'stay'


# define state Turn
class Turn(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global mission
        rospy.loginfo('Executing state Turn')
        if circle_track != 0:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 4
            return 'next'
        else:
            vel_turn = Twist()
            vel_turn.angular.z = -0.3
            pub.publish(vel_turn)
            return 'stay'


# define state Track_circle
class Track_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global mission
        rospy.loginfo('Executing state Track')
        if (0.02 >= cross_circle_z >= -0.02) and (0.044 >= cross_circle_y >= -0.044) and cross_circle_z != 0.0:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 5
            return 'next'
        else:
            return 'stay'


# define state Cross_circle
class Cross_circle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['next', 'stay'])

    def execute(self, userdata):
        global mission, pose0
        rospy.loginfo('Executing state Cross')
        if pose0.pose.position.x != 0:
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 6
            return 'next'
        else:
            vel_cross = Twist()
            vel_cross.linear.x = 0.3
            rospy.loginfo("VX: "+str(vel_cross.linear.x))
            pub.publish(vel_cross)
            return 'stay'


# # define state Up_down
# class Up_down(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['stay', 'next'])
#
#     def execute(self, userdata):
#         global mission, pose0
#         rospy.loginfo('Executing state Down')
#         if pose0.pose.position.z <= 1.0:
#             zero_twist = Twist()
#             pub.publish(zero_twist)
#             mission = 7
#             return 'next'
#         else:
#             down_twist = Twist()
#             down_twist.linear.z = -0.2
#             pub.publish(down_twist)
#             return 'stay'

# define state Traj_track
class Traj_track(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['stay'])

    def execute(self, userdata):
        global mission, pose0
        rospy.loginfo('Executing state Traj')
        rospy.loginfo("position_x: " + str(pose0.pose.position.x) + " position_y: " + str(pose0.pose.position.y))
        if (1.2 <= pose0.pose.position.x <= 1.3) and (-1.2 <= pose0.pose.position.y <= -1.3):
            zero_twist = Twist()
            pub.publish(zero_twist)
            mission = 7
            rospy.loginfo("Stop")
            # return 'next'
        else:
            return 'stay'

# define state Go_point
# class Go_point(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,
#                              outcomes=['next', 'stay'])
#
#     def execute(self, userdata):
#         global mission, pose
#         rospy.loginfo('Executing state Point')
#         if pose is not None:
#             zero_twist = Twist()
#             pub.publish(zero_twist)
#             mission = 7
#             return 'next'
#         else:
#             return 'stay'

def callback_cmd_tag(msg):
    if mission == 1:
        vel_tag = msg
        pub.publish(vel_tag)


def callback_mission_pad(msg):
    global pad_id
    pad_id = msg.data


def callback_tof_btm(msg):
    global height
    height = msg.range


def callback_track(msg):
    global circle_track
    circle_track = msg.width


def callback_cross(msg):
    if mission == 4:
        vel_circle = msg
        pub.publish(vel_circle)
    global cross_circle_z, cross_circle_y
    cross_circle_z = msg.linear.z
    cross_circle_y = msg.angular.z
    rospy.loginfo("linear_z: "+str(cross_circle_z))



def callback_pose(msg):
    global pose0
    pose0 = msg
def callback_traj(msg):
    if mission == 6:
        vel_traj = msg
        pub.publish(vel_traj)
# def callback_point(msg):



if __name__ == '__main__':
    mission = 1
    pad_id = 0
    circle_track = 0
    cross_circle_z = 0
    cross_circle_y = 0
    vel_tag = Twist()
    vel_circle = Twist()
    vel_traj = Twist()
    # vel_point_x = 0.0
    # vel_point_y = 0.0
    pose0 = PoseStamped()
    start = 0

    rospy.init_node('state_machine')

    rospy.Subscriber('/mission_pad_id', UInt8, callback_mission_pad)
    rospy.Subscriber('/cmd_vel_tag', Twist, callback_cmd_tag)
    rospy.Subscriber('/tof_btm', Range, callback_tof_btm)
    rospy.Subscriber('/circle_msg', ROI, callback_track)
    rospy.Subscriber('/cmd_vel_circle', Twist, callback_cross)
    # rospy.Subscriber('/cmd_vel_point', Twist, callback_point)
    rospy.Subscriber('/new_pose', PoseStamped, callback_pose)
    rospy.Subscriber('/cmd_vel_traj', Twist, callback_traj)
    # rospy.Subscriber('/cmd_vel_point', Twist, callback_point)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # rate
    # rate = rospy.Rate(10.0)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAG_TRACK', Tag_track(),
                               transitions={'next': 'RISE', 'stay': 'TAG_TRACK'})
        smach.StateMachine.add('RISE', Rise(),
                               transitions={'next': 'TURN', 'stay': 'RISE'})
        smach.StateMachine.add('TURN', Turn(),
                               transitions={'next': 'TRACK_CIRCLE', 'stay': 'TURN'})
        smach.StateMachine.add('TRACK_CIRCLE', Track_circle(),
                               transitions={'next': 'CROSS_CIRCLE', 'stay': 'TRACK_CIRCLE'})
        smach.StateMachine.add('CROSS_CIRCLE', Cross_circle(),
                               transitions={'next': 'TRAJ_TRACK', 'stay': 'CROSS_CIRCLE'})
        # smach.StateMachine.add('GO_POINT', Go_point(),
        #                        transitions={'next': 'TRAJ_TRACK', 'stay': 'GO_POINT'})
        # smach.StateMachine.add('UP_DOWN', Up_down(),
        #                        transitions={'next': 'TRAJ_TRACK', 'stay': 'UP_DOWN'})
        smach.StateMachine.add('TRAJ_TRACK', Traj_track(),
                               transitions={'stay': 'TRAJ_TRACK'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    while not rospy.is_shutdown():
        outcome = sm.execute()
        # rate.sleep()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()
