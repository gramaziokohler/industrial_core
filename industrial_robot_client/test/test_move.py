#!/usr/bin/env python
import time
import rospy
from control_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from industrial_msgs.msg import *
from industrial_msgs.srv import *


def move_joint_path_command():
    # This test assumes a 6-DoF arm with numbered joints
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    Q1 = [0,0,0,0,0,0]
    Q2 = [0,0,1.57,-3.14,-1.57,0]
    Q3 = [0,0,0,0,0,0]

    rospy.wait_for_service('joint_path_command')
    try:
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        trajectory.points = [
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

        joint_path_command = rospy.ServiceProxy('joint_path_command', CmdJointTrajectory)
        joint_path_command(trajectory)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def move_cartesian_path_command():
    rospy.wait_for_service('cartesian_path_command')
    try:
        P1 = [0.400, 0.000, 0.300]
        P2 = [0.450, 0.000, 0.300]
        P3 = [0.450, 0.050, 0.300]
        P4 = [0.400, 0.050, 0.300]
        P5 = [0.400, 0.050, 0.250]
        P6 = [0.450, 0.050, 0.250]
        P7 = [0.450, 0.000, 0.250]
        P8 = [0.400, 0.000, 0.250]
        orient = Quaternion(0.995, -0.004, -0.043, 0.087)

        trajectory = CartesianTrajectory()
        for step, point in enumerate([P1, P2, P3, P4, P5, P6, P7, P8, P1]):
            trajectory.points.append(
                CartesianTrajectoryPoint(pose=Pose(Point(*point), orient), velocity=1, acceleration=1, blending_radius=0, time_from_start=rospy.Duration(2.0 + step)))

        cartesian_path_command = rospy.ServiceProxy('cartesian_path_command', CmdCartesianTrajectory)
        cartesian_path_command(trajectory)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    try:
        rospy.init_node("test_moves", anonymous=True, disable_signals=True)

        print('Starting move, press CTRL+C to exit')
        #move_joint_path_command()
        move_cartesian_path_command()

        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
