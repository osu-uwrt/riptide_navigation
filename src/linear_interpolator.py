#! /usr/bin/env python
import rospy
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import transform

class ExecuteTrajectory(object):

    def __init__(self):
        self.actionSub = rospy.Subscriber("/execute_trajectory/goal/", ExecuteTrajectoryActionGoal, self.execute_cb)

    def execute_cb(self, goal):
        rospy.loginfo("There are %d points in the trajectory!" % len(goal.goal.trajectory.multi_dof_joint_trajectory.points))
        
        # store the x y and z translations from since last time interval
        xDelta = ...geometry_msgs.transform.translation.x
        yDelta = ...geometry_msgs.transform.translation.y
        zDelta = ...geometry_msgs.transform.translation.z
        
        # store positions as p
        p = ...trajectory_msgs.JointTrajectoryPoint.positions
        # store velocities as v
        v = ...trajectory_msgs.JointTrajectoryPoint.velocities
        # store accelerations as a
        a = ...trajectory_msgs.JointTrajectoryPoint.accelerations

        p.append()
        v.append()
        a.append()






        


if __name__ == '__main__':
    rospy.init_node('execute_trajectory')
    server = ExecuteTrajectory()
    rospy.spin()
