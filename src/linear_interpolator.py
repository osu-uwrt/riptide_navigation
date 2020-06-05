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
        
        point1 = goal.goal.trajectory.multi_dof_joint_trajectory.points[len(points) - 1]
        point2 = goal.goal.trajectory.multi_dof_joint_trajectory.points[len(points)]

        # store the x y z and time at current position
        x2 = point2.transforms.translation.x
        y2 = point2.transforms.translation.y
        z2 = point2.transforms.translation.z

        time2 = time_from_start

        # store the x y z and time one index before current positions
        x1 = point1.transforms.translation.x
        y1 = point1.transforms.translation.y
        z1 = point1.transforms.translation.z

        time1 = time_from_start
        
        # store position as p:      
        
        # store velocity as v:
        
        # store acceleration as a:






        


if __name__ == '__main__':
    rospy.init_node('execute_trajectory')
    server = ExecuteTrajectory()
    rospy.spin()
