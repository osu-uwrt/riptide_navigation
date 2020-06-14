#! /usr/bin/env python
import rospy
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory

class ExecuteTrajectory(object):

    def __init__(self):
        self.actionSub = rospy.Subscriber("/execute_trajectory/goal/", ExecuteTrajectoryActionGoal, self.execute_cb)
        self.actionPub = rospy.Publisher("topic_name", MultiDOFJointTrajectory, queue_size=1)

    def execute_cb(self, goal):
        rospy.loginfo("There are %d points in the trajectory!" % len(goal.goal.trajectory.multi_dof_joint_trajectory.points))
        
        # array of each point as a move_it msg
        points = goal.goal.trajectory.multi_dof_joint_trajectory.points

        # arrays for position, velocity, and acceleration
        p = []
        v = []
        a = []

        # create a response (?)
        response = MultiDOFJointTrajectory()

        for i in range(len(points)):
            point = points[i]

            # store the x y z and time at current position
            x = point.transforms.translation.x
            y = point.transforms.translation.y
            z = point.transforms.translation.z

            # add position to array p      
            pos = [x, y, z]
            p.append(pos)

            # add velocity to array v
            times = []
            if (i == 0):
                p2 = p[1]
                p1 = p[0]
                times.append(points[1].time_from_start.to_sec() - points[0].time_from_start.to_sec())
            elif (i == len(points) - 1):
                end = len(points) - 1
                p2 = p[end]
                p1 = p[end - 1]
                times.append(points[end].time_from_start.to_sec() - points[end].time_from_start.to_sec())
            else:
                p2 = p[i + 1]
                p1 = p[i - 1]
                time.append(points[i + 1].time_from_start.to_sec() - points[i - 1].time_from_start.to_sec())
            time = times[i]
            vel = [(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time]
            v.append(vel)
            response[i].velocities.linear.x = vel[0]
            response[i].velocities.linear.y = vel[1]
            response[i].velocities.linear.z = vel[2]

        for j in range(len(v)):
            time = times[j]
            # add acceleration to array a
            if (j == 0):
                p2 = v[1]
                p1 = v[0]
            elif (j == len(v) - 1):
                end = len(v) - 1
                p2 = v[end]
                p1 = v[end - 1]
            else:
                p2 = v[i + 1]
                p1 = v[i - 1]
            acc = [(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time]
            a.append(acc)
            response[i].accelerations.linear.x = acc[0]
            response[i].accelerations.linear.y = acc[1]
            response[i].accelerations.linear.z = acc[2]

        # publish the response to a topic
        self.actionPub.publish(response)

if __name__ == '__main__':
    rospy.init_node('linear_interpolator')
    server = ExecuteTrajectory()
    rospy.spin()
