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
        
        points = goal.goal.trajectory.multi_dof_joint_trajectory.points

        p = []
        v = []
        a = []

        # create a response
        import MultiDOFJointTrajectoryPoint as response

        for i in range(len(points)):
            point = points[i]

            # store the x y z and time at current position
            x = point.transforms.translation.x
            y = point.transforms.translation.y
            z = point.transforms.translation.z


            time = points[i + 1].time_from_start.to_sec() - points[i - 1].time_from_start.to_sec()

            # add position to array p      
            pos = [x, y, z]
            p.append(pos)

            # add velocity to array v
            if (i == 0):
                v.append([0, 0, 0])
            elif (i == len(points)):
                # I am not sure what to put at the end of velocity array
            else:
                p2 = p[i + 1]
                p1 = p[i - 1]
                vel = [(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time]
                v.append(vel)

            # add acceleration to array a
            if (i = 0):
                a.append([0, 0, 0])
            elif (i = len(points)):
                #I am not sure what to put at end of accel array
            else:
                v2 = v[i + 1]
                v1 = v[i - 1]
                acc = [(v2[0] - v1[0])/time, (v2[1] - v1[1])/time, (v2[2] - v1[2])/time]
                a.append(acc)

        # add the velocity and the acceleration to the response variable
        response.velocities.linear = v
        response.accelerations.linear = a

        return response

if __name__ == '__main__':
    rospy.init_node('execute_trajectory')
    server = ExecuteTrajectory()
    rospy.spin()
