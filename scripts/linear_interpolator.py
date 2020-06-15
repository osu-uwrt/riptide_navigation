#! /usr/bin/env python
import rospy
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from geometry_msgs.msg import Transform, Twist

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

        # convert multi_dof_joint_trajectory to array p
        for i in range(len(points)):
            point = points[i]
            x = point.transforms[0].translation.x
            y = point.transforms[0].translation.y
            z = point.transforms[0].translation.z
            time = point.time_from_start.to_sec()

            # add position to array p      
            pos = [x, y, z, time]
            p.append(pos)

        arr = p
        ans = []
        for i in range(len(p)):
            current = arr[i]
            currentTime = current[3]
            time = 0
            p2 = None 
            p1 = None
            # add acceleration to array a
            if (i == 0):
                p2 = arr[1]
                p1 = arr[0]                
            elif (i == len(arr) - 1):
                end = len(arr) - 1
                p2 = arr[end]
                p1 = arr[end - 1]
            else:
                p2 = arr[i + 1]
                p1 = arr[i - 1]
            time = p2[3] - p1[3]           
            ans.append([(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time, currentTime])
        v = ans

        arr = v
        ans = []   
        for i in range(len(v)):
            current = arr[i]
            currentTime = current[3]
            time = 0
            # add acceleration to array a
            if (i == 0):
                p2 = arr[1]
                p1 = arr[0]                
            elif (i == len(arr) - 1):
                end = len(arr) - 1
                p2 = arr[end]
                p1 = arr[end - 1]
            else:
                p2 = arr[i + 1]
                p1 = arr[i - 1]
            time = p2[3] - p1[3]            
            currentTime = v[i][3]            
            ans.append([(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time, currentTime])
        a = ans

        # create a response object
        response = MultiDOFJointTrajectory()        
        
        for i in range(len(points)):            
            response.points.append(MultiDOFJointTrajectoryPoint())
            pos = p[i]
            vel = v[i]
            acc = a[i]
            time = pos[3]

            response.points[i].transforms.append(Transform())

            response.points[i].transforms[0].translation.x = pos[0]
            response.points[i].transforms[0].translation.y = pos[1]
            response.points[i].transforms[0].translation.z = pos[2]

            response.points[i].velocities.append(Twist())
    
            response.points[i].velocities[0].linear.x = vel[0]
            response.points[i].velocities[0].linear.y = vel[1]
            response.points[i].velocities[0].linear.z = vel[2]

            response.points[i].accelerations.append(Twist())

            response.points[i].accelerations[0].linear.x = acc[0]
            response.points[i].accelerations[0].linear.y = acc[1]
            response.points[i].accelerations[0].linear.z = acc[2]

            response.points[i].time_from_start = points[i].time_from_start

        # publish the response to a topic
        self.actionPub.publish(response)

if __name__ == '__main__':
    rospy.init_node('linear_interpolator')
    server = ExecuteTrajectory()
    rospy.spin()
