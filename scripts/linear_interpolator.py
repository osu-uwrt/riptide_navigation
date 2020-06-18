#! /usr/bin/env python
import rospy
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_multiply, quaternion_slerp, quaternion_inverse
import numpy as np
 

class ExecuteTrajectory(object):

    # q1 is first orientation, q2 is second orientation. Dt is time between them
    # Will return angular velocity to traverse between the two points in body frame. 
    # Must also pass currentOrientation. This is the orientation for the time period
    def calculateAngularVelocity(self, q1, q2, currentOrientation, dt):
        # Convert to tf quaternion format
        q1 = [q1.x, q1.y, q1.z, q1.w]
        q2 = [q2.x, q2.y, q2.z, q2.w]
        currentOrientation = [currentOrientation.x, currentOrientation.y, currentOrientation.z, currentOrientation.w]

        # Below code only works with small angles. Should be the case for interpolator
        # Compute dq of our error and convert to angular velocity
        # This uses the dq/dt = .5*q*w equation
        dq = np.array(q2) - np.array(q1)
        angularVel = quaternion_multiply(quaternion_inverse(currentOrientation), dq)[:3] / dt
        return angularVel
    
    def worldToBody(self, vector, orientation):
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        orientationInv = quaternion_inverse(orientation)
        vector.append(0)
        newVector = quaternion_multiply(orientation, quaternion_multiply(vector, orientationInv))[:3]
        return newVector

    def calculateLinearVelocity(self, p1, p2, currentOrientation, dt):
        # take derivate of position to get world velocity and convert to body velocity
        ans = [(p2[0] - p1[0])/dt, (p2[1] - p1[1])/dt, (p2[2] - p1[2])/dt]
        return self.worldToBody(ans, currentOrientation)

    def __init__(self):
        self.actionSub = rospy.Subscriber("/execute_trajectory/goal/", ExecuteTrajectoryActionGoal, self.execute_cb)
        self.actionPub = rospy.Publisher("topic_name", MultiDOFJointTrajectory, queue_size=1)

    def execute_cb(self, goal):        

        # array of each point as a move_it msg
        points = goal.goal.trajectory.multi_dof_joint_trajectory.points

        # arrays for linear and angular position, velocity, and acceleration
        p = []
        v = []
        a = []        

        angP = []
        angV = []
        angA = []    

        # array for time
        t = []

        # extract position, rotation and time from multi_dof_joint_trajectory
        for i in range(len(points)):
            point = points[i]
            
            # add position to array p           
            p.append([
                point.transforms[0].translation.x, 
                point.transforms[0].translation.y, 
                point.transforms[0].translation.z
            ])

            # add rotation quaternion to array r
            angP.append(point.transforms[0].rotation)

            # add time at the point to array t
            t.append(point.time_from_start.to_sec())


        # take derivate of position to get velocity relative to the robot body
        for i in range(len(points)):
            currentOrientation = None           
            p2 = None 
            p1 = None
            q2 = None
            q1 = None
            dt = 0
            # front of array
            if (i == 0):
                p2 = p[1]
                p1 = p[0]

                q2 = angP[1]
                q1 = angP[0]
                currentOrientation = q1

                dt = t[1] - t[0]        
            # end of array
            elif (i == len(p) - 1):
                end = len(p) - 1
                p2 = p[end]
                p1 = p[end - 1]

                q2 = angP[end]
                q1 = angP[end - 1]
                currentOrientation = q2

                dt = t[end] - t[end - 1]     
            # normal case
            else:
                p2 = p[i + 1]
                p1 = p[i - 1]

                q2 = angP[i + 1]
                q1 = angP[i - 1]
                currentOrientation = angP[i]   

                dt = t[i + 1] - t[i - 1]

            # add linear and angular velocity at the current point to v and angV arrays
            v.append(self.calculateLinearVelocity(p1, p2, currentOrientation, dt))
            angV.append(self.calculateAngularVelocity(q1, q2, currentOrientation, dt))



        # take derivate of velocity to get acceleration relative to the robot body
        for i in range(len(points)):
            currentOrientation = None           
            p2 = None 
            p1 = None
            q2 = None
            q1 = None
            dt = 0
            # front of array
            if (i == 0):
                p2 = v[1]
                p1 = v[0]

                q2 = angV[1]
                q1 = angV[0]
                currentOrientation = q1

                dt = t[1] - t[0]        
            # end of array
            elif (i == len(v) - 1):
                end = len(v) - 1
                p2 = v[end]
                p1 = v[end - 1]

                q2 = angV[end]
                q1 = angV[end - 1]
                currentOrientation = q2

                dt = t[end] - t[end - 1]     
            # normal case
            else:
                p2 = (p[i + 1] + p[i])/2
                p1 = (p[i - 1] + p[i])/2

                q2 = (angP[i + 1] + angP[i])/2
                q1 = (angP[i - 1] + angP[i])/2
                currentOrientation = angP[i]   

                dt = (t[i + 1] + t[i])/2 - (t[i - 1] + t[i])/2

            # add linear and angular acceleration at the current point to a and angA arrays
            a.append(self.calculateLinearVelocity(p1, p2, currentOrientation, dt))
            angA.append(self.calculateAngularVelocity(q1, q2, currentOrientation, dt))

        # create an object to store the response
        response = MultiDOFJointTrajectory()        
        
        # build each point in the response 
        for i in range(len(points)):            
            response.points.append(MultiDOFJointTrajectoryPoint())
            pos = p[i]
            vel = v[i]
            acc = a[i]
            angVel = angV[i]
            angAcc = angA[i]

            point = response.points[i]
            # add x y z position and angular rotation quaternion
            response.points[i].transforms.append(Transform())

            point.transforms[0].translation.x = pos[0]
            point.transforms[0].translation.y = pos[1]
            point.transforms[0].translation.z = pos[2]

            point.transforms[0].rotation = angP[i]

            # add x y z velocity and angular velocity
            point.velocities.append(Twist())

            point.velocities[0].linear.x = vel[0]
            point.velocities[0].linear.y = vel[1]
            point.velocities[0].linear.z = vel[2]

            point.velocities[0].angular.x = angVel[0]
            point.velocities[0].angular.y = angVel[1]
            point.velocities[0].angular.z = angVel[2]

            # add x y and z acceleration and angular acceleration
            point.accelerations.append(Twist())

            point.accelerations[0].linear.x = acc[0]
            point.accelerations[0].linear.y = acc[1]
            point.accelerations[0].linear.z = acc[2]

            point.accelerations[0].angular.x = angAcc[0]
            point.accelerations[0].angular.y = angAcc[1]
            point.accelerations[0].angular.z = angAcc[2]

            # add time since start of execution to each point
        point.time_from_start = points[i].time_from_start

        # publish the response to a topic
        self.actionPub.publish(response)

if __name__ == '__main__':
    rospy.init_node('linear_interpolator')
    server = ExecuteTrajectory()
    rospy.spin()
