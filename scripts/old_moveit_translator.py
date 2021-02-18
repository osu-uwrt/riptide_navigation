#! /usr/bin/env python
import rospy
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from tf.transformations import quaternion_multiply, quaternion_slerp, quaternion_inverse
from riptide_controllers.msg import FollowTrajectoryAction, FollowTrajectoryGoal
import numpy as np
 
# returns a sample trajectory to use for testing
def giveSampleGoal():   
    goal = ExecuteTrajectoryActionGoal()            
    ans = MultiDOFJointTrajectory()
    speed = 0.5
    duration = 20
    dt = 0.1    

    for i in range(int(duration / dt)):
        point = MultiDOFJointTrajectoryPoint()
        point.transforms.append(Transform(Vector3(-speed * dt * i, 0, -1), Quaternion(0, 0, 0, 1)))
        point.time_from_start = rospy.Duration(i * dt)

        ans.points.append(point)
    
    goal.goal.trajectory.multi_dof_joint_trajectory = ans
    return goal

class ExecuteTrajectory(object):

    def __init__(self):
        self.actionSub = rospy.Subscriber("/execute_trajectory/goal/", ExecuteTrajectoryActionGoal, self.execute_cb)
                
        self.client = actionlib.SimpleActionClient("puddles/follow_trajectory", FollowTrajectoryAction)
        self.client.wait_for_server()
          
    

    # q1 is first orientation, q2 is second orientation. Dt is time between them
    # Will return angular velocity to traverse between the two points in body frame. 
    # Must also pass currentOrientation. This is the orientation for the time period
    def calculateAngularVelocity(self, q1, q2, currentOrientation, dt):            
        # Below code only works with small angles. Should be the case for interpolator
        # Compute dq of our error and convert to angular velocity
        # This uses the dq/dt = .5*q*w equation
        dq = np.array(q2) - np.array(q1)
        angularVel = quaternion_multiply(quaternion_inverse(currentOrientation), dq)[:3] / dt
        return angularVel

    def worldToBody(self, vector, orientation):
        vector = np.append(vector, 0)
        orientationInv = quaternion_inverse(orientation)
        newVector = quaternion_multiply(orientationInv, quaternion_multiply(vector, orientation))
        return newVector[:3]

    def calculateLinearVelocity(self, p1, p2, currentOrientation, dt):
        p1 = np.array(p1)
        p2 = np.array(p2)
        # take derivate of position to get world velocity and convert to body velocity        
        ans = (p2 - p1) / dt
        return self.worldToBody(ans, currentOrientation)

    def calculateAcceleration(self, v1, v2, dt):
        v1 = np.array(v1)
        v2 = np.array(v2)
        ans = (v2 - v1) / dt
        return ans    

    def execute_cb(self, goal):       
        rospy.loginfo("Translating trajectory") 

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

        # extract position, rotation and time from each point in points
        for i in range(len(points)):
            point = points[i]
            
            # add position to array p           
            p.append([
                point.transforms[0].translation.x, 
                point.transforms[0].translation.y, 
                point.transforms[0].translation.z
            ])

            # add rotation to array angP
            angP.append([
                point.transforms[0].rotation.x,
                point.transforms[0].rotation.y,
                point.transforms[0].rotation.z,
                point.transforms[0].rotation.w
            ])

            # add time at the point to array t
            t.append(point.time_from_start.to_sec())


        # take derivate of position to get velocity relative to the robot body
        for i in range(len(points)):
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
                p2 = (v[i + 1] + v[i])/2
                p1 = (v[i - 1] + v[i])/2

                q2 = (angV[i + 1] + angV[i])/2
                q1 = (angV[i - 1] + angV[i])/2
                currentOrientation = angP[i]   

                dt = (t[i + 1] + t[i])/2 - (t[i - 1] + t[i])/2

            # add linear and angular acceleration at the current point to a and angA arrays
            a.append(self.calculateAcceleration(p1, p2, dt))
            angA.append(self.calculateAcceleration(q1, q2, dt))

        # create an object to store the response
        response = MultiDOFJointTrajectory()        
        
        # build each point in the response 
        for i in range(len(points)):                        
            pos = p[i]
            vel = v[i]
            acc = a[i]
            angPos = angP[i]
            angVel = angV[i]
            angAcc = angA[i]

            point = MultiDOFJointTrajectoryPoint()
            # add x y z position and angular rotation quaternion
            point.transforms.append(Transform())
            
            point.transforms[0].translation = Vector3()
            point.transforms[0].translation.x = pos[0]
            point.transforms[0].translation.y = pos[1]
            point.transforms[0].translation.z = pos[2]
            
            point.transforms[0].rotation = Quaternion()
            point.transforms[0].rotation.x = angPos[0]
            point.transforms[0].rotation.y = angPos[1]
            point.transforms[0].rotation.z = angPos[2]
            point.transforms[0].rotation.w = angPos[3]

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

            response.points.append(point)

        # send goal to start the action
        self.current_action = self.client.send_goal(FollowTrajectoryGoal(response))   

if __name__ == '__main__':
    rospy.init_node('moveit_translator')
    server = ExecuteTrajectory()
    rospy.spin()
