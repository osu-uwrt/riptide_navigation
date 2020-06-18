#! /usr/bin/env python
import rospy
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from geometry_msgs.msg import Transform, Twist
from tf.transformations import quaternion_multiply, quaternion_slerp, quaternion_inverse
import numpy as np
 

class ExecuteTrajectory(object):

    def calculateLinearVelocity(self, p1, p2, currentOrientation, dt):
        # take derivate of position to get velocity
        ans = []
            ans.append([(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time])
    return ans

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
 
 def worldToBody(self, positions, orientations):
    orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
    orientationInv = quaternion_inverse(orientation)
    vector.append(0)
    newVector = quaternion_multiply(orientation, quaternion_multiply(vector, orientationInv))[:3]
    return newVector

def __init__(self):
    self.actionSub = rospy.Subscriber("/execute_trajectory/goal/", ExecuteTrajectoryActionGoal, self.execute_cb)
    self.actionPub = rospy.Publisher("topic_name", MultiDOFJointTrajectory, queue_size=1)

def execute_cb(self, goal):        

    # array of each point as a move_it msg
    points = goal.goal.trajectory.multi_dof_joint_trajectory.points

    # arrays for position, velocity, acceleration, and time
    p = []
    v = []
    a = []
    t = []

    angP = []
    angV = []
    angA = []    

    # extract position, rotation and time from multi_dof_joint_trajectory
    for i in range(len(points)):
        point = points[i]
        x = 
        y = 
        z = point.transforms[0].translation.z
        t = point.time_from_start.to_sec()

        # add position to array p           
        p.append([
            point.transforms[0].translation.x, 
            point.transforms[0].translation.y, 
            point.transforms[0].translation.z
        ])

        # add rotation quaternion to array r
        angP.append([point.transforms[0].rotation])

        # add time at the point to array t
        t.append(time)


    # take derivate of position to get velocity relative to the robot body
    for i in range(len(points)):
        currentTime = times[i]
        currentOrientation = None           
        p2 = None 
        p1 = None
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
        v.append(calculateLinearVelocity(p1, p2, currentOrientation, dt))
        angV.append(calculateAngularVelocity(q1, q2, currentOrientation, dt))



    # take derivate of velocity to get acceleration relative to the robot body
    a = calculateLinearAcceleration(v,)
    arr = v
    ans = []   
    for i in range(len(v)):
        current = arr[i]
        currentTime = current[3]
        time = 0
        # front of array            
        if (i == 0):
            p2 = arr[1]
            p1 = arr[0]
        # end of array                
        elif (i == len(arr) - 1):
            end = len(arr) - 1
            p2 = arr[end]
            p1 = arr[end - 1]
        # normal case
        else:
            p2 = arr[i + 1]
            p1 = arr[i - 1]
        # add current acceleration to array
        time = p2[3] - p1[3]            
        currentTime = v[i][3]            
        ans.append([(p2[0] - p1[0])/time, (p2[1] - p1[1])/time, (p2[2] - p1[2])/time, currentTime])
    a = ans

    # create an object to store the response
    response = MultiDOFJointTrajectory()        
    
    # build each point in the response 
    for i in range(len(points)):            
        response.points.append(MultiDOFJointTrajectoryPoint())
        pos = p[i]
        vel = v[i]
        acc = a[i]
        time = pos[3]

        # add x y and z position
        response.points[i].transforms.append(Transform())

        response.points[i].transforms[0].translation.x = pos[0]
        response.points[i].transforms[0].translation.y = pos[1]
        response.points[i].transforms[0].translation.z = pos[2]

        # add x y and z velocity
        response.points[i].velocities.append(Twist())

        response.points[i].velocities[0].linear.x = vel[0]
        response.points[i].velocities[0].linear.y = vel[1]
        response.points[i].velocities[0].linear.z = vel[2]

        # add x y and z acceleration
        response.points[i].accelerations.append(Twist())

        response.points[i].accelerations[0].linear.x = acc[0]
        response.points[i].accelerations[0].linear.y = acc[1]
        response.points[i].accelerations[0].linear.z = acc[2]

        # add time since start of execution to each point
        response.points[i].time_from_start = points[i].time_from_start

    # publish the response to a topic
    self.actionPub.publish(response)

if __name__ == '__main__':
    rospy.init_node('linear_interpolator')
    server = ExecuteTrajectory()
    rospy.spin()
