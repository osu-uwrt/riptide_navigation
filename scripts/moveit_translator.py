#! /usr/bin/env python
import rospy
import actionlib
import copy

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from tf.transformations import quaternion_multiply, quaternion_slerp, quaternion_inverse
from riptide_controllers.msg import FollowTrajectoryAction, FollowTrajectoryGoal
import numpy as np
 
# returns a sample ExecuteTrajectoryActionGoal() to use for testing
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

        # self.goal = giveSampleGoal()
        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     self.execute_cb(self.goal)
        #     rate.sleep()            
          
    # returns a vector converted to the world frame
    #   vector = np array of size 3
    #   orientation np array of size 4 representing a quaternion
    #   return = np array of vector converted to the world frame
    def worldToBody(self, vector, orientation):
        vector = np.append(vector, 0)
        orientationInv = quaternion_inverse(orientation)
        newVector = quaternion_multiply(orientationInv, quaternion_multiply(vector, orientation))
        return newVector[:3]

    # returns the acceleration between two velocities
    #   v1 = Vector3() of velocity which occured first
    #   v2 = Vector3() of velocity which occured second
    #   dt = elapsed time between v1 and v2
    #   return = acceleration between v1 and v2 as a Vector3() 
    def calculate_acceleration(self, v1, v2, dt):
        v1 = np.array([v1.x, v1.y, v1.z])
        v2 = np.array([v2.x, v2.y, v2.z])
        ans = (v2 - v1) / dt
        return Vector3(x=ans[0], y=ans[1], z=ans[2])

    # returns average linear and angular velocity between two velocities
    #   v1 = Twist() of first velocity
    #   v2 = Twist() of second velocity
    #   return = Twist() of average linear and angular velocity
    def mid_velocity(self, v1, v2):
        linear = (np.array([v1.linear.x, v1.linear.y, v1.linear.z]) + 
            np.array([v2.linear.x, v2.linear.y, v2.linear.z])) / 2
        angular = (np.array([v1.angular.x, v1.angular.y, v1.angular.z]) + 
            np.array([v2.angular.x, v2.angular.y, v2.angular.z])) / 2
        return Twist(
            linear=Vector3(x=linear[0], y=linear[1], z=linear[2]),
            angular=Vector3(x=angular[0],y=angular[1],z=angular[2])
        ) 

    # returns velocities before and after the current point and the time between them
    #   i = index used to find nearest two velocities in p
    #   p = array of MultiDOFJointTrajectoryPoint()
    #   return = Twist() before i, Twist() after i, elapsed time between the twists
    def acceleration_points(self, i, p):        
        end = len(p) - 1 
        if (i == 0):
            v1 = p[0].velocities[0]
            v2 = p[1].velocities[0]
            dt = p[0].time_from_start.to_sec() - p[1].time_from_start.to_sec()        
        elif (i == end):                
            v1 = p[end - 1].velocities[0]
            v2 = p[end].velocities[0] 
            dt = p[end].time_from_start.to_sec() - p[end - 1].time_from_start.to_sec()                   
        else:
            p0 = p[i - 1]
            p1 = center = p[i] 
            p2 = p[i + 1]            
            v1 = self.mid_velocity(p0.velocities[0], p1.velocities[0])
            v2 = self.mid_velocity(p1.velocities[0], p2.velocities[0])
            dt = (p2.time_from_start.to_sec() - p0.time_from_start.to_sec()) / 2
        return v1, v2, dt

    # provides linear and angular acceleration at the current point
    #   p = array of MultiDOFJointTrajectoryPoint()
    #   i = index of the current point
    #   return = linear and angular acceleration as a Twist()
    def get_acceleration(self, i, p):                        
        v1, v2, dt = self.acceleration_points(i, p)
        return Twist(
            linear=self.calculate_acceleration(v1.linear, v2.linear, dt),
            angular=self.calculate_acceleration(v1.angular, v2.angular, dt)
        )

    # returns the velocity between two positions
    #   p1 = Vector3() at position that occured first
    #   p2 = Vector3() at position that occured second
    #   orientation = Quaternion() of current orientation
    #   dt = elapsed time between p1 and p2
    #   return = velocity between p1 and p2 as a Vector3()
    def calculate_linear_velocity(self, p1, p2, orientation, dt):
        v = (np.array([p2.x, p2.y, p2.z]) - np.array([p1.x, p1.y, p1.z])) / dt
        o = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        ans = self.worldToBody(v, o)
        return Vector3(x=ans[0], y=ans[1], z=ans[2])

    # returns the velocity needed to traverse between two orientations 
    #   q1 = Quaternion() at orientation that occurs first
    #   q2 = Quaternion() at orientation that occurs second
    #   o = the current world frame orientation in the time period
    #   dt =  elapsed time between q1 and q2
    #   return = needed velocity between q1 and q2 as a Vector3()     
    def calculate_angular_velocity(self, q1, q2, o, dt):            
        # Below code only works with small angles. Should be the case for interpolator
        # Compute dq of our error and convert to angular velocity
        # This uses the dq/dt = .5*q*w equation
        dq = np.array([q2.x, q2.y, q2.z, q2.w]) - np.array([q1.x, q1.y, q1.z, q1.w])
        ans = quaternion_multiply(quaternion_inverse(np.array([o.x, o.y, o.z, o.w])), dq)[:3] / dt    
        return Vector3(x=ans[0], y=ans[1], z=ans[2])

    # returns positions before and after the current point and the time between them
    #   i = index used to find nearest two points in array p
    #   p = array of MultiDOFJointTrajectoryPoint() 
    #   return = Transform() before i, Tranform() after i, elapsed time between the transforms
    def velocity_points(self, i, p):   
        end = len(p) - 1             
        if (i == 0):
            p1 = p[0]
            p2 = p[1]                                                
        elif (i == end):                
            p1 = p[end - 1]
            p2 = p[end]                        
        else:
            p1 = p[i - 1]
            p2 = p[i + 1] 
        dt = p2.time_from_start.to_sec() - p1.time_from_start.to_sec()
        return p1.transforms[0], p2.transforms[0], dt       

    # returns linear and angular velocity for the point at index i
    #   p = array of MultiDOFJointTrajectoryPoint()
    #   i = index of a point in p
    #   return = Twist() of linear and angular velocities
    def get_velocity(self, i, p):
        orientation = p[i].transforms[0].rotation
        p1, p2, dt = self.velocity_points(i, p)
        return Twist(
            linear=self.calculate_linear_velocity(p1.translation, p2.translation, orientation, dt),
            angular=self.calculate_angular_velocity(p1.rotation, p2.rotation, orientation, dt)          
        )

    # function to be called when a new trajectory is published
    #   goal = ExecuteTrajectoryActionGoal()
    def execute_cb(self, goal):
        ans = copy.deepcopy(goal.goal.trajectory.multi_dof_joint_trajectory)
        points = ans.points
        for i in range(len(points)):
            points[i].velocities.append(self.get_velocity(i, points))
        for i in range(len(points)):
            points[i].accelerations.append(self.get_acceleration(i, points))
        self.current_action = self.client.send_goal(FollowTrajectoryGoal(ans))  

if __name__ == '__main__':
    rospy.init_node('moveit_translator')
    server = ExecuteTrajectory()
    rospy.spin()
