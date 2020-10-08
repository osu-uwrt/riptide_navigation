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

def msgToNumpy(msg):
        if hasattr(msg, "w"):
            return np.array([msg.x, msg.y, msg.z, msg.w])
        return np.array([msg.x, msg.y, msg.z])  
 

class ExecuteTrajectory(object):

    def __init__(self):
        self.actionSub = rospy.Subscriber("/execute_trajectory/goal/", ExecuteTrajectoryActionGoal, self.execute_cb)
                
        self.client = actionlib.SimpleActionClient("puddles/follow_trajectory", FollowTrajectoryAction)
        self.client.wait_for_server()
  
    # Takes in two MultiDOFPoints and the current orientation as np array
    # Returns vector in the form [vx, vy, vz, vx ang, vy ang, vz ang]
    def get_velocity(self, point1, point2, current_orientation):
        velocity = np.zeros(6)

        if point1 == point2:
            return velocity

        p1 = point1.transforms[0]
        p2 = point2.transforms[0]
        dt = point2.time_from_start.to_sec() - point1.time_from_start.to_sec()

        linear_displacement = msgToNumpy(p2.translation) - msgToNumpy(p1.translation)
        velocity[:3] = linear_displacement / dt
        velocity[:3] = self.worldToBody(velocity[:3], current_orientation)

        # Below code only works with small angles. Should be the case for interpolator
        # Compute dq and convert to angular velocity
        # This uses the dq/dt = .5*q*w equation
        dq = msgToNumpy(p2.rotation) - msgToNumpy(p1.rotation)
        velocity[3:] = 2 * quaternion_multiply(quaternion_inverse(current_orientation), dq)[:3] / dt    
        return velocity
          
    # returns a vector converted to the world frame
    #   vector = np array of size 3
    #   orientation np array of size 4 representing a quaternion
    #   return = np array of vector converted to the world frame
    def worldToBody(self, vector, orientation):
        vector = np.append(vector, 0)
        orientationInv = quaternion_inverse(orientation)
        newVector = quaternion_multiply(orientationInv, quaternion_multiply(vector, orientation))
        return newVector[:3]

    # provides linear and angular acceleration at the current point
    #   p = array of MultiDOFJointTrajectoryPoint()
    #   i = index of the current point
    #   return = linear and angular acceleration as a Twist()
    def get_acceleration(self, i, p):  
        dt = p[1].time_from_start.to_sec() - p[0].time_from_start.to_sec()
        v1 = self.get_velocity_at_time(i - 0.5, p)
        v2 = self.get_velocity_at_time(i + 0.5, p)                      
        
        return (v2 - v1) / dt
     

    # returns linear and angular velocity for the point at index i
    #   p = array of MultiDOFJointTrajectoryPoint()
    #   i = index of a point in p
    #   return = Twist() of linear and angular velocities
    def get_velocity_at_time(self, i, p):
        end = len(p) - 1
        index1 = int(round(i - 0.6))
        index2 = int(round(i + 0.6))
        cur_index = int(round(i))
        if index1 < 0: index1 = 0
        if index2 > end: index2 = end
        if cur_index < 0: cur_index = 0
        if cur_index > end: cur_index = end
        current_orientation = msgToNumpy(p[cur_index].transforms[0].rotation)

        return self.get_velocity(p[index1], p[index2], current_orientation)

    # function to be called when a new trajectory is published
    #   goal = ExecuteTrajectoryActionGoal()
    def execute_cb(self, goal):
        rospy.loginfo("Interpolating trajectory")
        goal_trajectory = goal.goal.trajectory.multi_dof_joint_trajectory
        ans = MultiDOFJointTrajectory()
        ans.header = goal_trajectory.header
        points = ans.points

        for i in range(len(goal_trajectory.points)):
            points.append(MultiDOFJointTrajectoryPoint())
            points[i].transforms = goal_trajectory.points[i].transforms
            points[i].time_from_start = goal_trajectory.points[i].time_from_start

        for i in range(len(points)):
            velocity = self.get_velocity_at_time(i, points)
            points[i].velocities.append(Twist(Vector3(*velocity[:3]), Vector3(*velocity[3:])))

            acceleration = self.get_acceleration(i, points)
            points[i].accelerations.append(Twist(Vector3(*acceleration[:3]), Vector3(*acceleration[3:])))

        self.current_action = self.client.send_goal(FollowTrajectoryGoal(ans))  

if __name__ == '__main__':
    rospy.init_node('moveit_translator')
    server = ExecuteTrajectory()
    rospy.spin()
