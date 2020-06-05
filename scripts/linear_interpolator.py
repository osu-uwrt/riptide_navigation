import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Transform, Vector3
import math

class LinearInterpolator(Node):

    def __init__(self):
        super().__init__('linear_interpolation_action_server')
        self._action_server = ActionServer(
            self,
            #TODO Some sort of action goes here,
            'linear interpolator',
            self.execute_callback)

    
    def execute_callback(self, goal_handle):
        print("TODO")

    def create_velocity_arr(self, transforms, time_interval):
        vel_arr = []
        for i in range(0,transforms.len() - 2):
            vec3_init = transforms[i].translation
            vec3_fin = transforms[i + 1].translation
            vel = calc_velocity(vec3_init, vec3_fin, time_interval)
            vel_arr.push(vel)
        return vel_arr

    def create_accelleration_arr(self, vel_arr, time_interval):
        acc_arr = []
        for i in range(0, vel_arr.len() - 2):
            vel_init = vel_arr[i]
            vel_fin = vel_arr[i + 1]
            acc = calc_accelleration(vel_init, vel_fin, time_interval)
            acc_arr.push(acc)
        return acc_arr
            
    def calc_velocity(self, pos_init, pos_fin, time_interval):
        out = Twist()
        out.linear.x = (pos_fin.x - pos_init.x) / time_interval
        out.linear.y = (pos_fin.y - pos_init.y) / time_interval
        out.linear.z = (pos_fin.z - pos_init.z) / time_interval
        return out

    def calc_accelleration(self, vel_init, vel_fin, time_interval):
        out = Twist()
        out.linear.x = (vel_fin.linear.x - vel_init.linear.x) / time_interval
        out.linear.y = (vel_fin.linear.y - vel_init.linear.y) / time_interval
        out.linear.z = (vel_fin.linear.z - vel_init.linear.z) / time_interval
        return out

def main(args=None):
    rclpy.init(args=args)

    linear_interpolator = LinearInterpolator()

    rclpy.spin(linear_interpolator)


if __name__ == '__main__':
    main()