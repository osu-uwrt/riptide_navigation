import rclpy
from rclpy.node import Node


class LinearInterpolator(Node):

    def __init__(self):
        super().__init__('linear_interpolation_action_server')
        self._action_server = ActionServer(
            self,
            #TODO Some sort of action goes here,
            'linear interpolator',
            self.execute_callback)

    
    def execute_callback(self, goal_handle):

    


def main(args=None):
    rclpy.init(args=args)

    linear_interpolator = LinearInterpolator()

    rclpy.spin(linear_interpolator)


if __name__ == '__main__':
    main()