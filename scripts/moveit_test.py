import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    robot = RobotCommander()

    # plan to a random location
    a = robot.puddles_base

    # X, Y, Z, x, y, z, w
    r = [1, 0, -1, 1, 0, 0, 0]

    print("Starting planning")
    p = a.plan(r)
    print("Executing")
    a.execute(p)
    print("Done")