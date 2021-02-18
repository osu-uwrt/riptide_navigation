import sys
import rospy
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState, Constraints
from geometry_msgs.msg import Pose

if __name__=='__main__':

    # This one needs a kinematic solver I think

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    robot = RobotCommander()
    rospy.sleep(1)

    a = robot.puddles_base

    print("current pose:")
    print(a.get_current_pose())
    c = Constraints()

    waypoints = []
    waypoints.append(a.get_current_pose().pose)

    # Move forward
    wpose = Pose()
    wpose.position.x = wpose.position.x + 0.1
    waypoints.append(wpose)

    # Move down
    wpose.position.z -= 0.10
    waypoints.append(wpose)

    # Move to the side
    wpose.position.y += 0.05
    waypoints.append(wpose)

    plan, fraction = a.compute_cartesian_path(waypoints, 0.01, 0.0, path_constraints=c)
    print('Plan success percent: ', fraction)