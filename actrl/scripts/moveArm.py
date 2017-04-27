#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from geometry_msgs.msg import Twist #cmd_vel


def moveArm():

  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('moveArm',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  groupArm = moveit_commander.MoveGroupCommander("arm")


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)
  #Send cmd_vel
  cmdvel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist)

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame (Arm): %s" % groupArm.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame (Arm): %s" % groupArm.get_end_effector_link()


  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  group_variable_arm_values = groupArm.get_current_joint_values()

  #Send cmd_vel
  msg = Twist()

  for d in range(0,10):
      #Retract arm
      group_variable_arm_values[0] = -1
      group_variable_arm_values[1] = 2
      group_variable_arm_values[2] = 0.6
      groupArm.set_joint_value_target(group_variable_arm_values)

      #Plan path first
      plan1 = groupArm.plan()
      #Then execute
      groupArm.execute(plan1)
      print "******Arm joint: ", group_variable_arm_values
      rospy.sleep(1)

      #Ready arm
      group_variable_arm_values[0] = 0.6
      group_variable_arm_values[1] = 2
      group_variable_arm_values[2] = -1.05
      groupArm.set_joint_value_target(group_variable_arm_values)

      #Plan path first
      plan2 = groupArm.plan()

      #Then execute
      groupArm.execute(plan2)
      print "******Arm joint: ", group_variable_arm_values
      rospy.sleep(1)

      msg.linear.x = 0.8
      msg.linear.y = 0
      msg.angular.z = 0
      #print msg
      cmdvel_pub.publish(msg)

      rospy.sleep(2)

      #Extend arm
      group_variable_arm_values[0] = 1.5
      group_variable_arm_values[1] = 0.2
      group_variable_arm_values[2] = -0.2
      groupArm.set_joint_value_target(group_variable_arm_values)

      #Plan path first
      plan3 = groupArm.plan()

      #Then execute
      groupArm.execute(plan3)
      print "******Arm joint: ", group_variable_arm_values

      rospy.sleep(1)

      msg.linear.x = 0
      msg.linear.y = 0
      msg.angular.z = 0
      #print msg
      cmdvel_pub.publish(msg)



  '''
  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints
  ## for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(groupRArm.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.5
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  # second move down
  wpose.position.z -= 0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))

  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = groupRArm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold

  print "============ Waiting while RVIZ displays plan3..."
  #To execute the planned action
  groupRArm.execute(plan3)
  rospy.sleep(5)


  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  collision_object = moveit_msgs.msg.CollisionObject()



  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL
  '''
  print "============ STOPPING"


if __name__=='__main__':
  try:
    moveArm()
  except rospy.ROSInterruptException:
      pass
