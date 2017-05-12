#!/usr/bin/env python
import copy
import sys, select, termios, tty
import rospy

from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Twist #cmd_vel

import moveit_commander
import moveit_msgs.msg

robot = ""
groupArm = ""
dSpeed = 0.2            #Default Speed
incF =  0.1             #Speed increment factor
mCmd = [0.0,0.0]        #Default motion cmd: speed, angle

imgTop = []
imgBot = []

msg = """
         ~ Turtlebot Arm Control Dashboard v0.1.1 ~
               [ Reading from the keyboard! ]
------------------------------------------------------------
Arm positions:      Motion:              Speed:
z - Ready           w - Forward          q - Decrease Speed
x - Steady          s - Backward         e - Increase Speed
c - Extend          a - Rotate Left
                    d - Rotate Right
------------------------------------------------------------

CTRL-C to quit
"""

posBindings = {
        'z':(-1.0,2.0,0.6),
        'x':(0.6,2.0,-1.05),
        'c':(1.5,0.2,-0.2)
}

moveBindings = {
		'w':(dSpeed,0),
		'a':(0,dSpeed),
		'd':(0,-dSpeed),
		's':(-dSpeed,0),
        'q':('d',0),
        'e':('u',0)
	       }

def initMoveit():
    global robot, groupArm
    ## First initialize moveit_commander and rospy.
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a RobotCommander object.  This object is an interface to
    ## the robot as a whole.
    robot = moveit_commander.RobotCommander()

    ## Instantiate a MoveGroupCommander object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the left
    ## arm.  This interface can be used to plan and execute motions on the left
    ## arm.
    groupArm = moveit_commander.MoveGroupCommander("arm")


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def getCurPos(rbtGrp):
	return rbtGrp.get_current_joint_values()

def processPos(jval):
    poseName = {
        'z': "Ready",
        'x': "Steady",
        'c': "Extend"
    }
    stat = "NA"
    for c in posBindings:
        for d in range(0,len(posBindings[c])):
            #print d,c,jval[d], posBindings[c][d]
            if not round(jval[d],2) == posBindings[c][d]:
                break
            stat =c
    #print "Position:",poseName[c]
    if not stat == "NA":
        return poseName[stat]
    else:
        return "Unknown"


def setPos(rbtGrp,key):
    ga = getCurPos(rbtGrp)
    for j in range(0,len(posBindings[key])):
        ga[j] = posBindings[key][j]
    rbtGrp.set_joint_value_target(ga)
    print "Setting to Position", processPos(ga)
    plan1 = rbtGrp.plan()                 #Plan
    rbtGrp.execute(plan1)                 #execute
    rospy.sleep(1)                        #Gimme a break

def setMotion(key):
    global dSpeed
    if moveBindings[key][0] == 'd':
        dSpeed -= incF
        print "Speed:",dSpeed
    elif moveBindings[key][0] == 'u':
        dSpeed += incF
        print "Speed:",dSpeed
    else:
        for j in range(0,len(moveBindings[key])):
            mCmd[j] = moveBindings[key][j]
            twist = Twist()
            twist.linear.x = mCmd[0]; twist.linear.y = 0; twist.linear.z =0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = mCmd[1]
            pub.publish(twist)

if __name__=="__main__":
    global mCmd
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
    rospy.init_node('mainControl')

    initMoveit()

    try:
        print msg
        #print "Current position:" , getCurPos(groupArm)
        print "Current position:" ,processPos(getCurPos(groupArm))
        print "Speed:",dSpeed
        while(1):
            key = getKey()
            if key in posBindings.keys():
                setPos(groupArm, key )

            elif key in moveBindings.keys():
                setMotion(key)

            else:
                if (key == '\x03'):
                    break
                print "Invalid key"

    except Exception as e:
    	print e

    finally:
    	twist = Twist()
    	twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    	pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
