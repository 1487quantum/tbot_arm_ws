#!/usr/bin/env python
import copy
import sys, select, termios, tty, os
import rospy

import tensorflow as tf
pth = os.path.split(os.path.split(os.getcwd())[0])[0] + "/sphereinception"  #Go down 2 levels from actrl/scripts -> sphereinception
sys.path.append( pth)
from InceptionProcess import InceptionProcess

import threading

from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Twist #cmd_vel

import moveit_commander
import moveit_msgs.msg

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

robot = ""
groupArm = ""
dSpeed = 0.2            #Default Speed
incF =  0.1             #Speed increment factor
mCmd = [0.0,0.0]        #Default motion cmd: speed, angle

imgTop = []
imgBot = []

numPredicted = "Starting Tensorflow..."

msg = """
         ~ Turtlebot Arm Control Dashboard v0.1.1 ~
               [ Reading from the keyboard! ]
------------------------------------------------------------
Arm positions:      Motion:              Speed:
z - Ready           w - Forward          q - Decrease Speed
x - Steady          s - Backward         e - Increase Speed
c - Extend          a - Rotate Left
                    d - Rotate Right

Actions:
v - Prod
------------------------------------------------------------

CTRL-C to quit
"""

posBindings = {
        'z':(-1.0,2.0,0.6),
        'x':(0.6,2.0,-1.05),
        'c':(1.5,0.2,-0.2),
        'v':(0.0,0.0,0.0)
}

moveBindings = {
		'w':(1,0),
		'a':(0,1),
		'd':(0,-1),
		's':(-1,0),
        'q':('d',0),
        'e':('u',0)
	       }

client = ""

camImg = []

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
    if key == 'v':
        prod()
    else:
        cp = getCurPos(rbtGrp)
        for j in range(0,len(posBindings[key])):
            cp[j] = posBindings[key][j]
        rbtGrp.set_joint_value_target(cp)
        print "Setting to Position", processPos(cp)
        plan1 = rbtGrp.plan()                 #Plan
        rbtGrp.execute(plan1)                 #execute
        rospy.sleep(1)                        #Gimme a break

def processMotion(key):
    global dSpeed
    if moveBindings[key][0] == 'd':
        if dSpeed <= incF:
            dSpeed = incF
            print "Speed at min!",dSpeed
        else:
            dSpeed -= incF
            print "Speed:",dSpeed
    elif moveBindings[key][0] == 'u':
        dSpeed += incF
        print "Speed:",dSpeed
    else:
        for j in range(0,len(moveBindings[key])):
            mCmd[j] = moveBindings[key][j]
            setTwist(mCmd[0]*dSpeed,0,0,0,0,mCmd[1]*dSpeed)

def setTwist(lx,ly,lz,ax,ay,az):
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz
    twist.angular.x = ax
    twist.angular.y = ay
    twist.angular.z = az
    pub.publish(twist)

def image_callback(msg):
    global camImg
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(msg)
        (rows,cols,channels) = img.shape

    except CvBridgeError as e:
          print(e)

    camImg = img
    writeText(img, (20,30), "Num: "+numPredicted)

    cv2.imshow("camTop", img)
    cv2.waitKey(3)

def writeText(img_src, pt_loc, text):
    try:
      font = cv2.FONT_HERSHEY_SIMPLEX
      #img src, text, pt location, font, textsize, color, thickness, antialiasing
      cv2.putText(img_src, text , pt_loc, font, 0.7,(0,0,255),1,cv2.LINE_AA)
    except Exception,e:
      print e

def prod():
    setPos(groupArm, posBindings.keys()[2])
    setPos(groupArm, posBindings.keys()[1])
    setPos(groupArm, posBindings.keys()[2])
    print "Done!"

def recog():
    imager = InceptionProcess()
    labels = imager.initRecog()
    with tf.Session() as sess:
        softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
        while(1):
            global camImg, numPredicted
            imgSrc = camImg
            cv2.imwrite('a.jpg', imgSrc, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            rospy.sleep(0.2)
            a = imager.predictDigit(sess,softmax_tensor, labels, os.getcwd()+"/a.jpg")
            numPredicted = labels[a.index(max(a))] + " (" + str(round(max(a),4)) + ")"
            #print numPredicted
            rospy.sleep(0.1)

    print "recog() process finished!"

if __name__=="__main__":
    global mCmd
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)
    image_sub = rospy.Subscriber('cameraB/rgb/image_raw', Image, image_callback)
    rospy.init_node('mainControl')

    initMoveit()
    setPos(groupArm, posBindings.keys()[0])

    try:
        print msg
        #print "Current position:" , getCurPos(groupArm)
        print "Current position:" ,processPos(getCurPos(groupArm))
        print "Speed:",dSpeed

        print "==== Ready for input ==== "

        t = threading.Thread(target=recog)
        t.daemon = True
        t.start()

        while(1):
            key = getKey()
            if key in posBindings.keys():
                setPos(groupArm, key)

            elif key in moveBindings.keys():
                processMotion(key)

            else:
                if (key == '\x03'):
                    break
                print "Invalid key"

    except Exception as e:
    	print e

    finally:
    	setTwist(0,0,0,0,0,0)
        sys.exit()

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
