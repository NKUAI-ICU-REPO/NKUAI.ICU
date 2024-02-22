#! /usr/bin/python
import rospy
import math
import numpy as np
import actionlib
from actionlib_msgs.msg  import *
from geometry_msgs.msg import Pose,PoseWithCovarianceStamped,Point,Quaternion,Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from std_msgs.msg import String,Float32,Bool

from tf.transformations import quaternion_from_euler
import tf


ACTION_MOVE_BASE =  "move_base"
FRAME_MAP = "map"
FRAME_BASE = "base_frame"
voice_txt = None
voice_old = None
def callbackVoice(msg):
    global voice_txt
    voice_txt = msg.data
class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.clean_up)
        self.move_base = actionlib.SimpleActionClient(ACTION_MOVE_BASE,MoveBaseAction)
        rospy.loginfo('Waiting for move base action server ...')

        #self.move_base.wait_for_server(rospy.Duration(120))
        self.is_move_base_connected = False
        

        rospy.loginfo('Connected to move base server')


        self.map_robot_pose = [0,0,0]
        self.move_base_running = False
        self.blocking = True


        rospy.loginfo('Ready to go ...')


        rospy.sleep(1)
    def connect_to_move_base(self):
        while not self.is_move_base_connected:
            if self.move_base.wait_for_server(rospy.Duration(3)):
                self.is_move_base_connected = True
            else:
                rospy.loginfo("Waiting for MoveBase server to connect...")
        
        rospy.loginfo("MoveBase server connected!")

    def goto(self,target,blocking=True):
        self.blocking = blocking
        yaw = target[2] /180.0*math.pi
        quaternion =  quaternion_from_euler(0.0,0.0,yaw)
        target_pose = Pose(Point(target[0],target[1],target[2]),Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
        self.goal = MoveBaseGoal()

        rospy.loginfo('Starting navigation test ...')
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        rospy.loginfo('Going to the target ...')
        rospy.sleep(2)
        self.connect_to_move_base()
        self.goal.target_pose.pose=target_pose
        self.move_base.send_goal(self.goal)

        if blocking:
            waiting =self.move_base.wait_for_result()
            self.move_base_running = False
            if not waiting:
                rospy.logerr('Action server not avaliable')
            else:
                rospy.loginfo('Navigation result :%s'%self.move_base.get_result())
        self.is_move_base_connected = False
    def clean_up(self):
        rospy.loginfo('Shutting down  navigation  ...')


if __name__ == '__main__':
    rospy.init_node('navTurtlebot')
    #r = rospy.Rate(3)
    nav = NavToPoint()
    while not rospy.is_shutdown:
        rospy.loginfo(voice_txt)
        rospy.Subscriber('/voiceWords',String,callbackVoice)
        #listener = tf.TransformListener()
        
        if old_voice == None:
             old_voice=voice_txt
        elif voice_txt  == old_voice:
             continue
        else:
             old_voice=voice_txt
        if voice_txt == '去门口。':
            nav.goto([1,0,90])
        
        elif voice_txt =="正方形运动":
            nav.goto([3,2,0])
            rospy.sleep(2)
            nav.goto([3,5,90])
            rospy.sleep(2)
            nav.goto([5,5,-90])
            rospy.sleep(2)
            nav.goto([5,2,90])
            rospy.sleep(2)
            nav.goto([3,2,0])
        # elif voice_txt == "掉头":
        #     (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        #     nav.goto([trans[0]+2,trans[1]+2,0])
        rospy.sleep(2)
        #r.sleep()
    rospy.spin()
    

        

