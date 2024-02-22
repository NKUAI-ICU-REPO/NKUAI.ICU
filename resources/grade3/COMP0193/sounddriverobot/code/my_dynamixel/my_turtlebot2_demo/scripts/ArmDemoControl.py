#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64,Float32,Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from dynamixel_msgs.msg import JointState

control = 0
joint1state=None
joint3state = None
joint5state=None
def motor1_callback(msg):
    global joint1state
    joint1state = msg.current_pos
def motor3_callback(msg):
    global joint3state
    joint3state = msg.current_pos
def motor5_callback(msg):
    global joint5state
    joint5state = msg.current_pos
def voice_control(msg):
    #rospy.loginfo(msg.data)
    global voice_txt
    # voice_s = msg.data
    # voice_txt = voice_s.encode('utf-8').decode('unicode_escape')

    voice_txt = msg.data
    #rospy.loginfo(voice_txt)
def arm_control(msg):
    #rospy.loginfo(msg.data)
    global control
    control = msg.data

cmd_vel =rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
#wakeup_pub = rospy.Publisher('/voiceWakeup',String,queue_size=10)

move_cmd = Twist()

move_cmd.linear.x =0.0
move_cmd.angular.z=0
voice_txt  = None
cnt  =1
old_voice= None
class Loop:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.joint1 = rospy.Publisher('/pan_controller/command',Float64,queue_size=1)
        self.joint2=rospy.Publisher('/tilt_controller/command',Float64,queue_size=1)
        self.joint3=rospy.Publisher('/link_controller/command',Float64,queue_size=1)
        self.joint4=rospy.Publisher('/effort_controller/command',Float64,queue_size=1)
        self.joint5=rospy.Publisher('/hand_controller/command',Float64,queue_size=1)
        self.pos1 = Float64()
        self.pos2=Float64()
        self.pos3=Float64()
        self.pos4=Float64()
        self.pos5 = Float64()
        self.pos2=0.02
        self.pos1=0.02
        self.pos3=0.02
        self.pos4=0.03
        self.pos5 =0.01

    def work(self):
        while not rospy.is_shutdown():
            global old_voice
            pubMsg = String()
            pubMsg.data = 'any string'
            rospy.Subscriber('/pan_controller/state',JointState,motor1_callback)
            rospy.Subscriber('/link_controller/state',JointState,motor3_callback)
            rospy.Subscriber('/hand_controller/state',JointState,motor5_callback)
            rospy.Subscriber('/controlArm',Bool,arm_control)
            rospy.Subscriber('/voiceWords', String, voice_control)
            if old_voice == None:
                old_voice = voice_txt
            elif voice_txt == old_voice:
                continue
            else:
                old_voice = voice_txt
            if voice_txt == "招手。":
                t=0
                rospy.Subscriber('/link_controller/state',JointState,motor3_callback)
                self.pos3 = 2.2
                self.joint3.publish(self.pos3)
                rospy.sleep(5)
                while(t<2):
                    if(t==0):
                        rospy.Subscriber('/link_controller/state',JointState,motor5_callback)
                        self.pos3 = 2.2
                        self.joint3.publish(self.pos3)

                        rospy.sleep(2)
                        

                        t=t+1
                    else:
                        # self.pos1  = -self.pos1
                        # self.joint1.publish(self.pos1)
                        # self.pos3 = -self.pos3
                        # self.joint3.publish(self.pos3)
                        rospy.Subscriber('/effort_controller/state',JointState,motor5_callback)
                        self.pos3 = 2.45
                        self.joint3.publish(self.pos3)
                        rospy.sleep(2)
                        t=t+1
            if voice_txt=="摆一下手。":
                t=0
                while(t<10):
                    if(t==0):
                        rospy.Subscriber('/pan_controller/state',JointState,motor5_callback)
                        self.pos1 = 1.8
                        self.joint1.publish(self.pos1)

                        rospy.sleep(1)


                        t=t+1
                    else:
                        # self.pos1  = -self.pos1
                        # self.joint1.publish(self.pos1)
                        # self.pos3 = -self.pos3
                        # self.joint3.publish(self.pos3)
                        rospy.Subscriber('/pan_controller/state',JointState,motor5_callback)
                        self.pos1 = 2.2
                        self.joint1.publish(self.pos5)
                        rospy.sleep(1)

                        t=t+1
            if control == True:
                    rospy.Subscriber('/pan_controller/state',JointState,motor1_callback)
                    self.pos1=1.8
                    self.joint1.publish(self.pos1)
                    # rospy.sleep(3)
                    
                    self.pos1=2.2
                    self.joint1.publish(self.pos1)

            # self.pos1 = 0.25er('tilt_controller/command',Float64,queue_size=10)
        
    def cleanup(self):
        rospy.loginfo('Shutting down robot arm ...')
    

if __name__ == '__main__':
    rospy.init_node('arm')
    
    try:
        l=Loop()
        l.work()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
