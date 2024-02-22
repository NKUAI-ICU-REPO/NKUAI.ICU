#!  /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Float32,Bool
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    global yaw1
    yaw1=yaw


cmd_vel =rospy.Publisher('cmd_vel_mux/input/navi',Twist,queue_size=10)
wakeup_pub = rospy.Publisher('/voiceWakeup',String,queue_size=10)


forward_pub = rospy.Publisher('/goVoice',Bool,queue_size=5)

move_cmd = Twist()

move_cmd.linear.x =0.0
move_cmd.angular.z=0
voice_txt  = None
cnt  =1
voice_old = None
yaw1=0
def voice_control(msg):
    
    rospy.loginfo(msg.data)
    global voice_txt
    #voice_s = msg.data
    #voice_txt = voice_s.encode('utf-8').decode('unicode_escape')

    voice_txt = msg.data
    rospy.loginfo(voice_txt)
    # global voice_old
    # if voice_old  == None:
    #     voice_old = msg.data
    #     voice_txt = msg.data
    # if voice_old !=msg.data:
    #     voice_txt = msg.data
    #     voice_old = msg.data
  
    # else :
    #     rospy.loginfo('the voice control loss')

if  __name__ == '__main__':
    rospy.init_node("GoForward",anonymous=False)
  
   
    r= rospy.Rate(20)

    armPub=rospy.Publisher('/controlArm',Bool,queue_size=1)
    rospy.Subscriber('/voiceWords',String,voice_control)
    rospy.Subscriber('/odom', Odometry, callback)

    cnt=0
    voice_old =None

    old_voice = None

    while not rospy.is_shutdown():
        

        pubMsg = String()
        pubMsg.data =  'any string'
        wakeup_pub.publish(pubMsg)
        rospy.sleep(15)
        if old_voice == None:
             old_voice=voice_txt
        elif voice_txt  == old_voice:
             continue
        else:
             old_voice=voice_txt
        if voice_txt == "前进。":
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
            cmd_vel.publish(move_cmd)
            rospy.sleep(1.0) 
            move_cmd.linear.x = 0.0
            cmd_vel.publish(move_cmd)
        elif voice_txt == "左转。":
            move_cmd.angular.z = 0.5
            move_cmd.linear.x = 0.0
            cmd_vel.publish(move_cmd)
            rospy.sleep(0.5) 
            move_cmd.angular.z = 0.0
            cmd_vel.publish(move_cmd)
        elif voice_txt == "右转。":
            move_cmd.angular.z = -0.5
            move_cmd.linear.x = 0.0
            cmd_vel.publish(move_cmd)
            rospy.sleep(0.5) 
            move_cmd.angular.z = 0.0
            cmd_vel.publish(move_cmd)
        elif voice_txt == '后退。':
            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0
            cmd_vel.publish(move_cmd)
            rospy.sleep(1.0)
            move_cmd.linear.x = 0.0
            cmd_vel.publish(move_cmd)
        elif voice_txt== "跳舞。":
            echos=Bool()
            echos.data=True
            forward_pub.publish(echos)
            t=0
            while t<10:
                 
                if t==0:
                    
                    move_cmd.linear.x = 0.5
                    move_cmd.angular.z = 0.0
                    cmd_vel.publish(move_cmd)
                    rospy.sleep(1.0)
                    move_cmd.linear.x = 0.0
                    cmd_vel.publish(move_cmd)

                    move_cmd.angular.z = -0.8
                    move_cmd.linear.x = 0.0
                    cmd_vel.publish(move_cmd)
                    t = t + 1
                    rospy.sleep(1)


                else:
                    if abs(yaw1)>0.3:
                        Bmsg=Bool()
                        Bmsg.data=True
                        armPub.publish(Bmsg)
                    move_cmd.linear.x = 0.0
                    move_cmd.angular.z = -move_cmd.angular.z
                    cmd_vel.publish(move_cmd)
                    rospy.sleep(1)
                    t=t+1
            echos.data=False
            forward_pub.publish(echos)

        r.sleep()
    rospy.spin()

    # while not rospy.is_shutdown():
    #    cmd_voice  = rospy.Subscriber('kws_data',String,voice_control)
    #   # rospy.sleep()


