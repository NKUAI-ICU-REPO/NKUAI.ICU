#! /usr/bin/python2.7
import rospy
from std_msgs.msg import Float64

class Loop:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        #self.joint1 = rospy.Publisher('/pan_controller/command',Float64,queue_size=10)
        #self.joint2=rospy.Publisher('/tilt_controller/command',Float64,queue_size=10)
        #self.joint3=rospy.Publisher('/link_controller/command',Float64,queue_size=10)
        # self.joint4=rospy.Publisher('/effort_controller/command',Float64,queue_size=10)
        self.joint5 = rospy.Publisher('/hand_controller/command',Float64,queue_size=10)
        self.pos1 = Float64()
        self.pos2=Float64()
        self.pos3=Float64()
        self.pos4=Float64()
        self.pos2=0.02
        self.pos1=0.02
        self.pos3=0.02
        self.pos4=0.03
        self.pos5=0.02


      
        while not rospy.is_shutdown():
             self.pos1  = 0.02
            #  self.joint1.publish(self.pos1)
            #  rospy.loginfo('Joint 1 succed')
            #  rospy.sleep(3)
            #  self.pos2=0.01
            #  self.joint2.publish(self.pos2)
            #  rospy.sleep(3)
            #  self.pos3=0.01
            #  self.joint3.publish(self.pos3)
            #  rospy.sleep(2)
            #  self.pos4=0.01
            #  self.joint4.publish(self.pos4)
            #  rospy.sleep(2)
             self.pos5 = 0.01
             self.joint5.publish(self.pos5)
             rospy.sleep(2)
        

            # self.pos1 = 0.25er('tilt_controller/command',Float64,queue_size=10)
        self.pos1 = Float64()
        self.pos2=Float64()
        self.pos1=0.55
        
    def cleanup(self):
        rospy.loginfo('Shutting down robot arm ...')
    

if __name__ == '__main__':
    rospy.init_node('arm')

    try:
        Loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
