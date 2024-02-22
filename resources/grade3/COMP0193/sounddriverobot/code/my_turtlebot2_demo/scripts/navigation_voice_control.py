#!  /usr/bin/env python
# TurtleBot must have minimal.launch & amcl_demo.launch
# running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import rospy
import roslib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import time
import tf

have_packages_on_bot = False
waiting_for_input = True 
back_to_origin = True
cannot_move_until_order0_is_called = False 
voice_txt=None
cnt=False
listener = None
class GoToPose():


    def __init__(self):

        self.goal_sent = False

		# What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
		
	  # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

        #monitor kobuki's button events
        

    def goto(self, pos, quat):
        #if someone is currently putting or taking away packages, don't move!
        if(cannot_move_until_order0_is_called):
            rospy.loginfo("Waiting for voice order0 to be called.")
            time.sleep(2)
            return True
		
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result
	

			
    def shutdown(self):
        #if self.goal_sent:
        #self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)
def voice_control(msg):
        #rospy.loginfo(msg.data)
        global voice_txt
        global cnt
        voice_txt = msg.data
        global waiting_for_input
        global cannot_move_until_order0_is_called
        if ( voice_txt == "启动导航。") and cnt==False :
            #rospy.loginfo("Order0 has been called.")
            cannot_move_until_order0_is_called = False
            waiting_for_input = True
            cnt=True
if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        listener=tf.TransformListener()
        quaternion_m={'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        # Customize the following values so they are appropriate for your location
        position_a1 = {'x':2.13 , 'y' : 1.33}
        quaternion_a1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_a2 = {'x': 1.05, 'y' : 1.33}
        quaternion_a2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_b1 = {'x': 1.05, 'y' : 2.82}
        quaternion_b1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_b2 = {'x': 2.13, 'y' : 2.82}
        quaternion_b2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_o = {'x': 0.071, 'y': 0.006}
        quaternion_o = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}
        position_p1 = {'x': 2.0, 'y' : 2.00}
        quaternion_p1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_p2 = {'x': 2.75, 'y' : 2.55}
        quaternion_p2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_p3 = {'x': 2.83, 'y' : 1.05}
        quaternion_p3 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_p4 = {'x': 1.17, 'y' : 0.95}
        quaternion_p4 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_p5 = {'x': 0.25, 'y' : 3.45}
        quaternion_p5 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        position_p0 = {'x': 3.5, 'y': 2.0}
        quaternion_p0 = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}

        while not rospy.is_shutdown():	
            rospy.Subscriber("/voiceWords",String,voice_control)
            if waiting_for_input:
                if not have_packages_on_bot:
                    order = voice_txt
                    #rospy.loginfo("Go to %s", order)
                    if order == '第1点。':
                        waiting_for_input = True
                        success = navigator.goto(position_a1, quaternion_a1)
                        if success:
                            #print("*Yea, reached A1! If the packages are ready, call Order0 to allow GdBot to continue.")
                            have_packages_on_bot = True
                            #print("*Yea, reached A1! If the packages are ready, call Order0 to allow GdBot to continue.")
                            have_packages_on_bot = True
                            back_to_origin = False
							#tell TurtleBot not to move until the customer calles Ord

                        else:
                            #rospy.loginfo("The base failed to reach the desired pose, send the bot back.")
                            back_to_origin = navigator.goto(position_o, quaternion_o)
                            if back_to_origin:
                                #print('*The GdBot is at origin place.')
                                pass
						# Sleep to give the last log messages time to be sent
                        rospy.sleep(1)

                    elif order == '第2点。':
                        waiting_for_input = True
                        success = navigator.goto(position_a2, quaternion_a2)
                        if success:
                            print('*Yea, reached A2! If the packages are ready, call order0 to allow GdBot to continue.')
                            have_packages_on_bot = True
                            back_to_origin = False
							#tell TurtleBot not to move until the customer calles oder0
                            cannot_move_until_order0_is_called = True

                        else:
                            #rospy.loginfo("The base failed to reach the desired pose, send the bot back.")
                            back_to_origin = navigator.goto(position_o, quaternion_o)
                            if back_to_origin:
                                print('*The GdBot is at origin place.')
                            rospy.sleep(1)
                    elif order == '给我画个矩形。':
                        waiting_for_input = True
                        suc1=navigator.goto(position_a1, quaternion_a1)
                        rospy.sleep(3)
                        
                        suc2=navigator.goto(position_a2, quaternion_a2)
                        rospy.sleep(3)    
                        navigator.goto(position_b1,quaternion_b1)
                        rospy.sleep(3)
                        success = navigator.goto(position_b2, quaternion_b2)
                        if success:
                            #print("*Yea,drawing a rectangle has finished.")
                            have_packages_on_bot = True
                            back_to_origin = False
							#tell TurtleBot not to move until the customer calles Order0  
                            cannot_move_until_order0_is_called = True

                        else:
                            #rospy.loginfo("The base failed to reach the desired pose, send the bot back.")
                            back_to_origin = navigator.goto(position_o, quaternion_o)
                            if back_to_origin:
                                print('*The GdBot is at origin place.')
						# Sleep to give the last log messages time to be sent
                        rospy.sleep(1)
                    elif order == '给我画个五芒星。':
                        waiting_for_input = True
                        navigator.goto(position_p1, quaternion_p1)
                        rospy.sleep(3)
                        navigator.goto(position_p2, quaternion_p2)
                        rospy.sleep(3)    
                        navigator.goto(position_p3,quaternion_p3)
                        rospy.sleep(3)
                        navigator.goto(position_p4, quaternion_p4)
                        rospy.sleep(3)    
                        navigator.goto(position_p5,quaternion_p5)
                        rospy.sleep(3)
                        success = navigator.goto(position_p0, quaternion_p0)
                        if success:
                            print('*Yea,drawing a pentacle has finished')
                            have_packages_on_bot = True
                            back_to_origin = False
							#tell TurtleBot not to move until the customer calles oder0
                            cannot_move_until_order0_is_called = True	
                    elif order == "给我画个三角形。":
                        waiting_for_input=True
                        (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                        rospy.loginfo(trans[0])
                        rospy.loginfo(trans[1])
                        trans_dict={'x':abs(trans[0])+2,'y':abs(trans[1])+2}
                        navigator.goto(trans_dict,quaternion_m)	
                        (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                        trans_dict={'x':abs(trans[0])+2,'y':abs(trans[1])-2}
                        navigator.goto(trans_dict,quaternion_m)
                    elif order == '去中心点。':
                        if back_to_origin:
                            print('*The GdBot has been at the origin place.')
                            pass
                        else:
                            #rospy.loginfo("Begin sending the GdBot back.")
                            back_to_origin = navigator.goto(position_o, quaternion_o)
                            if back_to_origin:
                                print('*The GdBot is at origin place.')
                        rospy.sleep(1)
						
                    else:
                        #print('*Please choose from A1 and A2 or send the GdBot back(enter back):')
                        pass
                    #continue
                # rospy.loginfo("Package "+str(have_packages_on_bot))
                if have_packages_on_bot:
                    order = voice_txt
                    if order == '第3点。':
                        waiting_for_input = True
                        success = navigator.goto(position_b1, quaternion_b1)
                        if success:
                            #print("*Yea, reached B1! If the packages have been taken away, call order0 to allow GdBot to continue.")
                            have_packages_on_bot = False
                            back_to_origin = False
							#tell TurtleBot not to move until the customer calles order0
                            cannot_move_until_order0_is_called = True

                        else:
                            rospy.loginfo("The base failed to reach the desired pose")
                            back_to_origin = navigator.goto(position_o, quaternion_o)
                            if back_to_origin:
                                #print('*The GdBot is at origin place.')
                                pass

                        rospy.sleep(1)
                    elif order == '第4点。':
                        waiting_for_input = True
                        success = navigator.goto(position_b2, quaternion_b2)
                        if success:
                            #print("*Yea, reached B2! If the packages have been taken away, call order0 to allow GdBot to continue.")
                            have_packages_on_bot = False
                            back_to_origin = False
							#tell TurtleBot not to move until the customer calles order0
                            cannot_move_until_order0_is_called = True

                        else:
                            #rospy.loginfo("The base failed to reach the desired pose")
                            back_to_origin = navigator.goto(position_o, quaternion_o)
                            if back_to_origin:
                                pass
                                #print('*The GdBot is at origin place.')
                        rospy.sleep(1)
                    else:
                        #print('*Please choose from B1 and B2:')
                        pass
                    continue
                rospy.sleep(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")