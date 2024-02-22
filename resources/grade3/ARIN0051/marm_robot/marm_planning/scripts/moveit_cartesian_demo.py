#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose,PoseStamped
from copy import deepcopy
import numpy as np

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
        
        # 是否需要使用笛卡尔空间的运动规划
        self.cartesian = rospy.get_param('~cartesian', True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('arm')
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        self.arm.set_max_acceleration_scaling_factor(0.25)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
    #@staticmethod                                   
    def PlanningExample(self):
        # 控制机械臂运动到之前设置的“forward”姿态
        self.arm.set_named_target('forward')
        self.arm.go()
        
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        if self.cartesian:
            waypoints.append(start_pose)
            
        # 设置第二个路点数据，并加入路点列表
        # 第二个路点需要向后运动0.2米，向右运动0.2米
        wpose = deepcopy(start_pose)
        wpose.position.x -= 0.2
        wpose.position.y -= 0.2

        if self.cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            self.arm.set_pose_target(wpose)
            self.arm.go()
            rospy.sleep(1)
         
        # 设置第三个路点数据，并加入路点列表
        wpose.position.x += 0.05
        wpose.position.y += 0.15
        wpose.position.z -= 0.15
          
        if self.cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            self.arm.set_pose_target(wpose)
            self.arm.go()
            rospy.sleep(1)
        
        # 设置第四个路点数据，回到初始位置，并加入路点列表
        if self.cartesian:
            waypoints.append(deepcopy(start_pose))
        else:
            self.arm.set_pose_target(start_pose)
            self.arm.go()
            rospy.sleep(1)
            
        if self.cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            
            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
     
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses，路点列表
                                        0.01,        # eef_step，终端步进值
                                        0.0,         # jump_threshold，跳跃阈值
                                        True)        # avoid_collisions，避障规划
                
                # 尝试次数累加
                attempts += 1
                
                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # 控制机械臂回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    def PlanningCircle(self):
        self.arm.set_named_target('home')
        self.arm.go()


        rospy.sleep(1)


        # target_pose = PoseStamped()
        # target_pose.header.frame_id = self.reference_frame
        # target_pose.header.stamp = rospy.Time.now()     
        # target_pose.pose.position.x = 0.331958
        # target_pose.pose.position.y = 0.0
        # target_pose.pose.position.z = 0.307887
        
        
        # # 设置机械臂终端运动的目标位姿
        # self.arm.set_pose_target(target_pose, self.end_effector_link)
        # self.arm.go()

        # 初始化路点列表


        self.arm.set_named_target('forward')
        self.arm.go()


        rospy.sleep(1)

        start_pose  = self.arm.get_current_pose(self.end_effector_link)
        waypoints = []

        # if self.cartesian:
        #     waypoints.append(start_pose.pose)
            
        # 设置微调的缓冲路点数据，并加入路点列表
        target_pose = deepcopy(start_pose)
        target_pose.pose.position.x -= 0.02
        target_pose.pose.position.y-=0.02
        target_pose.pose.position.z+=0.04
        self.arm.set_pose_target(target_pose)
        self.arm.go()
        rospy.sleep(1)
        # target_pose.pose.orientation.x = -0.482974
        # target_pose.pose.orientation.y = 0.517043
        # target_pose.pose.orientation.z = -0.504953
        # target_pose.pose.orientation.w = -0.494393

       
                
        # 将圆弧上的路径点加入列表
        if self.cartesian:
            waypoints.append(target_pose.pose)
        # else:
        #     self.arm.set_pose_target(target_pose)
        #     self.arm.go()
        radius = 0.04  #半径
        centerA = target_pose.pose.position.y 
        centerB = target_pose.pose.position.z-radius #注意yz平面为正视面，第一次要设置z的最高点

        rospy.loginfo("the first pos center z is %f",centerB)

        centerAt = centerA-2*radius  # 第二次在y方向的平移
        centerBt = centerB 
        
        rate_a =1.0
        rate_b =1.0
        cnt =0
        for th in np.arange(np.pi/2,3*np.pi, 0.02):
            target_pose.pose.position.y= centerA + rate_a*radius * np.cos(th)
            target_pose.pose.position.z = centerB + rate_b*radius * np.sin(th)
            wpose = deepcopy(target_pose.pose)
            if self.cartesian:
                waypoints.append(deepcopy(wpose))
            else:
                self.arm.set_pose_target(wpose)
                self.arm.go()

            cnt+=1
            if cnt%100==0:
                rospy.loginfo('the Circle move to%d',cnt)

        # cnt =0

       

        if self.cartesian:
            fraction = 0.0   #路径规划覆盖率                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            
            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
     
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses，路点列表
                                        0.01,        # eef_step，终端步进值
                                        0.0,         # jump_threshold，跳跃阈值
                                        True)        # avoid_collisions，避障规划
                
                # 尝试次数累加
                attempts += 1
                
                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("First Circle Path computed successfully. Moving the arm.")
                self.arm.execute(plan)
                rospy.loginfo("First Circle Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("First Circle Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
        
        #rospy.sleep(1)
        rospy.loginfo("the first pos center x is %f",centerAt)
        waypoints= []                                               
        if self.cartesian:
            waypoints.append(target_pose.pose)

        for th in np.arange(0,2*np.pi, 0.02):
            target_pose.pose.position.y= centerAt + rate_a*radius * np.cos(th)
            target_pose.pose.position.z = centerBt+ rate_b*radius * np.sin(th)
            wpose = deepcopy(target_pose.pose)
            if self.cartesian:
                waypoints.append(deepcopy(wpose))
            else:
                self.arm.set_pose_target(wpose)
                self.arm.go()

            cnt+=1
            if cnt%100==0:
                rospy.loginfo('the Second Circle move to%d',cnt)


        if self.cartesian:
            fraction = 0.0   #路径规划覆盖率
            maxtries = 100   #最大尝试规划次数
            attempts = 0     #已经尝试规划次数
            
            # 设置机器臂当前的状态作为运动初始状态
            self.arm.set_start_state_to_current_state()
     
            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self.arm.compute_cartesian_path (
                                        waypoints,   # waypoint poses，路点列表
                                        0.01,        # eef_step，终端步进值
                                        0.0,         # jump_threshold，跳跃阈值
                                        True)        # avoid_collisions，避障规划
                
                # 尝试次数累加
                attempts += 1
                
                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                         
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Second Circle Path computed successfully. Moving the arm.")
                self.arm.execute(plan)
                rospy.loginfo("Second Circle Path execution complete.")
            # 如果路径规划失败，则打印失败信息
            else:
                rospy.loginfo("Second Circle Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
        # 控制机械臂回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
		


if __name__ == "__main__":
    try:
        marm_PlanningModel=MoveItCartesianDemo()
        #marm_PlanningModel.PlanningExample()
        marm_PlanningModel.PlanningCircle()
    except rospy.ROSInterruptException:
        pass
