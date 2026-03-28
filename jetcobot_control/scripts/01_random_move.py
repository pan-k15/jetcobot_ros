#!/usr/bin/env python3
# coding: utf-8
import sys
import rospy
from time import sleep
import moveit_commander
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化节点
    rospy.init_node("jetcobot_random_move")
    # 初始化机械臂规划组
    jetcobot = MoveGroupCommander("arm_group")
    # 当运动规划失败后，允许重新规划
    jetcobot.allow_replanning(True)
    # 设置规划时间
    jetcobot.set_planning_time(5)
    # 尝试规划的次数
    jetcobot.set_num_planning_attempts(10)
    # 设置允许目标位置误差
    jetcobot.set_goal_position_tolerance(0.01)
    # 设置允许目标姿态误差
    jetcobot.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差
    jetcobot.set_goal_tolerance(0.01)
    # 设置最大速度
    jetcobot.set_max_velocity_scaling_factor(1.0)
    # 设置最大加速度
    jetcobot.set_max_acceleration_scaling_factor(1.0)
    while not rospy.is_shutdown():
        # 设置随机目标点
        jetcobot.set_random_target()
        # 开始运动
        jetcobot.go()
        sleep(0.5)
        # 设置"left"为目标点
        #jetcobot.set_named_target("left")
        #jetcobot.go()
        #sleep(0.5)
        # 设置"right"为目标点
        #jetcobot.set_named_target("right")
        #jetcobot.go()
        #sleep(0.5)
