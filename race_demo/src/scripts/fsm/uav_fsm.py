#!/usr/bin/env python3
import rospy
import numpy as np
from enum import Enum

from race_demo.msg import UserCmdRequest
from race_demo.msg import DroneWayPoint
from race_demo.msg import DronePhysicalStatus


class UAVState(Enum):
    READY = 0
    ON_CAR = 1
    FLYING_GO = 2
    FLYING_BACK = 3
    WAITING_GO = 4
    WAITING_BACK = 5

class UAV_FSM:
    # 初始化
    def __init__(self, config):
        self.cmd_pub = rospy.Publisher('/cmd_exec', UserCmdRequest, queue_size=10000)
        self.state = UAVState.READY
        self.uav_sn = config["sn"]
        self.peer_id = config["peer_id"]
        self.task_guid = config["task_guid"]
        self.getUAVStatus = config["get_uav_status"]
        self.uav_status = self.getUAVStatus(self.uav_sn)
        self.sleep_time = 1
        self.cmd_list = []
    
    # 检测位置到达的函数
    def desPosReached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

    # 无人机状态转换的函数
    def stateChange(self, new_state):
        old_state = self.state
        self.state = new_state
        rospy.loginfo("UAV %s state changed from %s to %s" % (self.uav_sn, old_state, new_state))

    # 飞机航线飞行函数
    def flyOneRoute(self, route, speed):
        # 等待无人机就绪
        while self.uav_status.drone_work_state != DronePhysicalStatus.READY:
            self.uav_status = self.getUAVStatus(self.uav_sn)
            rospy.sleep(self.sleep_time)
        
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_EXEC_ROUTE
        msg.drone_way_point_info.droneSn = self.uav_sn
        takeoff_point = DroneWayPoint()
        takeoff_point.type = DroneWayPoint.POINT_TAKEOFF
        takeoff_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(takeoff_point)
        for waypoint in route:
            middle_point = DroneWayPoint()
            middle_point.type = DroneWayPoint.POINT_FLYING
            middle_point.pos.x = waypoint.x
            middle_point.pos.y = waypoint.y
            middle_point.pos.z = waypoint.z
            middle_point.v = speed
            middle_point.timeoutsec = 1000
            msg.drone_way_point_info.way_point.append(middle_point)
        land_point = DroneWayPoint()
        land_point.type = DroneWayPoint.POINT_LANDING
        land_point.timeoutsec = 1000
        msg.drone_way_point_info.way_point.append(land_point)
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish UAV %s fly command" % self.uav_sn)
        
        iteration = 0
        max_iteration = 10
        while self.uav_status.drone_work_state != DronePhysicalStatus.FLYING:
            self.uav_status = self.getUAVStatus(self.uav_sn)

            if iteration >= max_iteration:
                # self.cmd_pub.publish(msg)
                break
            # self.cmd_pub.publish(msg)
            rospy.sleep(1)
            
        rospy.loginfo("UAV %s flying" % self.uav_sn)
        self.target_pos = route[-1]

    # 抛餐函数
    def releaseCargo(self):
        cargo_id = self.uav_status.bind_cargo_id
        # 等待无人机就绪
        while self.uav_status.drone_work_state != DronePhysicalStatus.READY:
            self.uav_status = self.getUAVStatus(self.uav_sn)
            rospy.sleep(self.sleep_time)
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_RELEASE_CARGO
        msg.drone_msg.drone_sn = self.uav_sn
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish UAV %s release cargo %s command" % (self.uav_sn, cargo_id))
        
        iterations = 0
        max_iterations = 5
        while self.uav_status.bind_cargo_id != 0:
            self.uav_status = self.getUAVStatus(self.uav_sn)
            rospy.sleep(self.sleep_time)
            self.cmd_pub.publish(msg)
            # 这里不能加 break 会出问题
            # self.cmd_pub.publish(msg)
            # iterations += 1  
            # if iterations >= max_iterations: 
            #     rospy.loginfo("while max_iterations break releaseCargo")
            #     break 
        if self.uav_status.bind_cargo_id == 0:
            rospy.loginfo("UAV %s release cargo %s" % (self.uav_sn, cargo_id))

    # 获取状态的函数
    def getState(self):
        self.uav_status = self.getUAVStatus(self.uav_sn)
        state = {
            "state": self.state,
            "pos": self.uav_status.pos.position,
            "have_cargo": self.uav_status.bind_cargo_id != 0,
            "cargo_id": self.uav_status.bind_cargo_id,
            "remaining_battery": self.uav_status.remaining_capacity,
            "remaining_flytime": 0.0,
        }
        return state
    
    # 执行指令主函数
    def execCmd(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("UAV %s State: %s \n" % (self.uav_sn, self.state))
            self.uav_status = self.getUAVStatus(self.uav_sn)
            uav_work_state = self.uav_status.drone_work_state
            if uav_work_state == DronePhysicalStatus.ERROR:
                rospy.logerr_once("UAV %s error" % self.uav_sn) 
                rospy.loginfo("UAV %s error" % self.uav_sn)
                break
            uav_pos = self.uav_status.pos.position
            if self.state == UAVState.FLYING_GO and self.desPosReached(self.target_pos, uav_pos, threshold=0.5):
                self.releaseCargo()
                self.stateChange(UAVState.WAITING_BACK)
            elif len(self.cmd_list) != 0:
                if self.state == UAVState.WAITING_GO:
                    self.flyOneRoute(self.cmd_list[0].route, speed=10.0)
                    # self.cmd_list.pop(0)
                    # rospy.loginfo("--1--")
                    # rospy.loginfo(self.cmd_list[0].route)
                    self.stateChange(UAVState.FLYING_GO)
                elif self.state == UAVState.WAITING_BACK:
                    self.flyOneRoute(self.cmd_list[0].route, speed=10.0)
                    # self.cmd_list.pop(0)
                    # rospy.loginfo("--2--")
                    # rospy.loginfo(self.cmd_list[0].route)
                    self.stateChange(UAVState.FLYING_BACK)
            
            self.cmd_list.clear()
            rospy.sleep(0.1)
        