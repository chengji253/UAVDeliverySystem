#!/usr/bin/env python3
import rospy
import numpy as np
from enum import Enum

from uav_fsm import UAVState

from race_demo.msg import UserCmdRequest, UserCmdResponse
from race_demo.msg import CarPhysicalStatus
from race_demo.msg import SelfCommand

class CarState(Enum):
    WAITING_PICKUP = 0
    RUNNING = 1
    WAITING_GOTO_GW = 2
    WAITING_GOTO_AW = 3
    WAITING_UAV_WORKING = 4

class Car_FSM:
    def __init__(self, config):
        self.cmd_pub = rospy.Publisher('/cmd_exec', UserCmdRequest, queue_size=10000)
        self.state = CarState.WAITING_PICKUP
        self.last_state = CarState.WAITING_PICKUP
        self.car_sn = config["sn"]
        self.peer_id = config["peer_id"]
        self.task_guid = config["task_guid"]
        self.loading_pos = config["loading_pos"]
        self.getUAVStatus = config["get_uav_status"]
        self.getCarStatus = config["get_car_status"]
        self.getReadyUAV = config["get_ready_uav"]
        self.pubUAVState = config["publish_change_state"]
        self.car_status = self.getCarStatus(self.car_sn)
        self.sleep_time = 2
        self.cmd_list = []

    # 检测位置到达的函数
    def desPosReached(self, des_pos, cur_pos, threshold):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur)) < threshold

    # 无人车状态转换的函数
    def changeState(self, new_state):
        self.last_state = self.state
        self.state = new_state
        rospy.loginfo("Car %s state changed from %s to %s" % (self.car_sn, self.last_state, self.state))

    # 移动地面车辆的函数
    def moveWithRoute(self, route):
        if len(route) == 0:
            return
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_CAR_EXEC_ROUTE
        msg.car_route_info.carSn = self.car_sn
        for waypoint in route:
            msg.car_route_info.way_point.append(waypoint)
        msg.car_route_info.yaw = 0.0
        # rospy.loginfo("--route debug--")
        # rospy.loginfo(len(route))
        # rospy.loginfo(route)
        self.target_pos = route[-1]
        self.car_status = self.getCarStatus(self.car_sn)
        car_work_state = self.car_status.car_work_state
        
        # while car_work_state != CarPhysicalStatus.CAR_READY:
        #     self.car_status = self.getCarStatus(self.car_sn)
        #     car_work_state = self.car_status.car_work_state
        #     rospy.sleep(0.5)
        
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish Car %s move command" % (self.car_sn))   
        
        iteration = 0
        max_iteration = 5
        while car_work_state != CarPhysicalStatus.CAR_RUNNING:
            self.car_status = self.getCarStatus(self.car_sn)
            car_work_state = self.car_status.car_work_state
            self.cmd_pub.publish(msg)
            rospy.sleep(2)
            iteration += 1
            if iteration >= max_iteration:
                # self.cmd_pub.publish(msg)
                rospy.loginfo("Car %s moveWithRoute command timeout" % (self.car_sn))
                break
            # self.cmd_pub.publish(msg)
        
    # 移动飞机到车上
    def moveDroneOnCar(self, uav_sn):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_CAR
        msg.binding_drone.car_sn = self.car_sn
        msg.binding_drone.drone_sn = uav_sn
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish UAV %s move on Car %s command" % (uav_sn, self.car_sn))
    
        iteration = 0
        max_iteration = 5
        while self.car_status.drone_sn != uav_sn:          
            self.car_status = self.getCarStatus(self.car_sn)
            rospy.sleep(self.sleep_time)
            
            self.cmd_pub.publish(msg)   
            
            if iteration >= max_iteration:
                # self.cmd_pub.publish(msg)
                rospy.loginfo("Car %s command timeout" % (self.car_sn))
                break

        rospy.loginfo("UAV %s move on Car %s" % (uav_sn, self.car_sn))

    # 往飞机上挂餐
    def moveCargoInDrone(self, cargo_id):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_CARGO_IN_DRONE
        msg.binding_cargo.cargo_id = cargo_id
        msg.binding_cargo.drone_sn = self.car_status.drone_sn
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish move cargo %s on UAV %s command" % (cargo_id, self.car_status.drone_sn))
        uav_status = self.getUAVStatus(self.car_status.drone_sn)
        
        iteration = 0
        max_iteration = 5
        while uav_status.bind_cargo_id != cargo_id:
            uav_status = self.getUAVStatus(self.car_status.drone_sn)
            rospy.sleep(self.sleep_time) 
                
            self.cmd_pub.publish(msg)  
            if iteration >= max_iteration:
                # self.cmd_pub.publish(msg)
                rospy.loginfo("Car %s command timeout" % (self.car_sn))
                break           

        rospy.loginfo("UAV %s load cargo %s" % (self.car_status.drone_sn, cargo_id))

    # 换电函数
    def batteryReplacement(self):
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_DRONE_BATTERY_REPLACEMENT
        msg.drone_msg.drone_sn = self.car_status.drone_sn
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish UAV %s charge command" % (self.car_status.drone_sn))
        uav_status = self.getUAVStatus(self.car_status.drone_sn)
        
        iteration = 0
        max_iteration = 5
        while uav_status.remaining_capacity < 99:
            uav_status = self.getUAVStatus(self.car_status.drone_sn)
            rospy.sleep(self.sleep_time)
            
            self.cmd_pub.publish(msg)  
            if iteration >= max_iteration:
                # self.cmd_pub.publish(msg)
                rospy.loginfo("Car %s command timeout" % (self.car_sn))
                break   
        rospy.loginfo("UAV %s charge battery" % self.car_status.drone_sn)

    # 回收飞机函数
    def droneRetrieve(self):
        self.car_status = self.getCarStatus(self.car_sn)
        uav_sn = self.car_status.drone_sn
        msg = UserCmdRequest()
        msg.peer_id = self.peer_id
        msg.task_guid = self.task_guid
        msg.type = UserCmdRequest.USER_CMD_MOVE_DRONE_ON_BIRTHPLACE
        msg.unbind_info.drone_sn = self.car_status.drone_sn
        msg.unbind_info.car_sn = self.car_sn
        self.cmd_pub.publish(msg)
        rospy.loginfo("Publish Car %s retrieve UAV %s command" % (self.car_sn, self.car_status.drone_sn))
        
        iteration = 0
        max_iteration = 10
        while self.car_status.drone_sn != '':
            self.car_status = self.getCarStatus(self.car_sn)
            rospy.sleep(self.sleep_time)
            self.cmd_pub.publish(msg)
            if iteration >= max_iteration:
                # self.cmd_pub.publish(msg)
                rospy.loginfo("Car %s droneRetrieve command timeout" % (self.car_sn))
                break
            # self.cmd_pub.publish(msg)        
            
        rospy.loginfo("UAV %s retrieve from Car %s" % (uav_sn, self.car_sn,))
        self.pubUAVState(uav_sn, UAVState.READY)
    
    # 获取状态的函数
    def getState(self):
        self.car_status = self.getCarStatus(self.car_sn)
        state = {
            "state": self.state,
            "pos": self.car_status.pos.position,
            "have_uav": self.car_status.drone_sn != '',
            "uav_sn": self.car_status.drone_sn,
            "remaining_runtime": 0.0,
        }
        return state
    
    # 状态机主函数
    def execCmd(self):
        iterations = 0
        
        while not rospy.is_shutdown():
            
            iterations += 1 
            if iterations % 50 == 0:
                rospy.loginfo("Car %s State: %s \n" % (self.car_sn, self.state))
            
            self.car_status = self.getCarStatus(self.car_sn)
            car_pos = self.car_status.pos.position
            car_work_state = self.car_status.car_work_state
            if car_work_state == CarPhysicalStatus.CAR_ERROR:
                rospy.logerr_once("Car %s error" % self.car_sn)
                rospy.loginfo("Car %s error" % self.car_sn)
                break
            if self.state == CarState.WAITING_PICKUP:
                # 如果没有指令，接到无人机转换成 WAITING_GOTO_GW，
                if len(self.cmd_list) == 0:
                    if self.car_status.drone_sn != '':
                        self.changeState(CarState.WAITING_GOTO_GW)
                        self.pubUAVState(self.car_status.drone_sn, UAVState.ON_CAR)
                    # # 没有命令 到达工作区附近 随机上一个飞机
                    # elif self.desPosReached(self.loading_pos, car_pos, threshold=0.5) and car_work_state == CarPhysicalStatus.CAR_READY:
                    #     uav_sn = self.getReadyUAV()
                    #     if uav_sn != '': 
                    #         self.moveDroneOnCar(uav_sn)
                    #         self.changeState(CarState.WAITING_UAV_WORKING)
                    #         self.pubUAVState(uav_sn, UAVState.ON_CAR)
                else:
                    if self.cmd_list[0].type == SelfCommand.MOVE_TO_TARGET:
                        self.moveWithRoute(self.cmd_list[0].route)
                        self.cmd_list.pop(0)
                        self.changeState(CarState.RUNNING)
                    elif self.desPosReached(self.loading_pos, car_pos, threshold=0.6) and self.cmd_list[0].type == SelfCommand.RECEIVE_UAV:
                        uav_sn = self.getReadyUAV()
                        if uav_sn != '': 
                            self.moveDroneOnCar(uav_sn)
                            self.cmd_list.pop(0)
                            self.changeState(CarState.WAITING_UAV_WORKING)
                            self.pubUAVState(uav_sn, UAVState.ON_CAR)
                    # elif self.cmd_list[0].type == SelfCommand.RECEIVE_UAV:
                    #     self.moveWithRoute(self.cmd_list[0].route)
                    #     self.changeState(CarState.RUNNING)
            
            elif self.state == CarState.RUNNING:
                if self.desPosReached(self.target_pos, car_pos, threshold=0.5) and car_work_state == CarPhysicalStatus.CAR_READY:
                    if self.last_state == CarState.WAITING_PICKUP:
                        self.changeState(CarState.WAITING_PICKUP)
                    elif self.last_state == CarState.WAITING_GOTO_GW:
                        self.changeState(CarState.WAITING_GOTO_GW)
                    elif self.last_state == CarState.WAITING_GOTO_AW:
                        self.changeState(CarState.WAITING_GOTO_AW)
                        # 只能发布一次route，就会切换成 WAITING_GO
                        self.pubUAVState(self.car_status.drone_sn, UAVState.WAITING_GO)

            elif self.state == CarState.WAITING_GOTO_GW:
                if len(self.cmd_list) != 0:
                    self.moveWithRoute(self.cmd_list[0].route)
                    self.cmd_list.pop(0)
                    self.changeState(CarState.RUNNING)
                elif self.desPosReached(self.loading_pos, car_pos, threshold=0.5) and car_work_state == CarPhysicalStatus.CAR_READY:
                    self.changeState(CarState.WAITING_UAV_WORKING)

            elif self.state == CarState.WAITING_GOTO_AW:
                # rospy.loginfo("CarState.WAITING_GOTO_AW")
                # rospy.loginfo(self.cmd_list)
                if self.car_status.drone_sn != '' and len(self.cmd_list) != 0 \
                            and self.cmd_list[0].type == SelfCommand.UAV_RETRIEVE:
                    self.changeState(CarState.WAITING_UAV_WORKING)
                
                if len(self.cmd_list) != 0 and self.cmd_list[0].type == SelfCommand.MOVE_TO_TARGET:
                    des_pos = self.cmd_list[0].route[-1]
                    if not self.desPosReached(des_pos, car_pos, threshold=0.5):
                        self.moveWithRoute(self.cmd_list[0].route)
                        self.cmd_list.pop(0)
                        self.changeState(CarState.RUNNING)
                        # rospy.loginfo("---1---")
                    else:
                        self.cmd_list.pop(0)
                        # rospy.loginfo("---2---")
                
                elif self.car_status.drone_sn == '' and not self.desPosReached(self.loading_pos, car_pos, threshold=0.5):
                    self.changeState(CarState.WAITING_PICKUP)
                    # rospy.loginfo("---3---")
                # 无人车到某个位置，告诉无人机可以起飞
                # elif self.desPosReached(self.target_pos, car_pos, threshold=0.5):
                #     self.pubUAVState(self.car_status.drone_sn, UAVState.WAITING_GO)
                        
                elif len(self.cmd_list) != 0 and self.cmd_list[0].type == SelfCommand.LOAD_CARGO:
                    # 在上货的时候 收到了多个load cargo 因为飞机上已经有货了 直接pop出去
                    self.cmd_list.pop(0)
                
                else:
                    if len(self.cmd_list) != 0:
                        self.cmd_list.pop(0)
                    # rospy.loginfo("---4---")
                    # rospy.loginfo(len(self.cmd_list))
                    # rospy.loginfo(self.car_status.drone_sn)
                
            elif self.state == CarState.WAITING_UAV_WORKING:                
                if len(self.cmd_list) != 0:
                    if self.cmd_list[0].type == SelfCommand.UAV_RETRIEVE:
                        self.droneRetrieve()
                        self.cmd_list.pop(0)
                        self.changeState(CarState.WAITING_GOTO_AW)
                    elif self.cmd_list[0].type == SelfCommand.UAV_CHARGE:
                        self.batteryReplacement()
                        self.cmd_list.pop(0)
                    elif self.cmd_list[0].type == SelfCommand.LOAD_CARGO:
                        self.moveCargoInDrone(self.cmd_list[0].cargo_id)
                        self.cmd_list.pop(0)
                        self.changeState(CarState.WAITING_GOTO_AW)
            
            self.cmd_list.clear()
            rospy.sleep(0.1)
            
        
        