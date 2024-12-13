#!/usr/bin/env python3
import rospy
import json
import copy
import time
import re
import numpy as np
import sys
sys.path.append('/home/catkin_ws/src/race_demo/src/scripts/scheduler')

from air_traffic_scheduler import Air_traffic_scheduler
from car_traffic_scheduler import Car_traffic_scheduler

from race_demo.msg import PanoramicInfo
from race_demo.srv import QueryVoxel
from race_demo.msg import SelfCommand
from race_demo.msg import SelfUAVSwarm, SelfCarSwarm
from race_demo.msg import Position
from race_demo.msg import UserCmdResponse

class mainPipeline:
    def __init__(self):
        rospy.init_node('main_pipeline', anonymous=True)
        rospy.set_param("/rosout", "/home/catkin_ws/logs")
        self.info_sub = rospy.Subscriber('/panoramic_info', PanoramicInfo, self.panoramicInfoCallback, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/cmd_resp', UserCmdResponse, self.cmdRespCallback, queue_size=10)
        self.map_client = rospy.ServiceProxy('query_voxel', QueryVoxel)

        self.uav_swarm_sub = rospy.Subscriber('/uav_swarm', SelfUAVSwarm, self.uavSwarmCallback, queue_size=10)
        self.car_swarm_sub = rospy.Subscriber('/car_swarm', SelfCarSwarm, self.carSwarmCallback, queue_size=10)
        self.uav_cmd_pub = rospy.Publisher('/self_uav_command', SelfCommand, queue_size=100)
        self.car_cmd_pub = rospy.Publisher('/self_car_command', SelfCommand, queue_size=100)

        # 读取配置文件和信息
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        # 从配置中提取装载货物点信息
        self.loading_cargo_point = self.config['taskParam']['loadingCargoPoint']
        # 从配置中提取地图边界信息
        self.map_boundary = self.config['taskParam']['mapBoundaryInfo']

        # 从配置中提取卸货站点列表
        self.unloading_cargo_stations = self.config['taskParam']['unloadingCargoStationList']
        
        # 提取并存储车的序列号列表
        self.car_infos = self.config['taskParam']['magvParamList']
        self.car_sn_list = [car['magvSn'] for car in self.car_infos]

        # 提取并存储无人机的序列号列表
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.drone_sn_list = [drone['droneSn'] for drone in self.drone_infos]

        # 存储对等端ID
        self.peer_id = self.config['peerId']
        # 存储任务全局唯一标识符
        self.task_guid = self.config['taskParam']['guid']
        # 初始化运单状态为空
        self.bills_status = None
        # 初始化分数为空
        self.score = None
        # 初始化事件列表为空
        self.events = None

        self.sites_management = {}
        self.sites_management["ground_work"] = {"is_valid" : True,
                                                "sn"       : ''}
        self.receive_car_state = False
        self.receive_uav_state = False
        
        self.air_traffic_scheduler = Air_traffic_scheduler()
        self.car_traffic_scheduler = Car_traffic_scheduler()
        
        self.uav_id_to_unloading_station_id = {}

        self.bill_cnt = 0
        self.ready_bills_list = []
        self.delivery_bills_dict = {}
        self.over_bills_dict = {}
        self.cargo_id_to_pos_dict = {}

        self.cargo_expect_time = []
        # 初始化每个起飞位置的上一个订单位置，避免相同订单位置拥堵
        self.last_cargo_pos_id = [-1, -1]
        # self.last_cargo_pos_id = []
        self.cargo_chose = None
        
    def panoramicInfoCallback(self, panoramic_info):
        self.bills_status = panoramic_info.bills
        self.score = panoramic_info.score
        if self.events != panoramic_info.events:
            self.events = panoramic_info.events
            if len(self.events) != 0 and self.events[-1].event_type == "EVENT_DELIVERY_SUCCESS":
                # 解析JSON字符串
                data = json.loads(self.events[-1].description)
                # 提取cargoIndex的值
                cargo_id = data['cargoIndex']
                over_time = data['timestamp']
                self.updateOverBills(cargo_id, over_time)
                rospy.loginfo("cargo %s actual score is: %s, over time: %s" % (cargo_id, self.events[-1].score, over_time))
            elif len(self.events) != 0:
                rospy.loginfo("Events: " + (self.events[-1].description))


    def cmdRespCallback(self, cmd):
        if cmd.type == 1:
            rospy.logerr("Receive command respose: DRONE_NOT_READY")
        elif cmd.type == 2:
            rospy.logerr("Receive command respose: DRONE_PLAN_ROUTE_VERIFICATION_FAILED")
        elif cmd.type == 3:
            rospy.logerr("Receive command respose: CAR_CANNOT_EXECUTE_RELEASE_OPERATION")
        elif cmd.type == 4:
            rospy.logerr("Receive command respose: DRONE_NOT_ON_AVIATION_OPERATION_POINT")
        elif cmd.type == 5:
            rospy.logerr("Receive command respose: DRONE_NOT_ON_BIRTHPLACE")
        elif cmd.type == 6:
            rospy.logerr("Receive command respose: DRONE_NOT_ON_LOADING_CARGO_STATION")
        elif cmd.type == 7:
            rospy.logerr("Receive command respose: DRONE_NOT_ON_UNLOADING_CARGO_STATION")
        elif cmd.type == 8:
            rospy.logerr("Receive command respose: DRONE_BINDING_OTHER_CARGO")
        elif cmd.type == 9:
            rospy.logerr("Receive command respose: DRONE_BINDING_OTHER_CAR")
        elif cmd.type == 10:
            rospy.logerr("Receive command respose: DRONE_NO_BINDING_CARGO")
        elif cmd.type == 11:
            rospy.logerr("Receive command respose: DRONE_CANNOT_EXECUTE_BATTERY_REPLACEMENT")
        elif cmd.type == 12:
            rospy.logerr("Receive command respose: CAR_NOT_READY")
        elif cmd.type == 13:
            rospy.logerr("Receive command respose: CAR_PLAN_ROUTE_VERIFICATION_FAILED")
        elif cmd.type == 14:
            rospy.logerr("Receive command respose: CAR_HAVE_OTHER_DRONE")
        elif cmd.type == 15:
            rospy.logerr("Receive command respose: CAR_NOT_ON_LOADING_CARGO_STATION")
        elif cmd.type == 16:
            rospy.logerr("Receive command respose: NO_BINDING_CAR_AND_DRONE")
        elif cmd.type == 17:
            rospy.logerr("Receive command respose: CARGO_NO_NOT_STARTED")
        elif cmd.type == 18:
            rospy.logerr("Receive command respose: CARGO_NO_DELIVERY")
        elif cmd.type == 19:
            rospy.logerr("Receive command respose: CARGO_WRONG_DESTINATION")
        elif cmd.type == 20:
            rospy.logerr("Receive command respose: LOADING_CARGO_STATION_NO_DRONE")
        elif cmd.type == 21:
            rospy.logerr("Receive command respose: LOADING_CARGO_STATION_NO_CAR")
        elif cmd.type == 22:
            rospy.logerr("Receive command respose: DRONE_IN_FLYING_RELEASE_CARGO")
        elif cmd.type == 22:
            rospy.logerr("Receive command respose: CARGO_NOT_AVAILABLE_FOR_DELIVERY_TIME")
 
    def uavSwarmCallback(self, swarm):
        self.uav_ready = swarm.READY
        self.uav_on_car = swarm.ON_CAR
        self.uav_waiting_go = swarm.WAITING_GO
        self.uav_waiting_back = swarm.WAITING_BACK
        self.uav_flying_go = swarm.FLYING_GO
        self.uav_flying_back = swarm.FLYING_BACK
        
        self.uavSwarmInfoToDict()
        self.receive_uav_state = True

    def uavSwarmInfoToDict(self):
        self.uav_info_dict = {}
        for uav_now in self.uav_ready:
            self.uav_info_dict[uav_now.sn] = uav_now
        for uav_now in self.uav_on_car:
            self.uav_info_dict[uav_now.sn] = uav_now
        for uav_now in self.uav_waiting_go:
            self.uav_info_dict[uav_now.sn] = uav_now
        for uav_now in self.uav_waiting_back:
            self.uav_info_dict[uav_now.sn] = uav_now
        for uav_now in self.uav_flying_go:
            self.uav_info_dict[uav_now.sn] = uav_now
        for uav_now in self.uav_flying_back:
            self.uav_info_dict[uav_now.sn] = uav_now
    
    def carSwarmCallback(self, swarm):
        self.car_waiting_pickup = swarm.WAITING_PICKUP
        self.car_running = swarm.RUNNING
        self.car_waiting_go_gw = swarm.WAITING_GOTO_GW
        self.car_waiting_go_aw = swarm.WAITING_GOTO_AW
        self.car_waiting_uav_work = swarm.WAITING_UAV_WORKING
        
        self.carSwarmInfoToDict()
        self.receive_car_state = True
        
    def carSwarmInfoToDict(self):
        # 将得到的信息储存到字典里面 每一次订阅就刷新一次
        self.car_info_dict = {}
        for car_now in self.car_waiting_pickup:
            self.car_info_dict[car_now.sn] = car_now
        for car_now in self.car_running:
            self.car_info_dict[car_now.sn] = car_now        
        for car_now in self.car_waiting_go_gw:
            self.car_info_dict[car_now.sn] = car_now
        for car_now in self.car_waiting_go_aw:
            self.car_info_dict[car_now.sn] = car_now        
        for car_now in self.car_waiting_uav_work:
            self.car_info_dict[car_now.sn] = car_now        
        

    #####################################################################################################
    ###### 会用到的函数
    #####################################################################################################
    # 发布指令
    def pubCommand(self, obj, command):
        if obj == "uav":
            self.uav_cmd_pub.publish(command)
        elif obj == "car":
            self.car_cmd_pub.publish(command)
        rospy.sleep(0.5)
    
    def uav_fly_cmd(self, uav_id, route):
        command = SelfCommand()
        command.route = route
        command.uav_sn = uav_id
        command.type = SelfCommand.DELIVER
        self.pubCommand("uav", command)

    def uav_swarm_fly_control(self, uav_cmd):
        # rospy.loginfo("--uav_swarm_control--")
        for key, value in uav_cmd.items():
            uav_id = key
            if value['cmd'] == 'DELIVER':
                route = value['route']
                self.uav_fly_cmd(uav_id, route)

    def car_move_to_target_cmd(self, car_id, route):
        command = SelfCommand()
        command.route = route
        command.car_sn = car_id
        command.type = SelfCommand.MOVE_TO_TARGET
        self.pubCommand("car", command)
    
    def car_load_cargo_cmd(self, car_id, cargo_id):
        command = SelfCommand()
        command.cargo_id = cargo_id
        command.car_sn = car_id
        command.type = SelfCommand.LOAD_CARGO
        self.pubCommand("car", command)
    
    def car_retrieve_uav_cmd(self, car_id):
        command = SelfCommand()
        command.car_sn = car_id
        command.type = SelfCommand.UAV_RETRIEVE
        self.pubCommand("car", command)
        
    def car_charge_uav_cmd(self, car_id):
        command = SelfCommand()
        command.car_sn = car_id
        command.type = SelfCommand.UAV_CHARGE
        self.pubCommand("car", command)

    def car_receive_uav_cmd(self, car_id):
        command = SelfCommand()
        command.car_sn = car_id
        command.type = SelfCommand.RECEIVE_UAV
        self.pubCommand("car", command)
    
    def car_swarm_control(self, car_cmd):
        # rospy.loginfo("--car_swarm_control--")
        for key, value in car_cmd.items():
            car_id = key
            if value['cmd'] == 'MOVE_TO_TARGET':
                route = value['route']
                self.car_move_to_target_cmd(car_id, route)
            elif value['cmd'] == 'LOAD_CARGO':
                if self.cargo_chose == None:
                    continue
                self.car_load_cargo_cmd(car_id, self.cargo_chose["index"])
                # 将货物位置与货物id绑定
                self.cargo_id_to_pos_dict[self.cargo_chose["index"]] = self.cargo_chose['pos']
            elif value['cmd'] == 'UAV_RETRIEVE':
                self.car_retrieve_uav_cmd(car_id)
            elif value['cmd'] == 'UAV_CHARGE':
                self.car_charge_uav_cmd(car_id)
            elif value['cmd'] == 'RECEIVE_UAV':
                self.car_receive_uav_cmd(car_id)
    
    def load_cargo_car_update(self):
        for car in self.car_waiting_go_aw:
            if car.have_uav is True:
                uav_id = car.uav_sn
                self.load_cargo_success_bill_add(uav_id, car.sn)
                
    def load_cargo_success_bill_add(self, uav_id, car_id):
        # 只有上货成功后将订单从list中删除
        if self.uav_info_dict[uav_id].have_cargo is True:
            # and self.uav_info_dict[uav_id].cargo_id == self.cargo_id:
            # 上货的时候 将uav id映射到unloading_station_id 后根据这个设定航线
            unloading_station_id = self.cargo_id_to_unloading_station_id(self.uav_info_dict[uav_id].cargo_id)
            self.uav_id_to_unloading_station_id[uav_id] = copy.copy(unloading_station_id)
            # 更新delivery_bills
            self.updateDeliveryBills(self.uav_info_dict[uav_id].cargo_id, car_id)
             
    def cargo_id_to_unloading_station_id(self, cargo_id):  
        unloading_station_id = None           
        x = self.cargo_id_to_pos_dict[cargo_id].x
        y = self.cargo_id_to_pos_dict[cargo_id].y
        z = self.cargo_id_to_pos_dict[cargo_id].z

        for i in range(len(self.unloading_cargo_stations)):
            x1 = self.unloading_cargo_stations[i]["position"]["x"]
            y1 = self.unloading_cargo_stations[i]["position"]["y"]
            z1 = self.unloading_cargo_stations[i]["position"]["z"]
            if x1 == x and y1 == y and z1 == z:
                unloading_station_id = i
                # rospy.loginfo("x = " + str(x1) + ", " + str(y1) + ", " + str(z1))
                break
        if unloading_station_id is None:
            rospy.loginfo('cargo_id_to_unloading_station_id error !!!')
        
        return unloading_station_id
    
    def calculateCargoExpectTime(self):
        for i in range(len(self.air_traffic_scheduler.unloadingStaion_id_to_path)):
            dist1_1 = abs(self.air_traffic_scheduler.take_off_p_1.z + 61)
            dist1_2 = abs(self.air_traffic_scheduler.take_off_p_2.z + 61)
            dist1_3 = abs(self.air_traffic_scheduler.take_off_p_3.z + 61)
            dist2 = 12
            go_path = self.air_traffic_scheduler.unloadingStaion_id_to_path[i]['path_go']
            dist3 = 0
            pre_pos = np.array([go_path[0][0], go_path[0][1], -61])
            for pos in go_path[2:-1]:
                dist3 += np.linalg.norm(pos - pre_pos)
                pre_pos = pos
            go_mid_pos = self.air_traffic_scheduler.unloading_cargo_id_to_mid_pos[i]['go_mid_pos']
            dist4 = np.linalg.norm(go_path[-1] - np.array(go_mid_pos))
            dist5 = np.linalg.norm(np.array(go_mid_pos) - go_path[-1])
            time1 = (dist1_1 + dist2 + dist3 + dist4 + dist5) / 4.25 + 20
            if i == 0:
                time2 = (dist1_2 + dist2 + dist3 + dist4 + dist5) / 4.25 + 60
            else:
                time2 = (dist1_2 + dist2 + dist3 + dist4 + dist5) / 4.25 + 30
            time3 = (dist1_3 + dist2 + dist3 + dist4 + dist5) / 4.25 + 20
            self.cargo_expect_time.append([time1, time2, time3])

    def genBillDict(self, bill):
        for i in range(len(self.unloading_cargo_stations)):
            x1 = self.unloading_cargo_stations[i]["position"]["x"]
            y1 = self.unloading_cargo_stations[i]["position"]["y"]
            z1 = self.unloading_cargo_stations[i]["position"]["z"]
            if x1 == bill.target_pos.x and y1 == bill.target_pos.y and z1 == bill.target_pos.z:
                unloading_station_id = i
                # rospy.loginfo("x = " + str(x1) + ", " + str(y1) + ", " + str(z1))
                break
        bill_dict = {"index": bill.index, 
                     "orderTime": bill.orderTime,
                     "betterTime": bill.betterTime,
                     "timeout": bill.timeout,
                     "pos_id": unloading_station_id,
                     "pos": bill.target_pos,
                     "remainBetterTime" : bill.betterTime,
                     "expectScore": 100.0}
        return bill_dict
        
    def updateReadyBills(self):
        current_time = time.time() * 1000 # 毫秒级时间戳
        # 将可用订单加入ready列表
        while (self.bills_status[self.bill_cnt].orderTime < current_time):
            self.ready_bills_list.append(self.genBillDict(self.bills_status[self.bill_cnt]))
            self.bill_cnt += 1

        # 更新所有订单的预期分数
        for bill in self.ready_bills_list:
            expect_delivery_time = self.expectDeliveryTime(bill["pos_id"], 1) + current_time
            if expect_delivery_time > bill["betterTime"]:
                bill["expectScore"] = 100.0 / float(bill["timeout"] - bill["betterTime"]) * float(bill["timeout"] - expect_delivery_time)
                bill["remainBetterTime"] = 0
            else:
                bill["remainBetterTime"] = bill["betterTime"] - expect_delivery_time
        
        # 删除预期分数小于或等于5的订单
        self.ready_bills_list = [bill for bill in self.ready_bills_list if bill["expectScore"] > 5]
        
        # 首先按照expectScore降序排列，然后按照remainBetterTime升序排列
        self.ready_bills_list.sort(key=lambda x: (-x['expectScore'], x['remainBetterTime']))

    def updateDeliveryBills(self, cargo_id, car_id):
        take_off_pos_id = int((int(re.search(r'\d+', car_id).group()) - 1) / 2)
        # 使用循环遍历 self.ready_bills_list 并修改原列表 
        for bill in self.ready_bills_list[:]: 
            if bill["index"] == cargo_id:
                self.ready_bills_list.remove(bill)  # 从原列表中移除
                bill["take_off_pos_id"] = take_off_pos_id
                self.delivery_bills_dict[bill["index"]] = bill  # 添加到delivery列表             
                self.last_cargo_pos_id.append(bill["pos_id"])  # 将货物位置id加入队列
                self.last_cargo_pos_id.pop(0)
                rospy.loginfo("Begin delivery cargo %s, expected score: %s, from pos %s to pos %s" % (bill["index"], bill["expectScore"], bill["take_off_pos_id"], bill["pos_id"]))
                break
    
    def updateOverBills(self, cargo_id, over_time):
        # 从delivery字典中删除，添加到over字典中
        self.over_bills_dict[cargo_id] = self.delivery_bills_dict[cargo_id]
        del self.delivery_bills_dict[cargo_id]
        # 如果分数小于100分，更新订单期望时间
        if self.events[-1].score < 100:
            self.updateCargoExpectTime(cargo_id, over_time)
    
    # 无人机送餐预计时间
    def expectDeliveryTime(self, pos_id, take_off_pos_id):
        expect_take_cargo_time = 10 * 1000
        expect_fly_time = self.cargo_expect_time[pos_id][take_off_pos_id] * 1000
        return expect_take_cargo_time + expect_fly_time 

    def updateCargoExpectTime(self, cargo_id, over_delivery_time):
        bill = self.over_bills_dict[cargo_id]
        # 根据期望反推拿到cargo的时间
        pos_id = self.over_bills_dict[cargo_id]["pos_id"]
        begin_delivery_time = float(bill["timeout"]) - (bill["expectScore"] / 100.0 * float(bill["timeout"] - bill["betterTime"])) - self.cargo_expect_time[pos_id][1] * 1000 - 10 * 1000      
        delivery_time = float(over_delivery_time - begin_delivery_time) / 1000.0
        take_off_pos_id = self.over_bills_dict[cargo_id]["take_off_pos_id"]
        new_time = 0.3 * self.cargo_expect_time[pos_id][take_off_pos_id] + 0.7 * delivery_time
        rospy.loginfo("begin_delivery_time: %s, over_delivery_time: %s." % (begin_delivery_time, over_delivery_time))
        rospy.loginfo("from pos %s to pos %s expect time update from %s to %s." % (take_off_pos_id, pos_id, self.cargo_expect_time[pos_id][1], new_time))
        # 更新期望时间为平均值
        self.cargo_expect_time[pos_id][take_off_pos_id] = new_time
    
    def getBestCargo(self):
        self.updateReadyBills()
        if len(self.ready_bills_list) == 0:
            self.cargo_chose = None
        else:
            # 选取与之前pos_id不相同的订单，如果没有就选取第一个
            self.cargo_chose = self.ready_bills_list[0]
            for order in self.ready_bills_list:
                if order['pos_id'] not in self.last_cargo_pos_id:
                    self.cargo_chose = order
                    break
        return self.cargo_chose
        
    def main_one(self):
        # 状态机开始运行
        rospy.sleep(10.0)
        self.init_para()
        
        self.air_traffic_scheduler.init_flight_path()
        self.air_traffic_scheduler.init_uav_go_flight_time()
        self.car_traffic_scheduler.get_info_from_air(self.air_traffic_scheduler.uav_go_flight_time)
        self.calculateCargoExpectTime()
        
        self.car_go_initial_pos()
        rospy.sleep(23.0)
        
        iteration = 0
        
        start_time = rospy.get_time() 
        
        while not rospy.is_shutdown():
            iteration += 1
            # 计算总运行时间并每分钟打印一次
            if iteration % 100 == 0 and iteration != 0 and len(self.ready_bills_list) != 0:
                current_time = rospy.get_time()
                elapsed_time = current_time - start_time
                rospy.loginfo("---")
                rospy.loginfo("iteration = " + str(iteration))
                rospy.loginfo("Total score = " + str(self.score))
                rospy.loginfo(f"Total time: {elapsed_time // 60} minutes {elapsed_time % 60} seconds")
                rospy.loginfo("---")
                
                rospy.logwarn("---")
                rospy.logwarn("iteration = " + str(iteration))
                rospy.logwarn("Total score = " + str(self.score))
                rospy.logwarn(f"Total time: {elapsed_time // 60} minutes {elapsed_time % 60} seconds")
                rospy.logwarn("---")
                  
            self.getBestCargo()

            self.load_cargo_car_update()
            
            # 处理飞机部分
            # 更新飞机信息
            self.air_traffic_scheduler.update_uav_info( \
                                    self.uav_info_dict, self.uav_ready,
                                    self.uav_waiting_go, self.uav_waiting_back,
                                    self.uav_flying_go, self.uav_flying_back,
                                    self.uav_id_to_unloading_station_id)
            self.air_traffic_scheduler.update_car_info( \
                                    self.car_info_dict, 
                                    self.car_waiting_pickup, self.car_running,
                                    self.car_waiting_go_aw, self.car_waiting_go_gw,
                                    self.car_waiting_uav_work)
            uav_res = self.air_traffic_scheduler.run()

            ########## self.cargo_chose["pos_id"]
            self.car_traffic_scheduler.update_car_info( \
                        self.car_info_dict, 
                        self.car_waiting_pickup, self.car_running,
                        self.car_waiting_go_aw, self.car_waiting_go_gw,
                        self.car_waiting_uav_work)
            self.car_traffic_scheduler.update_uav_info( \
                        self.uav_info_dict, self.uav_ready,
                        self.uav_waiting_go, self.uav_waiting_back,
                        self.uav_flying_go, self.uav_flying_back,
                        self.uav_id_to_unloading_station_id)
            
            self.car_traffic_scheduler.get_land_info(self.air_traffic_scheduler.car_land_pressure,
                                                self.air_traffic_scheduler.unloading_cargo_pressure)
            if self.cargo_chose is not None:
                self.car_traffic_scheduler.get_cargo_info(self.cargo_chose["pos_id"])
            else:
                self.car_traffic_scheduler.get_cargo_info(None)
            car_cmd = self.car_traffic_scheduler.run()
            self.car_swarm_control(car_cmd)
            rospy.sleep(0.1)
                 
        
        rospy.spin() 
        
        rospy.loginfo("mission completed !!!")
    
    def init_para(self):
        # 初始化工作区位置
        self.loading_pos = Position(
                self.loading_cargo_point['x'],
                self.loading_cargo_point['y'],
                self.loading_cargo_point['z'])
        
    
    def car_go_initial_pos(self):
        rospy.loginfo("car_go_initial_pos")
        # self.air_traffic_manage.init_info()
        while not self.receive_car_state or not self.receive_uav_state:
            continue
        
        car_cmd = {}
        
        # 第一辆车去1点
        car_now_id = self.car_sn_list[0]
        start_pos = self.car_info_dict[car_now_id].pos
        mid_pos = Position(4  + 180, 10 + 420, -16)
        end_pos = self.air_traffic_scheduler.para.work_station_pos
        route = [start_pos, mid_pos, end_pos]
        car_cmd[car_now_id] = {'cmd': 'MOVE_TO_TARGET', 'route': route}
        
        # 第2辆车去2点
        car_now_id = self.car_sn_list[1]
        start_pos = self.car_info_dict[car_now_id].pos
        mid_1 = Position(4  + 180, 15 + 420, -16)
        mid_2 = Position(4  + 180, 11 + 420, -16)
        end_pos = self.air_traffic_scheduler.para.work_wait_p1
        # end_pos = self.air_traffic_scheduler.land_p1
        route = [start_pos, mid_1, mid_2, end_pos]
        car_cmd[car_now_id] = {'cmd': 'MOVE_TO_TARGET', 'route': route}
        
        # 第3辆车去3点
        car_now_id = self.car_sn_list[2]
        start_pos = self.car_info_dict[car_now_id].pos
        mid_1 = Position(3+180, 24+420, -16)
        mid_2 = Position(3+180, 15+420, -16)
        end_pos = self.air_traffic_scheduler.para.land_p2
        route = [start_pos, mid_1, mid_2, end_pos]
        car_cmd[car_now_id] = {'cmd': 'MOVE_TO_TARGET', 'route': route}
        
        # 第4辆车去4点
        car_now_id = self.car_sn_list[3]
        start_pos = self.car_info_dict[car_now_id].pos
        mid_pos = Position(18+180, 15+420, -16)
        end_pos = self.air_traffic_scheduler.para.wait_p2
        route = [start_pos, mid_pos, end_pos]
        car_cmd[car_now_id] = {'cmd': 'MOVE_TO_TARGET', 'route': route}

        # 第5辆车去5点
        car_now_id = self.car_sn_list[4]
        start_pos = self.car_info_dict[car_now_id].pos
        mid_1 = Position(10+180, 19+420, -16)
        # mid_pos_2 = Position(18+180, 15+420, -16)
        end_pos = self.air_traffic_scheduler.para.land_p3
        route = [start_pos, mid_1, end_pos]
        car_cmd[car_now_id] = {'cmd': 'MOVE_TO_TARGET', 'route': route}

        # 第6辆车去6点
        car_now_id = self.car_sn_list[5]
        start_pos = self.car_info_dict[car_now_id].pos
        mid_pos = Position(10+180, 25+420, -16)
        end_pos = self.air_traffic_scheduler.para.wait_p3
        route = [start_pos, mid_pos, end_pos]
        car_cmd[car_now_id] = {'cmd': 'MOVE_TO_TARGET', 'route': route}

        self.car_swarm_control(car_cmd)
 
        
if __name__ == "__main__":
    pipeline = mainPipeline()
    pipeline.main_one()
