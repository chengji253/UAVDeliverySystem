import rospy
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pymtmap
import copy
import math
from para import Para

from race_demo.msg import PanoramicInfo
from race_demo.srv import QueryVoxel
from race_demo.msg import SelfCommand
from race_demo.msg import SelfUAVSwarm, SelfCarSwarm
from race_demo.msg import Position
from race_demo.msg import UserCmdResponse



class Air_traffic_scheduler:
    # 空中交通管理类
    
    def __init__(self):
        
        self.para = Para()
        
        # 无人机机场位置 和 卸货的位置
        self.drone_station_origin = self.para.drone_station_origin
        # self.work_station_pos = [190, 425, -16]
        self.work_station_pos = self.para.work_station_pos
        self.drone_store_pos = self.para.drone_store_pos
        
        self.safe_flight_min_height = -60
        self.safe_flight_max_height = -120
        
        # 所有飞行路径的信息 
        # 根据路径id 查询 path 长短 去还是回的路 有哪些飞机分别在哪个位置
        self.flight_path_info = {}
        
        # 根据 卸货点id 找到去 和 回的路径id
        self.unloadingStaion_id_to_path = {}
        
        # 降落点信息 有哪些降落点可以使用 
        self.land_point = {}
        
        # 起飞点信息 有哪些起飞点可以使用
        self.take_off_point = {}
        
        # 卸货点的位置
        self.release_cargo_pos = {}

        # self.read_map_main()
        self.init_info()
        self.init_info_one()

        self.take_off_uav_id = None
        
    def init_info(self):
        
        self.take_off_p_1 = self.para.take_off_p_1
        self.take_off_p_2 = self.para.take_off_p_2
        self.take_off_p_3 = self.para.take_off_p_3

        self.land_p1 = self.para.land_p1
        self.land_p2 = self.para.land_p2
        self.land_p3 = self.para.land_p3
        
        self.land_pos_id_to_pos = {1:self.land_p1, 2:self.land_p2, 3:self.land_p3}

        self.wait_p1  = self.para.wait_p1 
        self.wait_p2  = self.para.wait_p2 
        self.wait_p3  = self.para.wait_p3 
        
        self.line_1_car_id_set  = self.para.line_1_car_id_set 
        self.line_2_car_id_set  = self.para.line_2_car_id_set 
        self.line_3_car_id_set  = self.para.line_3_car_id_set 
        
        self.line_1_car_id_list = self.para.line_1_car_id_list
        self.line_2_car_id_list = self.para.line_2_car_id_list
        self.line_3_car_id_list = self.para.line_3_car_id_list
        
        
        self.car_id = ['SIM-MAGV-0001', 'SIM-MAGV-0002', 'SIM-MAGV-0003',
                       'SIM-MAGV-0004', 'SIM-MAGV-0005', 'SIM-MAGV-0006']
 
        self.uav_fly_back_id_to_land_pos_id = {}
        
        self.uav_id = ['SIM-DRONE-0001', 'SIM-DRONE-0002', 'SIM-DRONE-0003', 'SIM-DRONE-0004', 'SIM-DRONE-0005',
                       'SIM-DRONE-0006', 'SIM-DRONE-0007', 'SIM-DRONE-0008', 'SIM-DRONE-0009', 'SIM-DRONE-0010',
                       'SIM-DRONE-0011', 'SIM-DRONE-0012', 'SIM-DRONE-0013', 'SIM-DRONE-0014', 'SIM-DRONE-0015',
                       'SIM-DRONE-0016', 'SIM-DRONE-0017', 'SIM-DRONE-0018', 'SIM-DRONE-0019', 'SIM-DRONE-0020',
                       'SIM-DRONE-0021', 'SIM-DRONE-0022', 'SIM-DRONE-0023', 'SIM-DRONE-0024', 'SIM-DRONE-0025',
                       'SIM-DRONE-0026', 'SIM-DRONE-0027', 'SIM-DRONE-0028', 'SIM-DRONE-0029', 'SIM-DRONE-0030']
        
        # 当前无人机id对应的route
        self.uav_id_to_route_now = {}

    def init_info_one(self):
        # 去和回来 飞机之间的时间差
        # 必须要大于这个时间差才可以起飞
        self.go_time_gap = []
        self.back_time_gap = []
        
        # 飞机id 到相关飞行信息的 dict
        self.uav_flight_dict = {}
        for i in range(30):
            dic = {'flying':False, 'on_go_path':False, 'on_back_path':False,
                   'remain_time': 0, 'unloading_cargo_id':None, 'land_pos_id':None}
            uav_id = self.uav_id[i]
            self.uav_flight_dict[uav_id] = dic

        
        # 卸货点的飞机信息
        self.unloading_cargo_pressure ={0: {'num':0, 'uav_time': {}},
                                        1: {'num':0, 'uav_time': {}},
                                        2: {'num':0, 'uav_time': {}},
                                        3: {'num':0, 'uav_time': {}},
                                        4: {'num':0, 'uav_time': {}},
                                        5: {'num':0, 'uav_time': {}}}
        # uav_time = {id : remain_time}
        # 降落点的飞机信息
        self.car_land_pressure = {1: {'num':0, 'uav_time': {}},
                                  2: {'num':0, 'uav_time': {}}, 
                                  3: {'num':0, 'uav_time': {}}}
        
        # 无人机出发的时间 计算一个静态的
        # 后面的字典station id 到 时间
        self.uav_go_flight_time = {1: {},
                                   2: {},
                                   3: {}}
        
        
        # 当一个飞机选择一个降落点后 它的状态需要一定时间才能完成转变
        # 此时这个飞机选定的降落点不能被其它飞机选择
        # 当这个飞机状态变为flying back后 dict自动删除
        # 如果飞机的状态还在wait back则不删除
        self.cannot_choose_car_dict = {}
    
    def init_flight_path(self):
        # 根据卸货点初始化相应的go back航线
        
        # for cargo in unloading_cargo_stations:
        #     idx = cargo['index']
        #     x = cargo['position']['x']
        #     y = cargo['position']['y']
        #     z = cargo['position']['z']  
        #     self.release_cargo_pos[idx] = [x, y, z]            
        
        # 选择出发区域中的一个中心位置 
        # 计算这个位置到cargo的path路径
        self.take_off_center_pos = [15 + 180, 20 + 420, -16]
        self.land_center_pos = [5 + 180, 20 + 420, -16]
        
              
        path_list_go = [
            np.array([[ 195,  440,  -16],
                    [ 200,  380, -117],
                    [ 156,  186, -117],
                    [ 146,  186,  -34]]), 
            np.array([[ 195,  440,  -16],
                    [ 250,  372.3, -101],
                    [ 420,  184, -101],
                    [ 430,  184,  -10]]), 
            np.array([[ 195,  440,  -16],
                    [ 280,  366.7, -117],
                    [ 518,  172, -117],
                    [ 528,  172,  -20]]), 
            np.array([[195, 440, -16],
                    [215, 480, -70],
                    [498, 530, -70],
                    [508, 514, -22]]), 
            np.array([[195, 440, -16],
                    [230, 448, -83],
                    [564, 414, -83],
                    [564, 394, -16]]), 
            np.array([[195, 440, -16],
                    [260, 423.5, -93],
                    [490, 410, -93],
                    [490, 390, -22]])]

        
        path_list_back = [
            np.array([[ 146,  186,  -34],
                    [ 136,  196, -117],
                    [ 170,  400, -117],
                    [ 185,  440,  -16]]), 
            np.array([[ 430,  184,  -10],
                    [ 440,  194, -101],
                    [ 215,  420, -101],
                    [ 185,  440,  -16]]), 
            np.array([[ 528,  172,  -20],
                    [ 538,  182, -118],
                    [ 170,  469.2, -118],
                    [ 185,  440,  -16]]), 
            np.array([[508, 514, -22],
                    [498, 504, -61],
                    [230, 466.3, -61],
                    [185, 440, -16]]), 
            np.array([[564, 394, -16],
                    [564, 394, -103],
                    [230, 434.5, -103],
                    [185, 440, -16]]), 
            np.array([[490, 390, -22],
                    [490, 380, -93],
                    [245, 416.5, -93],
                    [185, 440, -16]])] 
        
        # 卸货点的go back mid pos
        self.unloading_cargo_id_to_mid_pos = {
            0:{'go_mid_pos':[154, 186, -61], 'back_mid_pos':[138, 186, -61]},
            1:{'go_mid_pos':[430, 176, -61], 'back_mid_pos':[430, 192, -61]},
            2:{'go_mid_pos':[520, 172, -61], 'back_mid_pos':[536, 172, -61]},
            3:{'go_mid_pos':[508, 522, -61], 'back_mid_pos':[508, 506, -61]},
            4:{'go_mid_pos':[564, 402, -61], 'back_mid_pos':[564, 386, -61]},
            5:{'go_mid_pos':[490, 398, -61], 'back_mid_pos':[490, 382, -61]}}
        
        # for i in range(1, 7):
        #     end_pos = [self.land_center_pos[0], 
        #                  self.land_center_pos[1], self.land_center_pos[2]]
        #     start_pos = [self.release_cargo_pos[i][0], 
        #                self.release_cargo_pos[i][1], self.release_cargo_pos[i][2]]
        #     path_res = self.path_search_easy(start_pos, end_pos)
        #     # rospy.loginfo(path_res)
        #     path_list_back.append(path_res)
        # rospy.loginfo(path_list_back)

        # self.plot_map(path_list_go, path_list_back)
        # rospy.loginfo("----")
        # rospy.loginfo(self.release_cargo_pos)
        
        for i in range(len(path_list_go)):
            path_go = path_list_go[i]
            path_back = path_list_back[i]
            self.unloadingStaion_id_to_path[i] = \
            {"path_go": path_go, "path_back": path_back}

        # rospy.loginfo(self.unloadingStaion_id_to_path)
        
    def update_uav_info(self, uav_info_dict, uav_ready,
                          uav_waiting_go, uav_waiting_back,
                          uav_flying_go, uav_flying_back,
                          uav_id_to_unloading_station_id):
        # 根据总节点的信息 更新当下的path drone信息
        # 从而完成调度
        self.uav_info = uav_info_dict
        self.uav_ready = uav_ready
        self.uav_waiting_go = uav_waiting_go
        self.uav_waiting_back = uav_waiting_back
        self.uav_flying_go = uav_flying_go
        self.uav_flying_back = uav_flying_back
        self.uav_id_to_unloading_station_id = uav_id_to_unloading_station_id
        # rospy.loginfo("uav_info_dict update")
        # rospy.loginfo(uav_info_dict)

    def update_car_info(self, car_info_dict, 
                            car_waiting_pickup, car_running,
                            car_waiting_go_aw, car_waiting_go_gw,
                            car_waiting_uav_work):
        # 根据总节点的信息 更新当下car信息 从而完成调度
        self.car_info = car_info_dict
        self.car_waiting_pickup = car_waiting_pickup
        self.car_running = car_running                              
        self.car_waiting_go_aw = car_waiting_go_aw
        self.car_waiting_go_gw = car_waiting_go_gw
        self.car_waiting_uav_work = car_waiting_uav_work

    def find_car_info_set(self):
        self.car_waiting_pickup_set = set()
        self.car_running_set = set()
        self.car_waiting_go_aw_set = set()
        self.car_waiting_go_gw_set = set()
        self.car_waiting_uav_work_set = set()

        for car in self.car_waiting_pickup:
            car_id = car.sn
            self.car_waiting_pickup_set.add(car_id)
        for car in self.car_running:
            car_id = car.sn
            self.car_running_set.add(car_id)
        for car in self.car_waiting_go_aw:
            car_id = car.sn
            self.car_waiting_go_aw_set.add(car_id)
        for car in self.car_waiting_go_gw:
            car_id = car.sn
            self.car_waiting_go_gw_set.add(car_id)
        for car in self.car_waiting_uav_work:
            car_id = car.sn
            self.car_waiting_uav_work_set.add(car_id)

        # rospy.logwarn("self.car_running_set")
        # rospy.logwarn(self.car_running_set)
    def find_uav_fly_back(self):
        # 找到哪些无人机正在回来 哪些已经回来结束-降落
        uav_fly_back_set = set()
        uav_all_set = set()
        
        for uav in self.uav_flying_back:
            uav_id = uav.sn
            uav_fly_back_set.add(uav_id)
        
        for key, value in self.uav_info.items():
            uav_id = key
            uav_all_set.add(uav_id)
        
        # rospy.loginfo("uav_fly_back_set")
        # rospy.loginfo(uav_fly_back_set)
        # rospy.loginfo("uav_all_set")
        # rospy.loginfo(uav_all_set)
        # for uav_id in uav_all_set:
        #     if uav_id not in uav_fly_back_set:
        #         if uav_id in self.uav_fly_back_id_to_land_pos_id:
        #             del self.uav_fly_back_id_to_land_pos_id[uav_id]
        
        # rospy.loginfo(self.car_land_pressure)

    def judge_time_conflict(self, remain_time, uav_time_list, time_gap):
        rospy.loginfo("judge_time_conflict")
        rospy.loginfo("uav_time_list")
        rospy.loginfo(uav_time_list)
        rospy.loginfo("remain time = " + str(remain_time))
        rospy.loginfo("time_gap = " + str(time_gap))
        for i in range(len(uav_time_list)):
            if abs(uav_time_list[i] - remain_time) < time_gap:
                rospy.loginfo("have conflict")
                return True
        
        # 返回false表示没有冲突
        rospy.loginfo("no conflict")
        return False
    
    def canont_choose_update(self):
        # 不能选择的car刷新
        uav_fly_back_set = set()
        uav_wait_back_set = set()
        for uav in self.uav_flying_back:
            uav_fly_back_set.add(uav.sn)
        
        for uav in self.uav_waiting_back:
            uav_wait_back_set.add(uav.sn)
        
        cannot_choose_car_list = []
        for key, value in self.cannot_choose_car_dict.items():
            cannot_choose_car_list.append(key)
        
        if len(cannot_choose_car_list) != 0:
            rospy.loginfo("cannot choose car")
            rospy.loginfo(self.cannot_choose_car_dict)
        
        for uav_id in cannot_choose_car_list:
            if uav_id in uav_fly_back_set and uav_id not in uav_wait_back_set:
                rospy.loginfo("cannot choose delete")
                rospy.loginfo("uav_id = " + str(uav_id))
                rospy.loginfo("pos id = " + str(self.cannot_choose_car_dict[uav_id]))
                del self.cannot_choose_car_dict[uav_id]
    
    def set_uav_to_route_dict(self, uav_id, route):
        path_list = []
        for p in route:
            path_list.append([p.x, p.y, p.z])
        dict_n = {'route': route, 'path_idx': 0, 'path_list': path_list}
        self.uav_id_to_route_now[uav_id] = dict_n

    def choose_urgent_uav_go_back(self):
        # 在所有self.uav_waiting_back中选择最紧急的回来
        # 找到那些有飞机正在过来 且到达时间最小的
        if len(self.uav_waiting_back) == 1:
            return self.uav_waiting_back[0].sn

        uav_id_time = {}
        for uav in self.uav_waiting_back:
            uav_time = []
            uav_id = uav.sn
            unloading_cargo_id = self.uav_flight_dict[uav_id]['unloading_cargo_id']
            for another_uav in self.uav_flying_go:
                another_uav_id = another_uav.sn
                another_unload_cargo_id = self.uav_flight_dict[another_uav_id]['unloading_cargo_id']
                another_remain_time = self.uav_flight_dict[another_uav_id]['remain_time']
                if unloading_cargo_id == another_unload_cargo_id:
                    rospy.logwarn(str(another_uav_id)+" is flying to the same, time=" + str(another_remain_time))
                    rospy.loginfo(str(another_uav_id)+" is flying to the same, time=" + str(another_remain_time))
                    uav_time.append(another_remain_time)
            if len(uav_time) == 0:
                uav_time.append(9999)
            min_element = min(uav_time)
            uav_id_time[uav_id] = min_element
        min_time_drone_id = min(uav_id_time, key=lambda k: uav_id_time[k])
        rospy.loginfo("choose the urgent uav to go back=" + str(min_time_drone_id))
        rospy.logwarn("choose the urgent uav to go back=" + str(min_time_drone_id))
        return min_time_drone_id


    def determine_which_uav_can_back(self):
        uav_back_id_set = set()
        uav_id_to_land_pos_id = {}
        time_gap_all = self.para.back_time_gap_all
        
        if len(self.uav_waiting_back) != 0:
            uav_id = self.choose_urgent_uav_go_back()
            uav = self.uav_info[uav_id]
            # uav = self.uav_waiting_back[0]
            # uav_id = uav.sn
            # rospy.logwarn('uav_id=' +str(uav_id))
            if uav_id in self.cannot_choose_car_dict:
                pos_id = self.cannot_choose_car_dict[uav_id]
                uav_back_id_set.add(uav_id)
                uav_id_to_land_pos_id[uav_id] = pos_id
                return uav_back_id_set, uav_id_to_land_pos_id
                
            uav_pos = uav.pos
            unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
            path = self.unloadingStaion_id_to_path[unloading_station_id]['path_back']
            
            # remain_time = self.cal_remain_time(uav_pos, path)
            # 让line1 自动加1 这样优先选择 2 3 降落
            if len(self.uav_ready) >= 3:
                self.car_land_pressure[1]['num'] += 1
            # elif len(self.uav_ready) == 0:
            #     self.car_land_pressure[2]['num'] += 1
            #     self.car_land_pressure[3]['num'] += 1
            
            # 选择 num最小 且时间不冲突的 降落位置
            sorted_pos_id = sorted(self.car_land_pressure, key=lambda x: self.car_land_pressure[x]['num'])
            # 和所有别的飞机的降落时间差控制在 time_gap_all 以上 防止降落时冲突
            time_list_all = []
            for key, value in self.car_land_pressure.items():
                uav_time = value['uav_time']
                for k1, v1 in uav_time.items():
                    time_list_all.append(v1)
            # rospy.loginfo("time_list_all")
            # rospy.loginfo(time_list_all)
            # 和当前降落位置的飞机不能发生冲突
            cannot_choose_car_set = set()
            for key, value in self.cannot_choose_car_dict.items():
                cannot_choose_car_set.add(value)
            
            for pos_id in sorted_pos_id:
                # 每一个pos id 都要计算一下 remain time
                route = self.set_route_for_uav_back(uav_id, pos_id)
                self.set_uav_to_route_dict(uav_id, route)
                remain_time = self.cal_remain_time_one(uav_pos, uav_id)
                # remain_time = self.cal_remain_time(uav_pos, path)
                uav_number = self.car_land_pressure[pos_id]['num']
                # rospy.loginfo("uav_time_list")
                # rospy.loginfo(uav_time_list)
                # 每一个pos 都会判断一下 在for循环里面
                if uav_number == 0:
                    rospy.loginfo("uav_number == 0")
                    if self.judge_time_conflict(remain_time, time_list_all, time_gap_all) is False \
                    and pos_id not in cannot_choose_car_set \
                    and self.judge_car_situation_can_back(pos_id) is True:
                        rospy.loginfo("没有飞机在这条line上 可以返回 line=" + str(pos_id)+",uav_id=" + str(uav_id))
                        rospy.logwarn("uav_number == 0")
                        rospy.logwarn("没有飞机在这条line上 可以返回 line=" + str(pos_id)+",uav_id=" + str(uav_id))   
                        uav_back_id_set.add(uav_id)
                        uav_id_to_land_pos_id[uav_id] = pos_id
                        self.cannot_choose_car_dict[uav_id] = pos_id
                        route = self.set_route_for_uav_back(uav_id, pos_id)
                        self.set_uav_to_route_dict(uav_id, route)
                        remain_time = self.cal_remain_time_one(uav_pos, uav_id)
                        break

                elif uav_number == 1:
                    rospy.loginfo("uav_number == 1")
                    # 和当前的line的飞机不能有冲突
                    # 和别的line飞机不能有冲突
                    uav_time_n = self.car_land_pressure[pos_id]['uav_time']
                    uav_time_list = []
                    for key, value in uav_time_n.items():
                        uav_time_list.append(value)
                    time_gap_one = self.determine_land_pos_back_time(pos_id)
                    if self.judge_time_conflict(remain_time, uav_time_list, time_gap_one) is False \
                    and self.judge_time_conflict(remain_time, time_list_all, time_gap_all) is False:
                        if pos_id not in cannot_choose_car_set:
                            rospy.loginfo("满足约束 可以返回 line=" + str(pos_id)+ ",uav_id=" + str(uav_id) + " time_gap=" + str(time_gap_one))
                            rospy.logwarn("uav_number == 1, time_gap=" + str(time_gap_one))
                            rospy.logwarn("满足约束 可以返回 line=" + str(pos_id)+ ",uav_id=" + str(uav_id) + " time_gap=" + str(time_gap_one)) 
                            uav_back_id_set.add(uav_id)
                            uav_id_to_land_pos_id[uav_id] = pos_id
                            self.cannot_choose_car_dict[uav_id] = pos_id
                            route = self.set_route_for_uav_back(uav_id, pos_id)
                            self.set_uav_to_route_dict(uav_id, route)
                            remain_time = self.cal_remain_time_one(uav_pos, uav_id)
                            break
                else:
                    pass                    
        # if uav_back_id_set:
        #     rospy.loginfo('uav_back_id_set')
        #     rospy.loginfo(uav_back_id_set)
        rospy.loginfo('uav_id_to_land_pos_id')
        rospy.loginfo(uav_id_to_land_pos_id)
            
        return uav_back_id_set, uav_id_to_land_pos_id
    
    def determine_which_uav_can_go(self):
        rospy.loginfo("determine_which_uav_can_go")
        uav_go_id_set = set()
        time_gap = self.para.go_time_gap_all
        
        uav_is_taking_off = self.judge_uav_is_taking_off()
        
        if uav_is_taking_off is True:
            rospy.loginfo("uav is taking off cannot go!")
            return uav_go_id_set
        
        # for uav in self.uav_waiting_go:
        if len(self.uav_waiting_go) != 0:
            uav = self.uav_waiting_go[0]
            uav_id = uav.sn
            uav_pos = uav.pos
            unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
            path = self.unloadingStaion_id_to_path[unloading_station_id]['path_go']
            
            # remain_time = self.cal_remain_time(uav_pos, path)
            route = self.set_route_for_uav_go(uav_id)
            # self.uav_id_to_route_now[uav_id] = route
            self.set_uav_to_route_dict(uav_id, route)
            remain_time = self.cal_remain_time_one(uav_pos, uav_id)
            
            uav_time_n = self.unloading_cargo_pressure[unloading_station_id]['uav_time']
            uav_time_list = []
            for key, value in uav_time_n.items():
                uav_time_list.append(value)
            
            rospy.loginfo("uav_time_list")
            rospy.loginfo(uav_time_list)
            if self.judge_time_conflict(remain_time, uav_time_list, time_gap) is False:
                # uav_go_id_set.add(uav_id)
                self.take_off_uav_id = uav_id
                rospy.loginfo(str(uav_id) + " is allowed to go!")
                # rospy.logwarn(str(uav_id) + " is allowed to go!")
            else:
                rospy.loginfo("can not go!, uav_id=" + str(uav_id) + ", unloading_station_id=\
                              " + str(unloading_station_id))
                rospy.loginfo("can not go!=" + str(uav_time_n))

            # rospy.loginfo('uav_go_id_set')
            # rospy.loginfo(uav_go_id_set)
        
        # return uav_go_id_set
    

    def cal_flight_time(self, route):
        path_list = []
        for p in route:
            path_list.append([p.x, p.y, p.z])
        
        remaining_distance = 0
        for j in range(0, len(path_list) - 1):
            remaining_distance += self.euclidean_distance(path_list[j], path_list[j + 1])
        
        remaining_time = remaining_distance / 8 + 10
        return remaining_time        

    def cal_remain_time_one(self, uav_pos, uav_id):
        current_pos = [uav_pos.x, uav_pos.y, uav_pos.z]
        path_list = self.uav_id_to_route_now[uav_id]['path_list']
        path_idx = self.uav_id_to_route_now[uav_id]['path_idx']
        
        # rospy.logwarn("path idx = " + str(path_idx))
        # rospy.logwarn("path_list = " + str(path_list))
                    
        # 如果飞机还没有起飞 那么直接计算 总的路径距离除以速度就好了
        uav_wait_set = set()
        for uav in self.uav_waiting_back:
            uav_wait_set.add(uav.sn)
        for uav in self.uav_waiting_go:
            uav_wait_set.add(uav.sn)

        if uav_id in uav_wait_set:
            remaining_distance = 0
            for j in range(0, len(path_list) - 1):
                remaining_distance += self.euclidean_distance(path_list[j], path_list[j + 1])
            
            remaining_time = remaining_distance / 8 + 10
            return remaining_time
        
        # 在飞行中 那么计算剩下时间
        remaining_time = self.calculate_remaining_time_spe(current_pos, path_list, path_idx)
        
        if path_idx < len(path_list):
            # rospy.logwarn("node = " + str(path_list[path_idx]))
            # rospy.logwarn("eu_dis = " + str(self.euclidean_distance(path_list[path_idx], current_pos)))
            if self.euclidean_distance(path_list[path_idx], current_pos) <= 10:
                self.uav_id_to_route_now[uav_id]['path_idx'] += 1
                # rospy.logwarn("path idx + 1")
        
        if remaining_time <= 5:
            remaining_time = 5
            
        # rospy.loginfo("remain_time = " + str(remaining_time))

        return remaining_time
    
    def calculate_remaining_time_spe(self, current_position, path_list, nearest_index):        
        velocity = 8
        # 计算从当前位置到该路径段终点的距离
        if nearest_index >= len(path_list):
            return 0
        
        remaining_distance = self.euclidean_distance(current_position, path_list[nearest_index])

        # 计算从该路径段之后到终点的总距离
        for j in range(nearest_index, len(path_list) - 1):
            remaining_distance += self.euclidean_distance(path_list[j], path_list[j + 1])
        
        # 根据速度计算剩余时间
        remaining_time = remaining_distance / velocity
        return remaining_time


    def euclidean_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def project_onto_segment(self, p, p1, p2):
        # 计算向量
        v = np.array(p2) - np.array(p1)
        w = np.array(p) - np.array(p1)
        # 计算投影比例
        c1 = np.dot(w, v)
        c2 = np.dot(v, v)
        b = c1 / c2
        # 返回投影点
        return np.array(p1) + b * v

    def is_point_on_segment(self, p, p1, p2):
        # 判断投影点是否在线段内
        return np.all(np.minimum(p1, p2) <= p) and np.all(p <= np.maximum(p1, p2))    
    
    def cal_remain_time(self, uav_pos, path):
        path_height = path[1][2]
        height_gap = abs(uav_pos.z - path_height)
        uav_land_time = 5
        
        pos_n2 = Position(path[1][0], path[1][1], path[1][2])
        dis_n2 = self.dis_cal(uav_pos, pos_n2)
        
        pos_n3 = Position(path[2][0], path[2][1], path[2][2])
        dis_n3 = self.dis_cal(uav_pos, pos_n3)
        
        long_dis = self.dis_cal(pos_n2, pos_n3)

        vel = 7
        # rospy.loginfo("height_gap = " + str(height_gap))
        if height_gap <= 1:
            # 已经开始直线飞行 此时uav已经开始正式飞行前往卸货点 直接计算到第三个点的距离 除以速度就是飞行时间
            # rospy.loginfo("--直线飞行--")
            land_time = abs(path_height - path[3][2])/vel + uav_land_time
            remain_time = dis_n3/vel + land_time
            return remain_time
        
        if dis_n2 < dis_n3:
            # 刚起飞 仍然在上升
            # rospy.loginfo("--上升--")
            raise_time = (height_gap)/vel
            remain_time = long_dis/vel + raise_time + uav_land_time + abs(path_height - path[3][2])/vel
        else:
            # 开始降落了
            # rospy.loginfo("--降落--")
            remain_time = abs(uav_pos.z - path[3][2])/vel + uav_land_time
        return remain_time
    def update_uav_flight_info(self):
        # 根据当前传进来的飞机 状态更新相关状态
        
        # 卸货点的飞机信息 动态更新
        self.unloading_cargo_pressure ={0: {'num':0, 'uav_time': {}},
                                        1: {'num':0, 'uav_time': {}},
                                        2: {'num':0, 'uav_time': {}},
                                        3: {'num':0, 'uav_time': {}},
                                        4: {'num':0, 'uav_time': {}},
                                        5: {'num':0, 'uav_time': {}}}
        # uav_time = {id : remain_time}
        # 降落点的飞机信息
        self.car_land_pressure = {1: {'num':0, 'uav_time': {}},
                                  2: {'num':0, 'uav_time': {}}, 
                                  3: {'num':0, 'uav_time': {}}}
        
        for uav in self.uav_flying_go:
            uav_id = uav.sn
            uav_pos = uav.pos
            unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
            # rospy.loginfo(self.unloadingStaion_id_to_path)
            go_path = self.unloadingStaion_id_to_path[unloading_station_id]['path_go']
            
            # remain_time = self.cal_remain_time(uav_pos, go_path)
            remain_time = self.cal_remain_time_one(uav_pos, uav_id)
                
            self.uav_flight_dict[uav_id]['remain_time'] = remain_time
            self.uav_flight_dict[uav_id]['flying'] = True
            self.uav_flight_dict[uav_id]['on_go_path'] = True
            self.uav_flight_dict[uav_id]['on_back_path'] = False
            
            unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
            self.uav_flight_dict[uav_id]['land_pos_id'] = None
            self.uav_flight_dict[uav_id]['unloading_cargo_id'] = unloading_station_id          
            
            self.unloading_cargo_pressure[unloading_station_id]['num'] += 1
            self.unloading_cargo_pressure[unloading_station_id]['uav_time'].update({uav_id: remain_time})
                
        for uav in self.uav_flying_back:
            uav_id = uav.sn
            unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
            uav_pos = uav.pos
            back_path = self.unloadingStaion_id_to_path[unloading_station_id]['path_back']

            # remain_time = self.cal_remain_time(uav_pos, back_path)
            remain_time = self.cal_remain_time_one(uav_pos, uav_id)
            
            self.uav_flight_dict[uav_id]['remain_time'] = remain_time
            self.uav_flight_dict[uav_id]['flying'] = True
            self.uav_flight_dict[uav_id]['on_go_path'] = False
            self.uav_flight_dict[uav_id]['on_back_path'] = True  
            
            land_pos_id = self.uav_fly_back_id_to_land_pos_id[uav_id]
            
            self.uav_flight_dict[uav_id]['land_pos_id'] = land_pos_id
            self.uav_flight_dict[uav_id]['unloading_cargo_id'] = None      
            
            self.car_land_pressure[land_pos_id]['num'] += 1
            self.car_land_pressure[land_pos_id]['uav_time'].update({uav_id: remain_time})
        
        rospy.loginfo("self.unloading_cargo_pressure")
        for key, value in self.unloading_cargo_pressure.items():
            if value['num'] == 0:
                continue
            rospy.loginfo(key)
            rospy.loginfo(value)
        # rospy.loginfo(self.unloading_cargo_pressure)
        rospy.loginfo("")
        rospy.loginfo("self.car_land_pressure")
        for key, value in self.car_land_pressure.items():
            if value['num'] == 0:
                continue
            rospy.loginfo(key)
            rospy.loginfo(value)
        # rospy.loginfo(self.car_land_pressure)
        rospy.loginfo("")
        
        # rospy.logwarn("self.unloading_cargo_pressure")
        # for key, value in self.unloading_cargo_pressure.items():
        #     if value['num'] == 0:
        #         continue
        #     rospy.logwarn(key)
        #     rospy.logwarn(value)
        # # rospy.loginfo(self.unloading_cargo_pressure)
        # rospy.logwarn("")
        # rospy.logwarn("self.car_land_pressure")
        # for key, value in self.car_land_pressure.items():
        #     if value['num'] == 0:
        #         continue
        #     rospy.logwarn(key)
        #     rospy.logwarn(value)
        # # rospy.loginfo(self.car_land_pressure)
        # rospy.logwarn("")
    def judge_uav_is_taking_off(self):
        for uav in self.uav_flying_go:
            uav_id = uav.sn
            if abs(uav.pos.z - self.take_off_p_1.z) <= 30 \
                and abs(uav.pos.x - (180+15)) <= 10:
                rospy.loginfo(str(uav_id) + " is taking off")
                return True
        return False
    
    def judge_land_pos_which_line(self, land_n):
        if abs(land_n.x- self.para.land_p1.x) < 1 and \
        abs(land_n.y- self.para.land_p1.y) < 1:
            return 1
        elif abs(land_n.x- self.para.land_p2.x) < 1 and \
        abs(land_n.y- self.para.land_p2.y) < 1:
            return 2
        elif abs(land_n.x- self.para.land_p3.x) < 1 and \
        abs(land_n.y- self.para.land_p3.y) < 1:
            return 3
    

    def init_uav_go_flight_time(self):
        # 初始化三个line 六个station的出发飞行时间 存放在字典里面
        for line_id in range(1, 4):
            for unloading_station_id in range(0, 6):
                route = self.set_route_for_init(unloading_station_id, line_id)
                time = self.cal_flight_time(route)
                self.uav_go_flight_time[line_id][unloading_station_id] = time
        rospy.loginfo("uav_go_flight_time")
        rospy.loginfo(self.uav_go_flight_time)
        # rospy.logwarn("uav_go_flight_time")
        # rospy.logwarn(self.uav_go_flight_time)

    def set_route_for_init(self, unloading_station_id, line_id):
        # 为即将出发的无人机计算一个route
        go_path = self.unloadingStaion_id_to_path[unloading_station_id]['path_go']
        route = []
        
        if line_id == 1:
            now_pos = self.para.take_off_p_1
        elif line_id == 2:
            now_pos = self.para.take_off_p_2
        elif line_id == 3:
            now_pos = self.para.take_off_p_3

        now_pos.z += -5
        
        take_off_mid = copy.copy(now_pos)
        take_off_mid.z = -61

        if abs(now_pos.y - (self.take_off_p_1.y)) < 1:
            take_off_mid.x += 8
        elif abs(now_pos.y - (self.take_off_p_2.y)) < 1:
            take_off_mid.x += 8              
        elif abs(now_pos.y - (self.take_off_p_3.y)) < 1:
            take_off_mid.x += 8
                            
        # 添加当前位置 和 起飞前馈点
        route.append(now_pos)
        route.append(take_off_mid)
        
        # 添加第二个前馈点
        take_off_mid_two = copy.deepcopy(take_off_mid)
        take_off_mid_two.x += 12
        route.append(take_off_mid_two)
        
        # 添加 第三个前馈点 - 航线第一个点 xy 一样 但是高度是-61
        path_1 = go_path[1]
        take_off_mid_three = Position(path_1[0], path_1[1], -61)
        route.append(take_off_mid_three)

        # 添加航线位置
        for path in go_path[1:-1, :]:
            p_n = Position(path[0], path[1], path[2])
            route.append(p_n)
        
        # 添加降落前馈位置
        go_mid_pos = self.unloading_cargo_id_to_mid_pos[unloading_station_id]['go_mid_pos']
        mid_go = Position(go_mid_pos[0], go_mid_pos[1], go_mid_pos[2])
        route.append(mid_go)
        
        # 添加降落位置
        p_n = go_path[-1, :]
        land_n = Position(p_n[0], p_n[1], p_n[2])
        land_n.z += -5
        route.append(land_n)

        return route        

    def set_route_for_uav_go(self, uav_id):
        # 为即将出发的无人机计算一个route
        unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
        # rospy.loginfo(self.unloadingStaion_id_to_path)
        go_path = self.unloadingStaion_id_to_path[unloading_station_id]['path_go']
        route = []
        now_pos = self.uav_info[uav_id].pos
        now_pos.z += -5
        
        take_off_mid = copy.copy(now_pos)
        take_off_mid.z = -61

        if abs(now_pos.y - (self.take_off_p_1.y)) < 1:
            take_off_mid.x += 8
        elif abs(now_pos.y - (self.take_off_p_2.y)) < 1:
            take_off_mid.x += 8              
        elif abs(now_pos.y - (self.take_off_p_3.y)) < 1:
            take_off_mid.x += 8
                            
        # 添加当前位置 和 起飞前馈点
        route.append(now_pos)
        route.append(take_off_mid)
        
        # 添加第二个前馈点
        take_off_mid_two = copy.deepcopy(take_off_mid)
        take_off_mid_two.x += 12
        route.append(take_off_mid_two)
        
        # 添加 第三个前馈点 - 航线第一个点 xy 一样 但是高度是-61
        path_1 = go_path[1]
        take_off_mid_three = Position(path_1[0], path_1[1], -61)
        route.append(take_off_mid_three)

        # 添加航线位置
        for path in go_path[1:-1, :]:
            p_n = Position(path[0], path[1], path[2])
            route.append(p_n)
        
        # 添加降落前馈位置
        go_mid_pos = self.unloading_cargo_id_to_mid_pos[unloading_station_id]['go_mid_pos']
        mid_go = Position(go_mid_pos[0], go_mid_pos[1], go_mid_pos[2])
        route.append(mid_go)
        
        # 添加降落位置
        p_n = go_path[-1, :]
        land_n = Position(p_n[0], p_n[1], p_n[2])
        land_n.z += -5
        route.append(land_n)

        return route
    
    def set_route_for_uav_back(self, uav_id, land_pos_id):
        # 这里要给定land pos id 计算出route
        unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
        back_path = self.unloadingStaion_id_to_path[unloading_station_id]['path_back']
        route = []
        now_pos = self.uav_info[uav_id].pos
        now_pos.z += -5
        
        # 添加现在的位置
        route.append(now_pos)
        
        # 添加回来的 卸货起飞 前馈点  
        back_mid_pos = self.unloading_cargo_id_to_mid_pos[unloading_station_id]['back_mid_pos'] 
        mid_pos_one = Position(back_mid_pos[0], back_mid_pos[1], back_mid_pos[2])
        route.append(mid_pos_one)
        
        # 添加航线上的两个点
        for path in back_path[1:-1, :]:
            p_n = Position(path[0], path[1], path[2])
            route.append(p_n)
        # land_n = copy.deepcopy(self.land_point[0])
        
        # self.find_uav_fly_back()
        # land_pos_id = uav_id_to_land_pos_id[uav_id]
        land_pos = self.land_pos_id_to_pos[land_pos_id]
        self.uav_fly_back_id_to_land_pos_id[uav_id] = land_pos_id
        land_n = copy.deepcopy(land_pos)

        land_n.z += -5
        
        path_2 = back_path[2]
        
        line_num = self.judge_land_pos_which_line(land_n)
        
        # 回来的land前馈点一
        land_mid_one = Position(path_2[0], path_2[1], path_2[2])
        # 回来的land前馈点二 
        land_mid_two = copy.deepcopy(land_pos)
        # 回来的land前馈点三
        land_mid_three = copy.deepcopy(land_pos)
        
        # line_1_height = -80
        # line_2_height = -95
        # line_3_height = -110
        line_1_height = -80
        line_2_height = -90
        line_3_height = -100

        if line_num == 1:
            land_mid_one.z = line_1_height
            
            land_mid_two.x += -30
            land_mid_two.y += -35
            land_mid_two.z = line_1_height

            land_mid_three.z = -61
        elif line_num == 2:
            land_mid_one.z = line_2_height
            
            land_mid_two.x += -46
            land_mid_two.z = line_2_height

            land_mid_three.z = -61
        elif line_num == 3:
            land_mid_one.z = line_3_height

            land_mid_two.x += -30
            land_mid_two.y += 35
            land_mid_two.z = line_3_height
            
            land_mid_three.z = -61
        else:
            rospy.loginfo("line num error")
        
        route.append(land_mid_one)
        route.append(land_mid_two)
        route.append(land_mid_three)
                
        # 添加回来的land点  
        route.append(land_n)        
        
        return route
    
    def update_uav_takeoff_id(self):
        uav_flying_go_set = set()
        for uav in self.uav_flying_go:
            uav_id = uav.sn
            uav_flying_go_set.add(uav_id)
        
        if self.take_off_uav_id in uav_flying_go_set:
            self.take_off_uav_id = None
            # rospy.logwarn("take off is over! id is None")
            rospy.loginfo("take off is over! id is None")

    def run(self):
        cmd_res = {}
        
        self.update_uav_flight_info()
        self.canont_choose_update()
        self.find_land_wait_pos_car_set()
        self.find_car_info_set()
        self.find_neighbor_car_set()
        self.update_uav_takeoff_id()

        uav_back_id_set, uav_id_to_land_pos_id = self.determine_which_uav_can_back()
        self.determine_which_uav_can_go()
        self.find_uav_fly_back()
        
        
        # 让需要回来的飞机 立即起飞
        if len(self.uav_waiting_back) != 0:
            for uav in self.uav_waiting_back:
                uav_id = uav.sn
                
                if uav_id not in uav_back_id_set:
                    continue
                
                land_pos_id = uav_id_to_land_pos_id[uav_id]
                route = self.set_route_for_uav_back(uav_id, land_pos_id)
                # 将这个route 复制到uav id中
                # self.uav_id_to_route_now[uav_id] = route
                self.set_uav_to_route_dict(uav_id, route)
                
                cmd_res[uav_id] = {"cmd": "DELIVER", "route":route}
            
        # 让需要出发的飞机 立即起飞
        if len(self.uav_waiting_go) != 0:
            for uav in self.uav_waiting_go:
                uav_id = uav.sn
                
                if uav_id != self.take_off_uav_id:
                # if uav_id not in uav_go_id_set:
                    rospy.loginfo("uav is waiting to go! but not allowed to fly = " + str(uav_id))
                    continue
                
                unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
                
                route = self.set_route_for_uav_go(uav_id)
                # self.uav_id_to_route_now[uav_id] = route
                self.set_uav_to_route_dict(uav_id, route)

                cmd_res[uav_id] = {"cmd": "DELIVER", "route":route, 
                                   "unloading_station_id": unloading_station_id}
        
        if cmd_res != {}:
            rospy.loginfo("uav_cmd")
            rospy.loginfo(cmd_res)
        return cmd_res


    def find_neighbor_car_set(self):
        neighbor_car_set = set()
        dis_c = 0.6
        for key, car in self.car_info.items():
            car_id = key
            car_now_pos = car.pos
            if car_id in self.line_2_car_id_set or car_id in self.line_3_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.para.neighbor_pos)
                if dis_l < dis_c:
                    neighbor_car_set.add(car_id)
                               
        self.neighbor_car_set = neighbor_car_set

    def generate_line_points(self, startpos, endpos, spacing=1):
        # 生成离散的点
        # 提取x, y坐标
        x1, y1 = startpos[0], startpos[1]
        x2, y2 = endpos[0], endpos[1]
        # 计算总距离
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        # 计算需要生成的点的数量
        num_points = int(np.ceil(distance / spacing))
        # 生成离散点
        x_points = np.linspace(x1, x2, num_points)
        y_points = np.linspace(y1, y2, num_points)
        # 将x, y点组合成离散点列表
        points = list(zip(x_points, y_points))
        # for point in points:
        #     rospy.loginfo(point)
        return points

    def read_map_main(self):
        # 创建Map实例，假设地图文件路径为 "path/to/map/file"
        map_file_path = "voxel_map.bin"
        self.map_instance = pymtmap.Map(map_file_path)
        map_instance = self.map_instance

        # 检查地图是否有效
        if map_instance.IsValid():
            rospy.loginfo("Map is valid.")
            rospy.loginfo(f"Map boundaries: x({map_instance.min_x()} to {map_instance.max_x()}), "
                f"y({map_instance.min_y()} to {map_instance.max_y()}), "
                f"z({map_instance.min_z()} to {map_instance.max_z()})")
            
            self.map_max_x = map_instance.max_x()
            self.map_max_y = map_instance.max_y()
            self.map_max_z = map_instance.max_z()
            
            self.map_min_x = map_instance.min_x()
            self.map_min_y = map_instance.min_y()
            self.map_min_z = map_instance.min_z()
            
            # x, y, z = 1.0, 2.0, -222.0
            # voxel = map_instance.Query(x, y, z)
            
            # rospy.loginfo(f"Voxel at ({x}, {y}, {z}):")
            # rospy.loginfo(f"  Distance: {voxel.distance}")
            # rospy.loginfo(f"  Current Height to Ground: {voxel.cur_height_to_ground}")
            # rospy.loginfo(f"  Height to Ground: {voxel.height_to_ground}")
            # rospy.loginfo(f"  Semantic: {voxel.semantic}")
        else:
            rospy.loginfo("Map is not valid.")
            
    def path_search_easy(self, start_pos, end_pos):
        # 根据给出的起点和终点计算路径
        # 这个计算出来的路径 可能不在60-120高度之内 如果在的话 那么这个路径就是有效的
        # 如果不在60-120之内 就需要使用jps来搜索
        # 但是这个路径的好处是不需要转弯
        # start_pos = self.drone_station_pos
        # end_pos = self.cargo_pos[0]
        
        line_points = self.generate_line_points(start_pos, end_pos)
        # 通过暴力循环的方式 找到 路径安全高度
        path_satety_height = 0
        path_height_list = []
        for point in line_points:
            x, y = int(point[0]), int(point[1])
            for z in range(self.map_min_z, self.map_max_z):
                voxel = self.map_instance.Query(x, y, z)
                if voxel.semantic == 255:
                    continue   
                dis_n = voxel.distance
                # rospy.loginfo(f"{point} -> {dis_n}")
                if 10 >= dis_n >= 2:
                    path_height_list.append(z)
        
        # rospy.loginfo(path_height_list)
        path_satety_height = min(path_height_list)
        if path_satety_height >= self.safe_flight_min_height:
            path_satety_height = self.safe_flight_min_height
        
        rospy.loginfo(f"路径安全高度为：{path_satety_height}")
        
        path_res = []
        path_res.append(copy.copy(start_pos))
        start_one = [start_pos[0], start_pos[1], path_satety_height]
        end_one = [end_pos[0], end_pos[1], path_satety_height]
        path_res.append(start_one)
        path_res.append(end_one)
        path_res.append(copy.copy(end_pos))
        # for point in line_points:
        #     x, y, z = int(point[0]), int(point[1]), (path_satety_height)
        #     path_point = [x, y, z]
        #     path_res.append(path_point)
        
        path_res = np.array(path_res)
        return path_res
    
    
    def plot_map(self, path_list_go, path_list_back):
        rospy.loginfo("plot_map")
        # self.deal_with_map()
        # 地图边界
        x_min, x_max = 0, 700
        y_min, y_max = 0, 700
        z_min, z_max = -222, 0

        # 障碍物坐标列表
        # obstacles = self.map_obs

        # 创建一个3D图形
        fig = plt.figure(figsize = (12, 9))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制地图边界
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        # ax.set_zlim(z_min, z_max)
        ax.set_zlim(z_max, z_min)

        # 使用numpy数组一次性绘制所有障碍物
        # ax.scatter(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], color='r', marker='o', s=1)

        for path_res in path_list_go:
            # rospy.loginfo(path_res)
            path_res = np.array(path_res)
        # path_res = self.compute_path(self.drone_station_pos, self.cargo_pos[0])
            ax.plot3D(path_res[:, 0], path_res[:, 1], path_res[:, 2], color='b')

        for path_res in path_list_back:
            # rospy.loginfo(path_res)
            path_res = np.array(path_res)
        # path_res = self.compute_path(self.drone_station_pos, self.cargo_pos[0])
            ax.plot3D(path_res[:, 0], path_res[:, 1], path_res[:, 2], color='r')

        # 设置坐标轴标签
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')

        # 设置标题
        ax.set_title('3D Map with Obstacles')
        
        # plt.savefig("output.png")
        # 显示图形
        plt.show()
        
    def deal_with_map(self):
        rospy.loginfo("deal_with_map")
        # 使用步长为10代替5，减少遍历的点数
        for x in range(0, 700, 5):
            for y in range(0, 700, 5):
                for z in range(-222, 0, 5):
                    voxel = self.map_instance.Query(x, y, z)
                    if voxel.distance == 0 and voxel.semantic != 255:
                        self.map_obs.append([x, y, z])
        self.map_obs = np.array(self.map_obs)  # 转换为numpy数组

    def path_search_JPS(self, start_pos, end_pos, height):
        start_pos = (start_pos[0], start_pos[1])
        end_pos = (end_pos[0], end_pos[1])
        self.generate_matrix_map(height)
        path_res = self.jps_search(start_pos, end_pos)
        rospy.loginfo(path_res)
        path_out = []
        for p in path_res:
            pos = [p[0], p[1], height]
            path_out.append(pos)
        return path_out
    def jps_search(self, start_pos, end_pos):
        path_res = self.jps.method(self.map_matrix, start_pos, end_pos, 2)
        # self.plot_matrix(self.map_matrix, path_res)
        return path_res
    def generate_matrix_map(self, height):
        # 在当前高度下 生成一个二维矩阵matrix 0表示free 1表示obs
        matrix = np.zeros((self.map_max_x, self.map_max_y))
        for i in range(0, self.map_max_x):
            for j in range(0, self.map_max_y):
                voxel = self.map_instance.Query(i, j, height)
                # rospy.loginfo(voxel.distance)
                if i== 0 or j==0 or i == self.map_max_x-1 or j == self.map_max_y-1:
                    matrix[i, j] = 1
                elif voxel.distance >= 5 and voxel.semantic != 255:
                    matrix[i, j] = 0
                else:
                    matrix[i, j] = 1
        self.map_matrix = matrix
        self.plot_matrix(self.map_matrix)
        
    def plot_matrix(self, matrix):
        plt.figure(figsize=(10, 10))
        # plt.imshow(matrix, cmap='gray_r', interpolation='none')
        
        transposed_matrix = matrix.T
        plt.imshow(transposed_matrix, cmap='gray_r', interpolation='none')
        
        rectangle = plt.Rectangle((180, 420), 20, 30, linewidth=2, edgecolor='r', facecolor='none')
        plt.gca().add_patch(rectangle)
        
        # plt.plot([p[0] for p in path], [p[1] for p in path], 'b-o', linewidth=2)  # 使用蓝色线条和圆圈标记点
        
        plt.grid(color='red', linestyle='--', linewidth=1, alpha=0.5)
        
        plt.xlim(self.map_min_x-10, self.map_max_x+10)
        plt.ylim(self.map_min_y-10, self.map_max_y+10)
        plt.xlabel('X Axis')
        plt.ylabel('Y Axis')
        plt.axis()
        plt.show()        


    def dis_cal(self, des_pos, cur_pos):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur))

    def judge_car_situation_can_back(self, line_id):
        # 当前没有飞机 在line上 判断line是否有能力 接飞机
        # 如果当前line 没有飞机正在回来 
            # 如果没车在land pos 但是后面有车
        if self.car_land_pressure[line_id]['num'] == 0:
            # 如果有车在land pos直接回
            # if self.judge_one_car_is_on_landpos(line_id) is True:
            #     return True
            # 如果没车在land pos 但是后面有车正在来
            # if self.judge_one_car_is_comming(line_id) is True:
            #     return True
            # 如果没车在land pos 后面有车正在放飞飞机

            # 如果没车在land pos 后面有车正在移动出workspace

            # 如果没车在land pos 后面有车正在workspace上

            # 如果两个车都在wait gw 那就需要等待很长时间
            if self.judge_two_car_is_gw(line_id) is True:
                rospy.loginfo("two car is gw, need wait!")
                return False
            else:
                return True


    def determine_land_pos_back_time(self, line_id):
        # 根据当前的车运动的情况 决定飞机 选择当前回来的位置 的间隔时间
        # 之前的间隔时间是静态的 这次改成动态的
        # 这个考虑的case是已经有一架飞机要回来了
        # 后面还有一架飞机的间隔时间判定
        rospy.loginfo("line_id=" +str(line_id))
                 
        # 前面有一架飞机 land pos有车等待
        if self.car_land_pressure[line_id]['num'] == 1:
            if line_id != 1:
                if self.judge_line_two_car_ready(line_id) is True:
                    rospy.loginfo("后面有另一个在wait pos上等待")
                    rospy.logwarn("后面有另一个在wait pos上等待 line=" + str(line_id))
                    pos_time_gap = 12
                    return pos_time_gap    
                if self.judge_another_car_is_coming(line_id) is True:
                    rospy.loginfo("后面无车等待-另一个车已经放飞飞机 正在回来的路上")
                    rospy.logwarn("后面无车等待-另一个车已经放飞飞机 正在回来的路上 line=" + str(line_id))
                    pos_time_gap = 13
                    return pos_time_gap                    
                if self.judge_another_car_is_wait_uav_fly(line_id) is True:
                    rospy.loginfo("后面无车等待-车正在等待飞机放飞")
                    rospy.logwarn("后面无车等待-车正在等待飞机放飞 line=" + str(line_id))
                    pos_time_gap = 15
                    return pos_time_gap
                if self.judge_another_car_is_moving_out_workspace(line_id) is True:
                    rospy.loginfo("后面无车等待-正在移动出workspace")
                    rospy.logwarn("后面无车等待-正在移动出workspace line=" + str(line_id))
                    pos_time_gap = 20
                    return pos_time_gap
                if self.judge_another_car_is_on_workspace(line_id) is True:
                    rospy.loginfo("后面无车等待-车正在workspace")
                    rospy.logwarn("后面无车等待-车正在workspace line=" + str(line_id))
                    pos_time_gap = 30
                    return pos_time_gap               
                if self.judge_another_car_is_wait_gw(line_id) is True:
                    rospy.loginfo("后面无车等待-车还没进workspace")
                    rospy.logwarn("后面无车等待-车还没进workspace line=" + str(line_id))
                    pos_time_gap = self.determine_waiting_time(line_id)
                    return pos_time_gap                   

            if line_id == 1:
                pos_time_gap = 50
            elif line_id == 2:
                pos_time_gap = 60
            elif line_id == 3:
                pos_time_gap = 65
            # pos_time_gap = 999

            rospy.loginfo("前面if都跳过了")
            # rospy.logwarn("前面if都跳过了 line=" + str(line_id))            
            return pos_time_gap

    def determine_waiting_time(self, line_id):
        # 当line id的车处于gw状态时 判断前面大概要等待几个车
        nei_num = len(self.neighbor_car_set)
        car_1_id = self.line_1_car_id_list[0]
        car_2_id = self.line_1_car_id_list[1]
        if car_1_id in self.work_wait_pos_car_set or car_2_id in self.work_wait_pos_car_set:
            line1_num = 1
        else:
            line1_num = 0

        all_sum = nei_num + line1_num

        if line_id == 1:
            if nei_num == 0:
                pos_time_gap = 36
            else:
                pos_time_gap = 40
        elif line_id == 2:
            if all_sum == 2:
                pos_time_gap = 60
            elif all_sum == 1:
                pos_time_gap = 55
            elif all_sum == 0:
                pos_time_gap = 50
        elif line_id == 3:
            if all_sum == 2:
                pos_time_gap = 65
            elif all_sum == 1:
                pos_time_gap = 55
            elif all_sum == 0:
                pos_time_gap = 50
        
        return pos_time_gap

    def judge_car_in_gw_actual(self, car_id):
        # 判断处于gw状态
        # 有飞机但是没有货
        if car_id in self.car_waiting_go_gw_set:
            return True
        elif car_id in self.car_running_set:
            if self.car_info[car_id].have_uav is True:
                uav_id = self.car_info[car_id].uav_sn
                if self.uav_info[uav_id].have_cargo is False:
                    return True
        return False
    
    def judge_car_in_aw_actual(self, car_id):
        # 判断处于aw状态
        # 有飞机 有货
        if car_id in self.car_waiting_go_gw_set:
            return True
        elif car_id in self.car_running_set:
            if self.car_info[car_id].have_uav is True:
                uav_id = self.car_info[car_id].uav_sn
                if self.uav_info[uav_id].have_cargo is True:
                    return True
        return False        

    def judge_car_in_pickup_actual(self, car_id):
        # 判断处于pickup状态
        # 或者运动中 没有飞机
        if car_id in self.car_waiting_pickup_set:
            return True
        elif car_id in self.car_running_set:
            if self.car_info[car_id].have_uav is False:
                return True
        return False
    def judge_two_car_is_gw(self, line_id):
        # 两个车都处于gw的状态
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        if self.judge_car_in_gw_actual(car_id_a) is True \
        and self.judge_car_in_gw_actual(car_id_b) is True:
            rospy.loginfo("两个车都处于gw的状态")
            rospy.logwarn("两个车都处于gw的状态")
            return True
        else:
            return False        

    def judge_one_car_is_comming(self, line_id):
        # 没有车在land pos但是 另外有车正在过来
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]
        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3    
        # 一个车不在land pos 
        # 而另一车刚放飞飞机正在过来:y pos大于line y - 1，车已经处于wait pick up的状态
        if car_id_a not in self.land_pos_car_set \
        and self.judge_car_in_pickup_actual(car_id_b) is True \
        and car_b.pos.y > line_pos.y - 1:
            return True
        elif car_id_b not in self.land_pos_car_set \
        and self.judge_car_in_pickup_actual(car_id_a) is True \
        and car_a.pos.y > line_pos.y - 1:
            return True
        else:
            return False

    def judge_one_car_is_on_landpos(self, line_id):
        # 任意有一辆车在land pos上
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        if car_id_a in self.land_pos_car_set or car_id_b in self.land_pos_car_set:
            return True        
        else:
            return False

    def judge_another_car_is_wait_gw(self, line_id):
        # 后面无车等待-车还没进workspace
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        if car_id_a in self.land_pos_car_set and self.judge_car_in_gw_actual(car_id_b) is True \
            and car_id_a in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_a) + " in land pos, " + str(car_id_b) + "还没进工作区")
            rospy.logwarn(str(car_id_a) + " in land pos, " + str(car_id_b) + "还没进工作区")
            return True
        elif car_id_b in self.land_pos_car_set and self.judge_car_in_gw_actual(car_id_a) is True \
            and car_id_b in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_b) + " in land pos, " + str(car_id_a) + "还没进工作区")
            rospy.logwarn(str(car_id_b) + " in land pos, " + str(car_id_a) + "还没进工作区")
            return True
        else:
            return False        

    def judge_another_car_is_on_workspace(self, line_id):
        # 后面无车等待 另一个正在工作区
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        if car_id_a in self.land_pos_car_set and car_id_b in self.car_waiting_uav_work_set \
            and car_id_a in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_a) + " in land pos, " + str(car_id_b) + "在工作区")
            rospy.logwarn(str(car_id_a) + " in land pos, " + str(car_id_b) + "在工作区")
            return True
        elif car_id_b in self.land_pos_car_set and car_id_a in self.car_waiting_uav_work_set \
            and car_id_b in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_b) + " in land pos, " + str(car_id_a) + "在工作区")
            rospy.logwarn(str(car_id_b) + " in land pos, " + str(car_id_a) + "在工作区")
            return True
        else:
            return False
        
    def judge_another_car_is_moving_out_workspace(self, line_id):
        # 后面无车等待 另一个车正在移动出workspace
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]
        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3        

        # 一个车正在land pos 
        # 而另一车正在移动出workspace:y pos大于line y - 1，车已经处于waiting_go_aw_set的状态
        if car_id_a in self.land_pos_car_set and self.judge_car_in_aw_actual(car_id_b) is True \
            and self.para.work_station_pos.y < car_b.pos.y < line_pos.y - 1 \
            and car_id_a in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_a) + " in land pos, " + str(car_id_b) + " 正在移动出workspace")
            rospy.logwarn(str(car_id_a) + " in land pos, " + str(car_id_b) + " 正在移动出workspace")
            return True
        elif car_id_b in self.land_pos_car_set and self.judge_car_in_aw_actual(car_id_a) is True \
            and self.para.work_station_pos.y < car_a.pos.y < line_pos.y - 1 \
            and car_id_b in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_b) + " in land pos, " + str(car_id_a) + " 正在移动出workspace")
            rospy.logwarn(str(car_id_b) + " in land pos, " + str(car_id_a) + " 正在移动出workspace")
            return True
        else:
            return False

    def judge_another_car_is_wait_uav_fly(self, line_id):
        # 后面无车等待-另一个车已经放飞飞机 正在回来的路上
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]

        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3

        # 一个车正在land pos 
        # 而另一车正在等待飞机起飞:y pos大于line y - 1，车已经处于waiting_go_aw_set的状态
        if car_id_a in self.land_pos_car_set and car_id_b in self.car_waiting_go_aw_set \
            and car_b.pos.y > line_pos.y - 1 \
            and car_id_a in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_a) + " in land pos, " + str(car_id_b) + "等待飞机起飞")
            rospy.logwarn(str(car_id_a) + " in land pos, " + str(car_id_b) + "等待飞机起飞")
            return True
        elif car_id_b in self.land_pos_car_set and car_id_a in self.car_waiting_go_aw_set \
            and car_a.pos.y > line_pos.y - 1 \
            and car_id_b in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_b) + " in land pos, " + str(car_id_a) + "等待飞机起飞")
            rospy.logwarn(str(car_id_b) + " in land pos, " + str(car_id_a) + "等待飞机起飞")
            return True
        else:
            return False

    def judge_another_car_is_coming(self, line_id):
        # 后面无车等待-另一个车已经放飞飞机 正在回来的路上
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]

        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3

        # 一个车正在正在land pos 
        # 而另一车刚放飞飞机正在过来:y pos大于line y - 1，车已经处于wait pick up的状态
        if car_id_a in self.land_pos_car_set and self.judge_car_in_pickup_actual(car_id_b) is True \
            and car_b.pos.y > line_pos.y - 1 \
            and car_id_a in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_a) + " in land pos, " + str(car_id_b) + "刚放飞飞机过来")
            rospy.logwarn(str(car_id_a) + " in land pos, " + str(car_id_b) + "刚放飞飞机过来")
            return True
        elif car_id_b in self.land_pos_car_set and self.judge_car_in_pickup_actual(car_id_a) is True \
            and car_a.pos.y > line_pos.y - 1 \
            and car_id_b in self.car_waiting_pickup_set:
            rospy.loginfo(str(car_id_b) + " in land pos, " + str(car_id_a) + "刚放飞飞机过来")
            rospy.logwarn(str(car_id_b) + " in land pos, " + str(car_id_a) + "刚放飞飞机过来")
            return True
        else:
            return False

    def judge_line_two_car_ready(self, line_id):
        car_id_a, car_id_b = self.get_two_car_id(line_id)

        if car_id_a in self.land_pos_car_set and car_id_b in self.wait_pos_car_set and \
            car_id_a in self.car_waiting_pickup_set and car_id_b in self.car_waiting_pickup_set:
            rospy.logwarn("Two car is ready")
            rospy.loginfo("Two car is ready")
            return True
        elif car_id_b in self.land_pos_car_set and car_id_a in self.wait_pos_car_set \
            and car_id_a in self.car_waiting_pickup_set and car_id_b in self.car_waiting_pickup_set:
            rospy.logwarn("Two car is ready")
            rospy.loginfo("Two car is ready")
            return True
        else:
            return False

    def get_two_car_id(self, line_id):
        if line_id == 1:
            car_id_a = self.line_1_car_id_list[0]
            car_id_b = self.line_1_car_id_list[1]
        elif line_id == 2:
            car_id_a = self.line_2_car_id_list[0]
            car_id_b = self.line_2_car_id_list[1]
        elif line_id == 3:    
            car_id_a = self.line_3_car_id_list[0]
            car_id_b = self.line_3_car_id_list[1]  
        return car_id_a, car_id_b
    def find_land_wait_pos_car_set(self):
        land_pos_car_set = set()
        wait_pos_car_set = set()
        work_wait_pos_car_set = set()
        # 找到正处于land pos wait pos上 且处于car_waiting_pickup状态的车
        dis_c = 0.8
        for key, car in self.car_info.items():
            car_id = key
            car_now_pos = car.pos
            if  car_id in self.line_1_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.para.land_p1)
                dis_w = self.dis_cal(car_now_pos, self.para.wait_p1)
                dis_q = self.dis_cal(car_now_pos, self.para.work_wait_p1)
                
                if dis_l < dis_c:
                    land_pos_car_set.add(car_id)
                if dis_w < dis_c:
                    wait_pos_car_set.add(car_id)
                if dis_q < dis_c:
                    work_wait_pos_car_set.add(car_id)
                
            elif car_id in self.line_2_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.para.land_p2)
                dis_w = self.dis_cal(car_now_pos, self.para.wait_p2)
                dis_q = self.dis_cal(car_now_pos, self.para.work_wait_p2)
                
                if dis_l < dis_c:
                    land_pos_car_set.add(car_id)
                if dis_w < dis_c:
                    wait_pos_car_set.add(car_id)
                if dis_q < dis_c:
                    work_wait_pos_car_set.add(car_id)

            elif car_id in self.line_3_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.para.land_p3)
                dis_w = self.dis_cal(car_now_pos, self.para.wait_p3)
                dis_q = self.dis_cal(car_now_pos, self.para.work_wait_p3)

                if dis_l < dis_c:
                    land_pos_car_set.add(car_id)
                if dis_w < dis_c:
                    wait_pos_car_set.add(car_id)
                if dis_q < dis_c:
                    work_wait_pos_car_set.add(car_id)
                
            else:
                continue  
        # return land_pos_car_set, wait_pos_car_set, work_wait_pos_car_set
        self.land_pos_car_set = land_pos_car_set
        self.wait_pos_car_set = wait_pos_car_set
        self.work_wait_pos_car_set = work_wait_pos_car_set

# a1 = Air_traffic_scheduler()
# a1.init_flight_path()