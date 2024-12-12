import rospy
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pymtmap
import copy
import math
from para import Para
from enum import Enum

# from fsm.car_fsm import CarState
from race_demo.msg import PanoramicInfo
from race_demo.srv import QueryVoxel
from race_demo.msg import SelfCommand
from race_demo.msg import SelfUAVSwarm, SelfCarSwarm
from race_demo.msg import Position
from race_demo.msg import UserCmdResponse


class Car_traffic_scheduler:
    # 地面交通管理类
    
    def __init__(self):
        
        self.para = Para()
        
        # 无人机机场位置 和 上货的位置
        self.drone_station_origin = self.para.drone_station_origin
        # self.work_station_pos = [190, 425, -16]
        self.work_station_pos = self.para.work_station_pos
        self.drone_store_pos = self.para.drone_store_pos
        
        
        # 所有飞行路径的信息 
        # 根据路径id 查询 path 长短 去还是回的路 有哪些飞机分别在哪个位置
        self.flight_path_info = {}
        
        # 根据 卸货点id 找到去 和 回的路径id
        self.release_cargo_idx_to_path = {}
        
        # 降落点信息 有哪些降落点可以使用 
        self.land_point = {}
        
        # 起飞点信息 有哪些起飞点可以使用
        self.take_off_point = {}
        
        # 卸货点的位置
        self.release_cargo_pos = {}
        
        # 工作区现在是否有车 有车就是True 没有就是False
        self.work_station_free = True
        # 当前占据工作区的car id
        self.work_station_car_id = None
        
        # 临界区是否是free
        self.nei_pos_free = True
        self.nei_pos_car_id = None
        
        self.work_wait_p1_free = True
        self.work_wait_p1_car_id = None

        self.work_wait_p2_free = True
        self.work_wait_p2_car_id = None

        self.work_wait_p3_free = True
        self.work_wait_p3_car_id = None

        self.leave_work_station_car_id = None

        self.car_info = None

        self.init_info()

        self.take_off_dis_gap = 1.0

        self.time_scale = 0.8
        
    def init_info(self):
        
        # 初始化 空中管理类的信息
        self.take_off_p_1 = self.para.take_off_p_1
        self.take_off_p_2 = self.para.take_off_p_2
        self.take_off_p_3 = self.para.take_off_p_3
        # self.take_off_mid_p = self.para.take_off_mid_p 

        self.land_p1 = self.para.land_p1
        self.land_p2 = self.para.land_p2
        self.land_p3 = self.para.land_p3

        self.wait_p1  = self.para.wait_p1 
        self.wait_p2  = self.para.wait_p2 
        self.wait_p3  = self.para.wait_p3 

        self.work_wait_p1  = self.para.work_wait_p1
        self.work_wait_p2  = self.para.work_wait_p2 
        self.work_wait_p3  = self.para.work_wait_p3
        
        self.line_1_car_id_set  = self.para.line_1_car_id_set 
        self.line_2_car_id_set  = self.para.line_2_car_id_set 
        self.line_3_car_id_set  = self.para.line_3_car_id_set 
        
        self.line_1_car_id_list = self.para.line_1_car_id_list
        self.line_2_car_id_list = self.para.line_2_car_id_list
        self.line_3_car_id_list = self.para.line_3_car_id_list
                
        self.car_id = ['SIM-MAGV-0001', 'SIM-MAGV-0002', 'SIM-MAGV-0003',
                       'SIM-MAGV-0004', 'SIM-MAGV-0005', 'SIM-MAGV-0006']
        
        
        # # 标志等待位置是否free
        # self.take_off_p_car_id = None
        # self.take_off_p_free = True
        
        # self.take_off_wait_1_car_id = None
        # self.take_off_wait_1_free = True
        # self.take_off_wait_1_pos = self.para.take_off_wait_1
        
        # self.take_off_wait_2_car_id = None
        # self.take_off_wait_2_free = True
        # self.take_off_wait_2_pos = self.para.take_off_wait_2

    def update_uav_info(self, uav_info_dict, uav_ready,
                        uav_waiting_go, uav_waiting_back,
                        uav_flying_go, uav_flying_back,
                        uav_id_to_unloading_station_id):
        self.uav_info_dict = uav_info_dict
        self.uav_ready = uav_ready
        self.uav_waiting_go = uav_waiting_go
        self.uav_waiting_back = uav_waiting_back
        self.uav_flying_go = uav_flying_go
        self.uav_flying_back = uav_flying_back
        self.uav_id_to_unloading_station_id = uav_id_to_unloading_station_id
        

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
        # rospy.loginfo("car_info_dict update")
        # rospy.loginfo(car_waiting_pickup)
        # rospy.loginfo(car_running)
        # rospy.loginfo(car_waiting_go_aw)
        # rospy.loginfo(car_waiting_go_gw)
        # rospy.loginfo(car_waiting_uav_work)
        # rospy.loginfo(car_info_dict)

    def get_info_from_air(self, uav_go_flight_time):
        self.uav_go_flight_time = uav_go_flight_time

    def judge_work_wait_p3_is_free(self):
        work_wait_p3 = self.para.work_wait_p3
        for car in self.car_info.values():
            car_pos = car.pos
            dis_n = self.dis_cal(car_pos, work_wait_p3)
            if dis_n <= 0.6:
                return False 
        if self.work_wait_p3_car_id is not None:
            car_id = self.work_wait_p3_car_id
            car_pos = self.car_info[car_id].pos
            if car_pos.y < self.para.work_wait_p3.y - 0.1:
                dis_n = self.dis_cal(car_pos, work_wait_p3)
                if dis_n > 0.6:
                    self.work_wait_p3_car_id = None
                return False
            else:
                pass
        
        return True

    def judge_work_wait_p2_is_free(self):
        work_wait_p2 = self.para.work_wait_p2
        for car in self.car_info.values():
            car_pos = car.pos
            dis_n = self.dis_cal(car_pos, work_wait_p2)
            if dis_n <= 0.6:
                return False 
        if self.work_wait_p2_car_id is not None:
            car_id = self.work_wait_p2_car_id
            car_pos = self.car_info[car_id].pos
            if car_pos.y < self.para.work_wait_p2.y - 0.1:
                dis_n = self.dis_cal(car_pos, work_wait_p2)
                if dis_n > 0.6:
                    self.work_wait_p2_car_id = None
                return False
            else:
                pass
        
        return True

    def judge_work_wait_p1_is_free(self):
        work_wait_p1 = self.para.work_wait_p1
        for car in self.car_info.values():
            car_pos = car.pos
            dis_n = self.dis_cal(car_pos, work_wait_p1)
            if dis_n <= 0.6:
                return False 
        if self.work_wait_p1_car_id is not None:
            car_id = self.work_wait_p1_car_id
            car_pos = self.car_info[car_id].pos
            if car_pos.x > self.para.work_wait_p1.x + 0.1 and \
                car_pos.y < self.para.work_wait_p1.y - 0.1:
                dis_n = self.dis_cal(car_pos, work_wait_p1)
                if dis_n > 0.6:
                    self.work_wait_p1_car_id = None
                return False
            else:
                pass
        
        return True


    def judge_nei_pos_is_free(self):
        nei_pos = self.para.neighbor_pos
        
        for car in self.car_info.values():
            car_pos = car.pos
            dis_n = self.dis_cal(car_pos, nei_pos)
            if dis_n <= 0.6:
                return False 

        if self.nei_pos_car_id is not None:
            # 如果有car指定要进来 
            # rospy.loginfo('car is cmd to come! nei pos is occupied')
            car_id = self.nei_pos_car_id
            car_pos = self.car_info[car_id].pos
            # 进来的这个车 有飞机 并且上货 还走远了 那么就意味着可以进下一个车
            if car_pos.x > self.para.neighbor_pos.x + 0.1:
                dis_n = self.dis_cal(car_pos, nei_pos)
                if dis_n > 0.6:
                    self.nei_pos_car_id = None
            return False
        else:
            pass
        
        return True        
        
    
    def judge_car_ls_leave(self):
        if self.leave_work_station_car_id is not None:
            car = self.car_info[self.leave_work_station_car_id]
            dis_gap = 1
            work_pos = self.work_station_pos        
            dis_n = self.dis_cal(car.pos, work_pos)
            if dis_n > dis_gap:
                self.leave_work_station_car_id = None
                rospy.loginfo("回收uav的车 离开工作区")
            
    def Judge_work_station_is_free(self):
        # rospy.loginfo("self.work_station_car_id")
        # rospy.loginfo(self.work_station_car_id)
        dis_gap = 0.6

        work_pos = self.work_station_pos
        for car in self.car_info.values():
            car_pos = car.pos
            dis_n = self.dis_cal(car_pos, work_pos)
            if dis_n <= dis_gap:
                return False
        
        if self.work_station_car_id is not None:
            # 如果有car指定要进来 
            # rospy.loginfo('car is cmd to come! work station is occupied')
            car_id = self.work_station_car_id
            car_pos = self.car_info[car_id].pos
            dis_n = self.dis_cal(car_pos, work_pos)
            if dis_n > dis_gap and car_pos.x > self.work_station_pos.x + 0.6:
                self.work_station_car_id = None
            
            rospy.loginfo("work station is not free car_id=" + str(self.work_station_car_id))
            return False
        else:
            pass
        
        rospy.loginfo("work station is free")
        return True
    
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
                dis_l = self.dis_cal(car_now_pos, self.land_p1)
                dis_w = self.dis_cal(car_now_pos, self.wait_p1)
                dis_q = self.dis_cal(car_now_pos, self.work_wait_p1)
                
                if dis_l < dis_c:
                    land_pos_car_set.add(car_id)
                if dis_w < dis_c:
                    wait_pos_car_set.add(car_id)
                if dis_q < dis_c:
                    work_wait_pos_car_set.add(car_id)
                
            elif car_id in self.line_2_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.land_p2)
                dis_w = self.dis_cal(car_now_pos, self.wait_p2)
                dis_q = self.dis_cal(car_now_pos, self.work_wait_p2)
                
                if dis_l < dis_c:
                    land_pos_car_set.add(car_id)
                if dis_w < dis_c:
                    wait_pos_car_set.add(car_id)
                if dis_q < dis_c:
                    work_wait_pos_car_set.add(car_id)

            elif car_id in self.line_3_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.land_p3)
                dis_w = self.dis_cal(car_now_pos, self.wait_p3)
                dis_q = self.dis_cal(car_now_pos, self.work_wait_p3)

                if dis_l < dis_c:
                    land_pos_car_set.add(car_id)
                if dis_w < dis_c:
                    wait_pos_car_set.add(car_id)
                if dis_q < dis_c:
                    work_wait_pos_car_set.add(car_id)
                
            else:
                continue  
        rospy.loginfo('land_pos_car_set')
        rospy.loginfo(land_pos_car_set)
        rospy.loginfo('wait_pos_car_set')
        rospy.loginfo(wait_pos_car_set)
        rospy.loginfo('work_wait_pos_car_set')
        rospy.loginfo(work_wait_pos_car_set)
        self.land_pos_car_set = land_pos_car_set
        self.wait_pos_car_set = wait_pos_car_set
        self.work_wait_pos_car_set = work_wait_pos_car_set
        return land_pos_car_set, wait_pos_car_set, work_wait_pos_car_set
    
    def find_neighbor_car_set(self):
        # 找到line2 3上 且处于nei pos的car
        neighbor_car_set = set()
        dis_c = 0.6
        for key, car in self.car_info.items():
            car_id = key
            car_now_pos = car.pos
            if car_id in self.line_2_car_id_set or car_id in self.line_3_car_id_set:
                dis_l = self.dis_cal(car_now_pos, self.para.neighbor_pos)
                if dis_l < dis_c:
                    neighbor_car_set.add(car_id)
                    
        rospy.loginfo('neighbor_car_set')
        rospy.loginfo(neighbor_car_set)                
        return neighbor_car_set
    
    def find_take_off_pos_1_car(self):
        for car in self.car_waiting_pickup:
            car_id = car.sn
            if car_id in self.line_1_car_id_set:
                car_now_pos = car.pos
                dis = self.dis_cal(car_now_pos, self.take_off_p_1)
                if dis < self.take_off_dis_gap:
                    # rospy.loginfo("find_take_off" + str(car_id))
                    return car_id
        return None

    def find_take_off_pos_2_car(self):
        for car in self.car_waiting_pickup:
            car_id = car.sn
            if car_id in self.line_2_car_id_set:
                car_now_pos = car.pos
                dis = self.dis_cal(car_now_pos, self.take_off_p_2)
                if dis < self.take_off_dis_gap:
                    # rospy.loginfo("find_take_off" + str(car_id))
                    return car_id
        return None

    def find_take_off_pos_3_car(self):
        for car in self.car_waiting_pickup:
            car_id = car.sn
            if car_id in self.line_3_car_id_set:
                car_now_pos = car.pos
                dis = self.dis_cal(car_now_pos, self.take_off_p_3)
                if dis < self.take_off_dis_gap:
                    # rospy.loginfo("find_take_off" + str(car_id))
                    return car_id
        return None


    def judge_wait_take_off_pos(self):
        # 如果此时pos被car 指定要来 那么就是False
        # 如果car要离开 car id 赋值为none 此时就看dis
        if self.take_off_wait_1_car_id is not None:
            
            self.take_off_wait_1_free = False
        else:
            for car in self.car_waiting_go_aw:
                car_id = car.sn
                car_now_pos = car.pos
                dis = self.dis_cal(car_now_pos, self.take_off_wait_1)        
                if dis > 0.5:
                    self.take_off_wait_1_free = True
        
        if self.take_off_wait_2_car_id is not None:
            
            self.take_off_wait_2_free = False
        else:
            for car in self.car_waiting_go_aw:
                car_id = car.sn
                car_now_pos = car.pos
                dis = self.dis_cal(car_now_pos, self.take_off_wait_2)        
                if dis > 0.5:
                    self.take_off_wait_2_free = True

        # 对于起飞点的判断是一样的
        if self.take_off_p_car_id is not None:
            self.take_off_p_free = False
        else:
            for car in self.car_waiting_pickup:
                car_id = car.sn
                car_now_pos = car.pos
                dis = self.dis_cal(car_now_pos, self.take_off_p)
                if dis > 0.5:
                    self.take_off_p_free = True
    
    def judge_have_wait_car(self, car_id_n, wait_pos_car_set):
        if car_id_n in self.line_1_car_id_set:
            for id in self.line_1_car_id_set:
                if id != car_id_n:
                    id_other = id
                    if id_other in wait_pos_car_set:
                        return True
                    else:
                        return False 
        elif car_id_n in self.line_2_car_id_set:
            for id in self.line_2_car_id_set:
                if id != car_id_n:
                    id_other = id
                    if id_other in wait_pos_car_set:
                        return True
                    else:
                        return False            
        elif car_id_n in self.line_3_car_id_set:
            for id in self.line_3_car_id_set:
                if id != car_id_n:
                    id_other = id
                    if id_other in wait_pos_car_set:
                        return True
                    else:
                        return False            
        else:
            return False
    
    def get_land_info(self, car_land_pressure, unloading_cargo_pressure):    
        self.car_land_pressure = car_land_pressure
        self.unloading_cargo_pressure = unloading_cargo_pressure

    def get_cargo_info(self, unloading_station_id):
        self.unload_station_id = unloading_station_id

    def car_have_landing_uav(self, car_id): 
        if car_id in self.line_1_car_id_set:
            num = self.car_land_pressure[1]['num']
            uav_time = self.car_land_pressure[1]['uav_time']
            if num == 0:
                return False
            for key, value in uav_time.items():
                if value < self.para.car_have_landing_uav_time_p1:
                    return True
            return False
            
        elif car_id in self.line_2_car_id_set:
            num = self.car_land_pressure[2]['num']
            uav_time = self.car_land_pressure[2]['uav_time']
            if num == 0:
                return False
            for key, value in uav_time.items():
                if value < self.para.car_have_landing_uav_time_p2:
                    return True
            return False
            
        elif car_id in self.line_3_car_id_set:
            num = self.car_land_pressure[3]['num']
            uav_time = self.car_land_pressure[3]['uav_time']
            if num == 0:
                return False
            for key, value in uav_time.items():
                if value < self.para.car_have_landing_uav_time_p3:
                    return True     
            return False
        else:
            return False
    
    def choose_urgent_car_to_go_nei(self, work_wait_pos_car_set):
        # 选择line2 3中紧急的车 进入到 nei pos
        # 选择接到飞机中 最紧急的车去
        car_land_pressure = {}
        for car in self.car_waiting_go_gw:
            car_id = car.sn  
            if car_id in self.line_2_car_id_set and car_id in work_wait_pos_car_set:
                car_land_pressure[car_id] = self.car_land_pressure[2]
                for uav_id, value in car_land_pressure[car_id]['uav_time'].items():
                    car_land_pressure[car_id]['uav_time'][uav_id] -= 5

            elif car_id in self.line_3_car_id_set and car_id in work_wait_pos_car_set:
                car_land_pressure[car_id] = self.car_land_pressure[3]
                for uav_id, value in car_land_pressure[car_id]['uav_time'].items():
                    car_land_pressure[car_id]['uav_time'][uav_id] -= 8


        # 找到所有uav_time中的最小值
        all_uav_times = [time for uav_times in car_land_pressure.values() \
                        for time in uav_times['uav_time'].values()]
        
        if len(all_uav_times) == 0:
            rospy.loginfo("all_uav_times is None --- choose car")
            for car in self.car_waiting_go_gw:
                car_id = car.sn 
                if car_id in self.line_2_car_id_set and car_id in work_wait_pos_car_set:
                    return car_id
                elif car_id in self.line_3_car_id_set and car_id in work_wait_pos_car_set:
                    return car_id
        
        min_uav_time = min(all_uav_times)

        # 找到最小uav_time对应的key
        min_key = None
        for key, value in car_land_pressure.items():
            if min_uav_time in value['uav_time'].values():
                if min_key is None or min_uav_time < min((value['uav_time'][k] \
                                                        for k in value['uav_time'])):
                    min_key = key
        
        if min_key == None:
            rospy.loginfo("e2 is None --- choose car")
            min_key = self.car_waiting_go_gw[0].sn 
        rospy.loginfo(f" urgent_car_to_go_nei is: {min_key}")
        return min_key
    
    def choose_urgent_car_to_go_work_station(self, work_wait_pos_car_set, nei_pos_car_set):
        # 选择最紧急的车 进入到工作台
        # 对于line1 要在work wait pos上
        # 对于line 23 要在nei pos上
        car_land_pressure = {}
        for car in self.car_waiting_go_gw:
            car_id = car.sn
            if car_id in work_wait_pos_car_set and car_id in self.line_1_car_id_set:
                car_land_pressure[car_id] = self.car_land_pressure[1]
            elif car_id in nei_pos_car_set:
                if car_id in self.line_2_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[2]
                    for uav_id, value in car_land_pressure[car_id]['uav_time'].items():
                        car_land_pressure[car_id]['uav_time'][uav_id] -= 5
                elif car_id in self.line_3_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[3]
                    for uav_id, value in car_land_pressure[car_id]['uav_time'].items():
                        car_land_pressure[car_id]['uav_time'][uav_id] -= 8

        # 找到所有uav_time中的最小值
        all_uav_times = [time for uav_times in car_land_pressure.values() \
                        for time in uav_times['uav_time'].values()]
        
        if len(all_uav_times) == 0:
            rospy.loginfo("all_uav_times is None --- choose car")
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                if car_id in work_wait_pos_car_set and car_id in self.line_1_car_id_set:
                    return car_id
                elif car_id in nei_pos_car_set and (car_id in self.line_2_car_id_set or \
                                                    car_id in self.line_3_car_id_set):
                    return car_id
        
        min_uav_time = min(all_uav_times)
        # 找到最小uav_time对应的key
        min_key = None
        for key, value in car_land_pressure.items():
            if min_uav_time in value['uav_time'].values():
                if min_key is None or min_uav_time < min((value['uav_time'][k] \
                                                        for k in value['uav_time'])):
                    min_key = key
        
        if min_key == None:
            rospy.loginfo("e3 is None --- choose car")
            min_key = self.car_waiting_go_gw[0].sn 
        rospy.loginfo(f"选择最紧急的车 进入到工作台 car is: {min_key}")
        return min_key
    
    def choose_have_uav_wait_car(self, work_wait_pos_car_set):
        # 选择接到飞机中 最紧急的车去
        if len(self.car_waiting_go_gw) == 1:
            return  self.car_waiting_go_gw[0].sn      
        else:
            car_land_pressure = {}
            for car in self.car_waiting_go_gw:
                
                # 有的车接到了飞机 但是并没有处于work wait pos的位置上
                car_id = car.sn
                if car_id not in work_wait_pos_car_set:
                    continue
                if car_id in self.line_1_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[1]
                elif car_id in self.line_2_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[2]
                elif car_id in self.line_3_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[3]
            
            
            # 找到所有uav_time中的最小值
            all_uav_times = [time for uav_times in car_land_pressure.values() \
                            for time in uav_times['uav_time'].values()]
            
            if len(all_uav_times) == 0:
                min_key = self.car_waiting_go_gw[0].sn 
                return min_key
            
            min_uav_time = min(all_uav_times)

            # 找到最小uav_time对应的key
            min_key = None
            for key, value in car_land_pressure.items():
                if min_uav_time in value['uav_time'].values():
                    if min_key is None or min_uav_time < min((value['uav_time'][k] \
                                                            for k in value['uav_time'])):
                        min_key = key
        
        if min_key == None:
            rospy.loginfo(" is None --- choose car")
            min_key = self.car_waiting_go_gw[0].sn 
        rospy.loginfo(f" The choose car is: {min_key}")
        return min_key
    
    def find_line_another_car_id(self, car_id):
        # 返回line中给定car id的另一个id
        line_num = self.judge_car_on_line_number(car_id)
        if line_num == 1:
            if car_id == self.line_1_car_id_list[0]:
                return self.line_1_car_id_list[1]
            else:
                return self.line_1_car_id_list[0]
        elif line_num == 2:
            if car_id == self.line_2_car_id_list[0]:
                return self.line_2_car_id_list[1]
            else:
                return self.line_2_car_id_list[0]
        elif line_num == 3:
            if car_id == self.line_3_car_id_list[0]:
                return self.line_3_car_id_list[1]
            else:
                return self.line_3_car_id_list[0]
        else:
            return None
        
    def judge_gw_car_on_work_wait_pos(self, work_wait_pos_car_set):
        # 判断是否有处于gw状态的车 且正在 wait work pos上
        if len(self.car_waiting_go_gw) == 0:
            return False
        else:
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                if car_id in work_wait_pos_car_set:
                    return True
    def judge_gw_car_on_work_wait_pos_or_nei_pos(self, work_wait_pos_car_set, nei_pos_car_set):
        # 判断是否有处于gw状态的车 line1且正在 wait work pos上 line23 位于nei pos上
        if len(self.car_waiting_go_gw) == 0:
            return False
        else:
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                if car_id in work_wait_pos_car_set:
                    if car_id in self.line_1_car_id_set:
                        return True        
                if car_id in nei_pos_car_set:
                    if car_id in self.line_2_car_id_set or car_id in self.line_3_car_id_set:
                        return True
            return False
        
    def judge_pickup_car_on_work_wait_pos(self, work_wait_pos_car_set):
        # 判断是否有处于pick up状态的车 且正在 wait work pos上
        if len(self.car_waiting_pickup) == 0:
            return False
        else:
            for car in self.car_waiting_pickup:
                car_id = car.sn
                if car_id in work_wait_pos_car_set:
                    return True        
    
    def find_pickup_car_on_land_pos(self, land_pos_car_set):
        if len(self.car_waiting_pickup) == 0:
            return None
        else:
            for car in self.car_waiting_pickup:
                car_id = car.sn
                if car_id in land_pos_car_set and car_id in self.line_1_car_id_set:
                    return car_id

    def choose_route_for_car_go_neighbor_pos(self, car_id):
        car = self.car_info[car_id]
        car_now_pos = car.pos
        car_goal_pos = self.para.neighbor_pos
        
        mid   = Position(1 + 180, 15 + 420, -16)
        mid_0 = Position(1 + 180, 10 + 420, -16)
        mid_1 = Position(2 + 180, 2 + 420, -16)
        # mid_2 = Position(9 + 180, 2 + 420, -16)

        if car_id in self.line_2_car_id_set:
            route = [car_now_pos, mid, mid_0, mid_1, car_goal_pos]
        elif car_id in self.line_3_car_id_set:
            route = [car_now_pos, mid, mid_0, mid_1, car_goal_pos]
        else:
            rospy.loginfo("route is None")
            return []
        
        return route

    def choose_route_for_car_go_workstation(self, car_id):
        car = self.car_info[car_id]
        car_now_pos = car.pos
        car_goal_pos = self.work_station_pos
        
        mid  = Position(10 + 180, 2 + 420, -16)

        if car_id in self.line_1_car_id_set :
            route = [car_now_pos, car_goal_pos]
        elif car_id in self.line_2_car_id_set:
            route = [car_now_pos, mid, car_goal_pos]
        elif car_id in self.line_3_car_id_set:
            route = [car_now_pos, mid, car_goal_pos]
        else:
            rospy.loginfo("route is None")
            return []
        
        return route
    
    def judge_car_on_line_number(self, car_id):
        if car_id in self.line_1_car_id_set:
            return 1
        elif car_id in self.line_2_car_id_set:
            return 2
        elif car_id in self.line_3_car_id_set:
            return 3    
    

    def judge_line23car_on_work_wait_pos(self, work_wait_pos_car_set):
        # 判断是否有line23的车 在 work wait pos上
        if len(self.car_waiting_go_gw) == 0:
            return False
        else:
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                if car_id in work_wait_pos_car_set:
                    if car_id in self.line_2_car_id_set or car_id in self.line_3_car_id_set:
                        return True
            return False
    
    def judge_car_is_going_from_work_wait_pos_to_neighbor_pos(self, car_id_n):
        rospy.loginfo("judge_car_is_going_from_work_wait_pos_to_neighbor_pos")
        # 判断line23上 是否有小车正在从work wait pos到neighbor pos的这条线上
        car_list_23 = self.line_2_car_id_list + self.line_3_car_id_list
        redund = 0.7
        
        x_right = self.para.neighbor_pos.x + redund
        x_left = self.work_wait_p3.x - redund

        y_up = self.work_wait_p3.y + redund - 1
        y_down = self.para.neighbor_pos.y - redund
        
        bool_obs = False
        
        for car_id in car_list_23:
            if car_id == car_id_n:
                continue
            
            car = self.car_info[car_id]
            car_pos = car.pos
            if x_left <= car_pos.x <= x_right and y_down <= car_pos.y <= y_up:
                rospy.loginfo("car_id=" + str(car_id) + " is going from work wait pos to neighbor pos")
                # rospy.logwarn("car_id=" + str(car_id) + " is going from work wait pos to neighbor pos")
                bool_obs =  True
                break
        
        rospy.loginfo("car_id_n= "+ str(car_id_n))
        if bool_obs:
            rospy.loginfo("None car is going from work wait pos to neighbor pos")
            return True
        else:
            return False
        
    def judge_car_is_going_from_land_pos_to_work_wait_pos(self, car_id):
        # 从land pos到work wait pos的这条线上是否有car 
        # 有就返回True 没有就返回False
        another_car_id = self.find_line_another_car_id(car_id)
        line_number = self.judge_car_on_line_number(car_id)
        
        another_car = self.car_info[another_car_id]
        another_car_pos = another_car.pos
        
        redund = 0.5
        
        rospy.loginfo("car_id=" + str(car_id))
        rospy.loginfo("another_car_id=" + str(another_car_id))
        rospy.loginfo("another_car_pos=" + str(another_car_pos))
        
        if line_number == 1:
            x_right = self.land_p1.x + redund
            x_left = self.work_wait_p1.x - redund
            y_up = self.land_p1.y + redund
            y_down = self.work_wait_p1.y - redund
        elif line_number == 2:
            x_right = self.land_p2.x + redund
            x_left = self.work_wait_p2.x - redund
            y_up = self.land_p2.y + redund
            y_down = self.work_wait_p2.y - redund
        elif line_number == 3:                
            x_right = self.land_p3.x + redund
            x_left = self.work_wait_p3.x - redund
            y_up = self.land_p3.y + redund
            y_down = self.work_wait_p3.y - redund
        
        rospy.loginfo("x_right=" + str(x_right))
        rospy.loginfo("x_left=" + str(x_left))
        rospy.loginfo("y_up=" + str(y_up))
        rospy.loginfo("y_down=" + str(y_down))
        rospy.loginfo("judge=" + str(x_left <= another_car_pos.x <= x_right \
                                and y_down <= another_car_pos.y <= y_up))
            
        if x_left <= another_car_pos.x <= x_right \
        and y_down <= another_car_pos.y <= y_up:
            rospy.logerr(str(another_car_id)+ " is going from land pos to work wait pos")
            rospy.loginfo(str(another_car_id)+ " is going from land pos to work wait pos")
            return True
        else:
            return False


    def run(self):
        cmd_res = {}
        land_pos_car_set, wait_pos_car_set, work_wait_pos_car_set \
              = self.find_land_wait_pos_car_set()
        nei_car_set = self.find_neighbor_car_set()
        take_off_car_id_1 = self.find_take_off_pos_1_car()
        take_off_car_id_2 = self.find_take_off_pos_2_car()
        take_off_car_id_3 = self.find_take_off_pos_3_car()        
        self.work_station_free = self.Judge_work_station_is_free()
        self.nei_pos_free = self.judge_nei_pos_is_free()
        self.work_wait_p1_free = self.judge_work_wait_p1_is_free()
        self.work_wait_p2_free = self.judge_work_wait_p2_is_free()
        self.work_wait_p3_free = self.judge_work_wait_p3_is_free()
        # rospy.loginfo("work is free = " + str(self.work_station_free))
        self.find_car_info_set()
        self.judge_car_ls_leave()
        self.update_car_uav_cargo_info()

        rospy.loginfo("剩余ready飞机数量=" + str(len(self.uav_ready)))

        rospy.loginfo("leave_work_station_car_id=" + str(self.leave_work_station_car_id))
        if self.leave_work_station_car_id is not None:
            rospy.loginfo("回收飞机后 立即离开工作区 leave_work_station_car_id=" + str(self.leave_work_station_car_id))
            car_id = self.leave_work_station_car_id
            car_now_pos = self.car_info[car_id].pos   
            # 对于line1来说 需要判断一下 take off pos是否有飞机
            if car_id in self.line_1_car_id_set:
                another_car_id = self.find_line_another_car_id(car_id)
                mid_1 = Position(15  + 180, 5 + 420, -16)
                car_goal_pos  = self.take_off_p_1
                route = [car_now_pos, mid_1, car_goal_pos]
                if self.car_info[another_car_id].have_uav is True \
                and another_car_id in work_wait_pos_car_set:
                    mid_2 = Position(12  + 180, 5 + 420, -16)
                    route = [car_now_pos, mid_2, self.para.take_off_p_1]
                    rospy.loginfo("另一车上有飞机 但是在work wait pos")
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
                elif self.car_info[another_car_id].pos.x < (180 + 13) and \
                self.car_info[another_car_id].have_uav is False:
                    # rospy.loginfo("pos=" + str(self.car_info[another_car_id].pos))
                    rospy.loginfo("line1 上 take off pos空了 后面的车 去 起飞点")
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
                elif another_car_id in land_pos_car_set:
                    rospy.loginfo("line1 车去降落点了 后面的车 去 起飞点")
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}                     

            elif car_id in self.line_2_car_id_set:
                # another_car_id = self.find_line_another_car_id(car_id)
                # if self.car_info[another_car_id].have_uav is True:
                #     uav_id = self.car_info[another_car_id].uav_sn
                #     if self.uav_info_dict[uav_id].have_cargo is False:
                #         rospy.loginfo("line 2 直接去land pos")
                #         mid_1 = Position(19  + 180, 5 + 420, -16)
                #         mid_2 = Position(19  + 180, 17 + 420, -16)
                #         car_goal_pos  = self.land_p2
                #         route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                #         cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}                          
                # else:
                bool_can_go = True
                another_car_id = self.find_line_another_car_id(car_id)
                # 判断一下 另一个车有飞机 且飞机有货 就不能出发
                if self.car_info[another_car_id].have_uav is True:
                    uav_id = self.car_info[another_car_id].uav_sn
                    if self.uav_info_dict[uav_id].have_cargo is True:
                        bool_can_go = False
                if bool_can_go is True:
                    rospy.loginfo("line 2 的去takeoff pos")
                    mid_1 = Position(19  + 180, 5 + 420, -16)
                    mid_2 = Position(19  + 180, 15 + 420, -16)
                    car_goal_pos  = self.take_off_p_2
                    route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}  
            elif car_id in self.line_3_car_id_set:
                # another_car_id = self.find_line_another_car_id(car_id)
                # if self.car_info[another_car_id].have_uav is True:
                #     uav_id = self.car_info[another_car_id].uav_sn
                #     if self.uav_info_dict[uav_id].have_cargo is False:
                #         rospy.loginfo("line 3 直接去land pos")
                #         mid_1 = Position(19  + 180, 5 + 420, -16)
                #         mid_2 = Position(19  + 180, 25 + 420, -16)
                #         car_goal_pos  = self.land_p3
                #         route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                #         cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}                 
                # else:
                rospy.loginfo("line 3 的去takeoff pos")
                mid_1 = Position(19  + 180, 5 + 420, -16)
                mid_2 = Position(19  + 180, 19 + 420, -16)
                car_goal_pos  = self.take_off_p_3
                route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 


        # 1-飞机起飞后车回到 wait pos
        if take_off_car_id_1 is not None:
            rospy.loginfo("飞机起飞后车回到 wait pos")
            car_id = take_off_car_id_1
            car_now_pos = self.car_info[car_id].pos
            if car_id in self.line_1_car_id_set and self.car_info[car_id].have_uav is False:
                # mid_1 = Position(1  + 180, 24.5 + 420, -16)
                rospy.loginfo("飞机已经起飞 回到wait pos")
                car_goal_pos = self.wait_p1
                route = [car_now_pos, car_goal_pos]
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 

        if take_off_car_id_2 is not None:
            car_id = take_off_car_id_2
            car_now_pos = self.car_info[car_id].pos
            if car_id in self.line_2_car_id_set and self.car_info[car_id].have_uav is False:
                # mid_1 = Position(6  + 180, 21 + 420, -16)
                car_goal_pos = self.wait_p2
                # route = [car_now_pos, mid_1, car_goal_pos]
                route = [car_now_pos, car_goal_pos]
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
        
        if take_off_car_id_3 is not None:
            car_id = take_off_car_id_3
            car_now_pos = self.car_info[car_id].pos
            if car_id in self.line_3_car_id_set and self.car_info[car_id].have_uav is False:
                car_goal_pos = self.wait_p3
                route = [car_now_pos, car_goal_pos] 
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}  
        
        # 2- 每一个line上 如果land pos上车空了 那么处于wait pos上的车 马上去到land pos的位置
        car_id_list_1 = self.line_1_car_id_list
        # line1 上 land pos空了 后面的车 补上
        # 对于line1 而言 land pos 和work wait pos都空了 后面的车才能上前
        # 如果car 0位于 wait pos上 且 car 1 既不在land pos也不在work wait pos上
        if car_id_list_1[0] in wait_pos_car_set and \
                car_id_list_1[1] not in land_pos_car_set:
            # rospy.loginfo("line1 上 land pos空了 后面的车 补上")
            car_id = car_id_list_1[0] 
            car_now_pos = self.car_info[car_id].pos
            car_goal_pos = self.land_p1
            route = [car_now_pos, car_goal_pos]
            cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}    
        elif car_id_list_1[1] in wait_pos_car_set and \
                car_id_list_1[0] not in land_pos_car_set:
            # rospy.loginfo("line1 上 land pos空了 后面的车 补上")
            car_id = car_id_list_1[1]  
            car_now_pos = self.car_info[car_id].pos
            car_goal_pos = self.land_p1
            route = [car_now_pos, car_goal_pos]
            cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}              

        car_id_list_2 = self.line_2_car_id_list
        # line2 上 land pos空了 后面的车 补上
        if car_id_list_2[0] in wait_pos_car_set and car_id_list_2[1] not in land_pos_car_set:
            # rospy.loginfo("line2 上 land pos空了 后面的车 补上")
            car_id = car_id_list_2[0] 
            car_now_pos = self.car_info[car_id].pos
            car_goal_pos = self.land_p2
            route = [car_now_pos, car_goal_pos]
            cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}    
        elif car_id_list_2[1] in wait_pos_car_set and car_id_list_2[0] not in land_pos_car_set:
            # rospy.loginfo("line2 上 land pos空了 后面的车 补上")
            car_id = car_id_list_2[1]  
            car_now_pos = self.car_info[car_id].pos
            car_goal_pos = self.land_p2
            route = [car_now_pos, car_goal_pos]
            cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}   

        car_id_list_3 = self.line_3_car_id_list
        # line3 上 land pos空了 后面的车 补上
        if car_id_list_3[0] in wait_pos_car_set and car_id_list_3[1] not in land_pos_car_set:
            # rospy.loginfo("line3 上 land pos空了 后面的车 补上")
            car_id = car_id_list_3[0] 
            car_now_pos = self.car_info[car_id].pos
            car_goal_pos = self.land_p3
            route = [car_now_pos, car_goal_pos]
            cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}    
        elif car_id_list_3[1] in wait_pos_car_set and car_id_list_3[0] not in land_pos_car_set:
            # rospy.loginfo("line3 上 land pos空了 后面的车 补上")
            car_id = car_id_list_3[1]  
            car_now_pos = self.car_info[car_id].pos
            car_goal_pos = self.land_p3
            route = [car_now_pos, car_goal_pos]
            cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}   
                
        
        # 3-上货后 尽快前往 起飞点
        if len(self.car_waiting_go_aw) != 0:
            rospy.loginfo("航空区的尽快出发")
            for car in self.car_waiting_go_aw:
                car_id = car.sn
                car_now_pos = car.pos
                
                # 对于line1来说 需要判断一下 take off pos是否有飞机
                if car_id in self.line_1_car_id_set:
                    another_car_id = self.find_line_another_car_id(car_id)
                    mid_1 = Position(15  + 180, 5 + 420, -16)
                    # mid_2 = Position(19  + 180, 24.5 + 420, -16)
                    car_goal_pos  = self.take_off_p_1
                    route = [car_now_pos, mid_1, car_goal_pos]

                    if self.car_info[another_car_id].have_uav is True \
                    and another_car_id in work_wait_pos_car_set:
                        rospy.loginfo("另一车上有飞机 但是在work wait pos")
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 

                    elif self.car_info[another_car_id].have_uav is False:
                        rospy.loginfo("line1 上 take off pos空了 后面的车 去 起飞点")
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 

                    
                elif car_id in self.line_2_car_id_set:
                    bool_can_go = True
                    another_car_id = self.find_line_another_car_id(car_id)
                    # 判断一下 另一个车有飞机 且飞机有货 就不能出发
                    if self.car_info[another_car_id].have_uav is True:
                        uav_id = self.car_info[another_car_id].uav_sn
                        if self.uav_info_dict[uav_id].have_cargo is True:
                            bool_can_go = False
                    if bool_can_go is True:
                        rospy.loginfo("line 2 航空区的尽快出发")
                        mid_1 = Position(19  + 180, 5 + 420, -16)
                        mid_2 = Position(19  + 180, 15 + 420, -16)
                        car_goal_pos  = self.take_off_p_2
                        route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}  
              
                elif car_id in self.line_3_car_id_set:
                    rospy.loginfo("line 3 航空区的尽快出发")
                    mid_1 = Position(19  + 180, 5 + 420, -16)
                    mid_2 = Position(19  + 180, 19 + 420, -16)
                    car_goal_pos  = self.take_off_p_3
                    route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}   
                   
        
        # 4-接到飞机后 先去work wait pos 等待工作区空闲
        # 找到处于gw状态 且位于land pos上的车 
        # 这里还需要判断一下 work wait pos 是否有空
        if len(self.car_waiting_go_gw) != 0:
            # 每一个车 得到降落的飞机都都需要判断是否要去 等待进入工作区
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                car = self.car_info[car_id]
                # car_id = self.car_waiting_go_gw[0].sn
                car_now_pos = car.pos
                # rospy.loginfo("有飞机降落到车上 car id=" + str(car_id))
                
                # 如果车属于line1 车在land pos上
                if car_id in self.line_1_car_id_set and car_id in land_pos_car_set:
                    another_car_id = self.find_line_another_car_id(car_id)
                    another_car = self.car_info[another_car_id]
                    
                    # 如果另一车没有在work wait pos上 那就直接去work wait pos
                    # self.judge_car_is_going_from_land_pos_to_work_wait_pos(car_id) is False
                    if self.work_wait_p1_free is True:
                        rospy.loginfo("如果另一车没有在work wait pos上 那就直接去work wait pos")
                        car_goal_pos = self.work_wait_p1
                        route = [car_now_pos, car_goal_pos]
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
                    
                    # 如果另一车是pick up状态 且已经没有ready的无人机
                    # 此时就会卡主  所以应该去解决这个卡主的状态
                    # 判断的4个条件：
                    # 1-前面确实有车 2-这个车没有飞机 
                    # 3-另一车处于work wait pos上 4-工作区是free
                    # 5-已经没有车 ready 这条是否需要加入 有待考虑
                    elif self.work_wait_p1_free is False \
                    and another_car.have_uav is False \
                    and another_car_id in work_wait_pos_car_set \
                    and self.work_station_free is True:
                        rospy.loginfo("当前car直接去工作区--前面那个车回到land pos")
                        # 当前这个car 直接去工作区                        
                        car_goal_pos = self.work_station_pos
                        mid_1 = Position(10 + 180, 11 + 420, -16)
                        route = [car_now_pos, mid_1, car_goal_pos]
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
                        self.work_station_car_id = car_id
                        # 前面那个车 回到land pos
                        another_car_pos = another_car.pos
                        another_car_goal_pos = self.land_p1
                        mid_2 = Position(9 + 180, 7 + 420, -16)
                        mid_3 = Position(6 + 180, 7 + 420, -16)
                        ano_route = [another_car_pos, mid_2, mid_3, another_car_goal_pos]
                        cmd_res[another_car_id] = {'cmd':'MOVE_TO_TARGET', 'route':ano_route}  
                
                # 1-处于line2上, 2-车在land pos上, 3-没有车从land pos到work wait pos
                # self.judge_car_is_going_from_land_pos_to_work_wait_pos(car_id) is False
                elif car_id in self.line_2_car_id_set and car_id in land_pos_car_set \
                and self.work_wait_p2_free is True:
                    rospy.loginfo("line2 在land pos上 直接去work wait pos2")
                    # mid = Position(4.5 + 180, 17 + 420, -16)
                    car_goal_pos = self.work_wait_p2
                    route = [car_now_pos, car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                    self.work_wait_p2_car_id = car_id
                
                # 1-处于line3上, 2-车在land pos上, 3-没有车从land pos到work wait pos
                elif car_id in self.line_3_car_id_set and car_id in land_pos_car_set \
                and self.work_wait_p3_free is True:
                    rospy.loginfo("line3 在land pos上 直接去work wait pos3")
                    # mid_1 = Position(1 + 180, 25 + 420, -16)
                    car_goal_pos = self.work_wait_p3
                    route = [car_now_pos, car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                    self.work_wait_p3_car_id = car_id
            
                # 处于等待进入区的车, 23-line的车 判断是否需要进入临界区
                # 如果车属于line23, 车在work wait pos上, 没有车正在去neighbor pos
                if self.judge_line23car_on_work_wait_pos(work_wait_pos_car_set) is True \
                and self.nei_pos_free is True:
                    rospy.loginfo("处于等待进入区的车, 2 3-line的车 判断是否需要进入临界区")
                    car_id = self.choose_urgent_car_to_go_nei(work_wait_pos_car_set)
                    route = self.choose_route_for_car_go_neighbor_pos(car_id)
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                    self.nei_pos_car_id = car_id
                    rospy.loginfo("car_id = " + str(car_id))
                    rospy.loginfo("route=" +  str(route))

        # 5- 处于land pos上 且处于pick up状态的飞机 
        if len(self.car_waiting_pickup) != 0:
            rospy.loginfo("处于pick up状态的飞机")
            car_id = self.find_pickup_car_on_land_pos(land_pos_car_set)
            
            if car_id is not None:
                car = self.car_info[car_id]
                car_now_pos = car.pos 
            
            # rospy.loginfo("car id = " + str(car_id))
            # 这里的route设定和上面的一样 其实可以抽象成单独的函数 后续可以做
            if car_id in self.line_1_car_id_set and car_id in land_pos_car_set:
                # mid = Position(6 + 180, 11 + 420, -16)
                # rospy.loginfo("line1")
                another_car_id  = self.find_line_another_car_id(car_id)
                
                # 如果后面已经有车等待 且暂时没有飞机要降落 且仍有空余的飞机
                # 就去work wait pos
                # another_car_id in wait_pos_car_set 
                # 这里判断另一个车是否正在移动 但是还没到work wait pos
                another_x = self.car_info[another_car_id].pos.x
                another_y = self.car_info[another_car_id].pos.y
                gap_n = 0.5
                if self.para.land_p1.x - gap_n <= another_x <= self.para.work_wait_p1.x + gap_n \
                    and self.para.work_wait_p1.y - gap_n <= another_y <= self.para.land_p1.y + gap_n:
                    rospy.loginfo("line1 另一车正在work wait pos")
                    another_is_going_to_work_wait_pos = True
                else:
                    rospy.loginfo("line1 另一车没有在work wait pos")
                    another_is_going_to_work_wait_pos = False
                    
                if self.car_have_landing_uav(car_id) is False \
                and len(self.uav_ready) != 0 \
                and another_is_going_to_work_wait_pos is False: 
                    rospy.loginfo("有飞机ready 前面没有车等待 没有飞机马上降落，去等待进入区")
                    car_goal_pos = self.work_wait_p1
                    # mid_1 = Position(8 + 180, 11 + 420, -16)
                    route = [car_now_pos,  car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                # self.work_station_car_id = car_id
            
            # 这里认为line2上车不要去上新的飞机  等待飞机降落就好了 这里line2 可以去接 保留修改可能
            # elif car_id in self.line_2_car_id_set and car_id in land_pos_car_set:
            #     another_car_id  = self.find_line_another_car_id(car_id)

            #     # rospy.loginfo("line2")
            #     if another_car_id in wait_pos_car_set \
            #         and self.car_have_landing_uav(car_id) is False \
            #         and len(self.uav_ready) != 0:
            #         # rospy.loginfo("go 2")
            #         mid = Position(4.5 + 180, 17 + 420, -16)
            #         car_goal_pos = self.work_wait_p2
            #         route = [car_now_pos, mid, car_goal_pos]
            #         cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                # self.work_station_car_id = car_id
            
            # 这里认为line3上车不要去上新的飞机  等待飞机降落就好了
            # elif car_id in self.line_3_car_id_set and car_id in land_pos_car_set:
            #     another_car_id  = self.find_line_another_car_id(car_id)
            #     # rospy.loginfo("line3")
            #     if another_car_id in wait_pos_car_set \
            #         and self.car_have_landing_uav(car_id) is False:  
            #         # rospy.loginfo("go 3")               
            #         mid_1 = Position(1 + 180, 25 + 420, -16)
            #         car_goal_pos = self.work_wait_p3
            #         route = [car_now_pos, mid_1, car_goal_pos]
            #         cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                # self.work_station_car_id = car_id                            

        # 如果nei pos一直有 但是车一直没动 就需要一直发move cmd
        if self.nei_pos_car_id != None:
            car_id = self.nei_pos_car_id
            car = self.car_info[car_id]
            car_pos = car.pos
            if car_pos.y >= self.para.work_wait_p2.y - 1.0:
                rospy.loginfo("send cmd to work wait stuck car id=" + str(car_id)) 
                route = self.choose_route_for_car_go_neighbor_pos(car_id)
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}

        if self.work_station_car_id != None:
            car_id = self.work_station_car_id
            car = self.car_info[car_id]
            car_pos = car.pos
            if car_pos.x <= self.para.work_station_pos.x - 3:
                if car_id in self.line_1_car_id_set:
                    rospy.loginfo("send cmd to nei stuck car id=" + str(car_id)) 
                    route = [car_pos, self.para.work_station_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                else:
                    mid = Position(10 + 180, 2 + 420, -16)
                    rospy.loginfo("send cmd to nei stuck car id=" + str(car_id)) 
                    route = [car_pos, mid, self.para.work_station_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}                    

        # 工作区没有被占据
        self.work_station_free = self.Judge_work_station_is_free()
        if self.work_station_free is True:
            rospy.loginfo("work_station_free is True")
            # 找到所有car_waiting_go_gw 让它们先去工作区 此时这些车上是有飞机的
            # rospy.loginfo("car_waiting_go_gw")
            # rospy.loginfo(self.car_waiting_go_gw)
            
            # 处于gw的车 在line1 处于work wait pos 和 line 23 处于nei pos上的车
            # 选择让去工作区
            if self.judge_gw_car_on_work_wait_pos_or_nei_pos(work_wait_pos_car_set, nei_car_set) is True:
                rospy.loginfo("找到处于gw的车 且位于nei pos上的车 让去工作区")
                car_id = self.choose_urgent_car_to_go_work_station(work_wait_pos_car_set, nei_car_set)
                route = self.choose_route_for_car_go_workstation(car_id)
                rospy.loginfo("the go work station car_id = " + str(car_id))
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                self.work_station_car_id = car_id            

            # 如果在等待进入工作区的飞机没有gw 但是有pickup的 且还有飞机ready 让去上飞机
            elif self.judge_gw_car_on_work_wait_pos_or_nei_pos(work_wait_pos_car_set, nei_car_set) is False \
            and self.judge_pickup_car_on_work_wait_pos(work_wait_pos_car_set) is True \
            and len(self.uav_ready) != 0:
                rospy.loginfo("如果在等待进入工作区的飞机没有gw 但是有pickup的 且还有飞机ready 让去上飞机")
                for car_id in work_wait_pos_car_set:
                    # rospy.loginfo(car_id)
                    # rospy.loginfo("judge_have_wait_car")
                    # rospy.loginfo(self.judge_have_wait_car(car_id, wait_pos_car_set))
                    # rospy.loginfo("car_have_landing_uav")
                    # rospy.loginfo(self.car_have_landing_uav(car_id))
                    # 这里只能让line1的车去
                    if car_id in self.line_1_car_id_set:
                        route = self.choose_route_for_car_go_workstation(car_id)
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}    
                        self.work_station_car_id = car_id
                        break   

        else: # 工作区被占据
            rospy.loginfo("work_station is occupied")
            # 判断是发送命令 是上货 还是还飞机
            if self.judge_car_is_on_work_station() is True:
                rospy.loginfo("car is on work station")
                car_id = self.work_station_car_id
                
                bool_need_retrieve = self.judge_need_retrieve(self.work_station_car_id)
                bool_ground = self.judge_unloading_station_id_conflict(car_id)
                
                if self.car_info[car_id].have_uav is True:
                    uav_id = self.car_info[car_id].uav_sn
                    if self.uav_info_dict[uav_id].have_cargo is False:
                        bool_have_cargo = False
                    else:
                        bool_have_cargo = True
                else:
                    bool_have_cargo = False
                # 如果时间冲突 地面即将起飞两个有一个冲突
                if bool_need_retrieve is True and self.car_info[car_id].have_uav is True \
                and bool_have_cargo is False:
                    uav_id = self.car_info[car_id].uav_sn
                    if self.uav_info_dict[uav_id].have_cargo == False:
                        rospy.loginfo("空中有冲突 回收飞机 " + str(car_id))
                        rospy.logwarn("空中有冲突 回收飞机 " + str(car_id))
                        cmd_res[car_id] = {'cmd':'UAV_RETRIEVE'}
                        self.leave_work_station_car_id = car_id                    
                elif bool_ground is True \
                and self.car_info[car_id].have_uav is True \
                and bool_have_cargo is False:
                    uav_id = self.car_info[car_id].uav_sn
                    if self.uav_info_dict[uav_id].have_cargo == False:
                        rospy.loginfo("地面有冲突 回收飞机 " + str(car_id))
                        rospy.logwarn("地面有冲突 回收飞机 " + str(car_id))
                        cmd_res[car_id] = {'cmd':'UAV_RETRIEVE'}
                        self.leave_work_station_car_id = car_id                       
                elif bool_need_retrieve is False and bool_ground is False \
                and self.car_info[car_id].have_uav is False:
                    # 只允许line1上飞机 z
                    if car_id in self.line_1_car_id_set and car_id != self.leave_work_station_car_id:
                        # rospy.logwarn("line1 上飞机")
                        rospy.loginfo("line1 上飞机")
                        cmd_res[car_id] = {'cmd':'RECEIVE_UAV'}
                elif bool_need_retrieve is True and self.car_info[car_id].have_uav is False \
                and bool_have_cargo is False:
                    # 车来接飞机 但是突然发现 来不及上 立即离开
                    if car_id in self.line_1_car_id_set:
                        rospy.loginfo("车来接飞机 但是突然发现 来不及上 立即离开")
                        self.leave_work_station_car_id = car_id 

            rospy.loginfo("len(self.car_waiting_uav_work) = " + str(len(self.car_waiting_uav_work)))
            if len(self.car_waiting_uav_work) != 0:
                rospy.loginfo("car_id=" + str(car_id))
                car_id = self.car_waiting_uav_work[0].sn
                rospy.loginfo("self.car_info[car_id].have_uav=" + str(self.car_info[car_id].have_uav))
                if self.car_info[car_id].have_uav is True and bool_need_retrieve is False:
                    uav_id = self.car_info[car_id].uav_sn
                    rospy.loginfo("self.uav_info_dict[uav_id].have_cargo=" + str(self.uav_info_dict[uav_id].have_cargo))
                    if self.uav_info_dict[uav_id].have_cargo == False:
                        # rospy.loginfo("")
                        # rospy.loginfo("remaining_battery  = " + str(self.uav_info_dict[uav_id].remaining_battery))
                        # rospy.loginfo(self.uav_info_dict[uav_id].remaining_battery <= 90)
                        if self.uav_info_dict[uav_id].remaining_battery <= 50:
                            if self.uav_info_dict[uav_id].remaining_battery <= 30:
                                rospy.loginfo("battery is low")
                                rospy.logwarn("充电 " + str(car_id))
                                cmd_res[car_id] = {'cmd':'UAV_CHARGE'}
                            elif self.judge_battery_num() is True:
                                rospy.loginfo("Too many battery low")
                                rospy.logwarn("充电 " + str(car_id))
                                cmd_res[car_id] = {'cmd':'UAV_CHARGE'}
                            else:
                                rospy.loginfo("load cargo")
                                # rospy.logwarn("上货")
                                cmd_res[car_id] = {'cmd':'LOAD_CARGO'}
                        else:
                            rospy.loginfo("load cargo")
                            # rospy.logwarn("上货")
                            cmd_res[car_id] = {'cmd':'LOAD_CARGO'}
                    
                # elif car_id in self.car_for_land_set:
                #     car_id = self.car_waiting_uav_work[0].sn
                #     if self.car_info[car_id].have_uav is True:
                #         rospy.loginfo("retrieve uav")
                #         # 如果车属于降落 还飞机
                #         cmd_res[car_id] = {'cmd':'UAV_RETRIEVE'}

        # rospy.loginfo("work is free = " + str(self.work_station_free))
        # if cmd_res != {}:
        #     rospy.logerr(f"car cmd res: {cmd_res}")
            
        rospy.loginfo(f"car cmd res: {cmd_res}")
            # rospy.loginfo(cmd_res)
        return cmd_res   
    
    def judge_car_is_on_work_station(self):
        if self.work_station_car_id == None:
            for car in self.car_info.values():
                dis_n = self.dis_cal(car.pos, self.work_station_pos)
                if dis_n < 0.6:
                    self.work_station_car_id = car.sn
                    return True
            return False
        
        car = self.car_info[self.work_station_car_id]
        dis_n = self.dis_cal(car.pos, self.work_station_pos)
        if dis_n < 0.6:
            return True
        else:
            return False

    def update_car_uav_cargo_info(self):
        # 找到那些还没有起飞 但是已经在移动的车
        # 一个即将出发的卸货站id set
        rospy.loginfo("uav_id_to_unloading_station_id")
        rospy.loginfo(self.uav_id_to_unloading_station_id)
        self.soon_to_go_unload_station_id_set = set()
        for car in self.car_waiting_go_aw:
            if car.have_uav is True:
                uav_id = car.uav_sn
                if uav_id not in self.uav_id_to_unloading_station_id:
                    continue
                unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
                self.soon_to_go_unload_station_id_set.add(unloading_station_id)
    
    def judge_unloading_station_id_conflict(self, car_id):
        unloading_station_id = self.unload_station_id
        if unloading_station_id in self.soon_to_go_unload_station_id_set \
        and car_id not in self.uav_id_to_unloading_station_id:
            # rospy.logwarn("即将上货的飞机 和马上起飞的 卸货点冲突！")
            rospy.loginfo("即将上货的飞机 和马上起飞的 卸货点冲突！")
            return True
        else:
            return False
    def cal_flight_route_time(self, line_id, unloading_station_id):
        # 根据line和要去的station 计算出一个飞行的时间
        time = self.uav_go_flight_time[line_id][unloading_station_id]
        # rospy.logwarn(f"line:{line_id} unloading_station_id:{unloading_station_id} flight time:{time}")
        rospy.loginfo(f"line:{line_id} unloading_station_id:{unloading_station_id} flight time:{time}")
        return time
    def determine_moving_to_landpos_time(self, line_id):
        if line_id == 1:
            moving_to_landpos_time = 15
        elif line_id == 2:
            moving_to_landpos_time = 17 
        elif line_id == 3:
            moving_to_landpos_time = 19       
        return moving_to_landpos_time * self.time_scale
    def determine_moving_to_takeoffpos_time(self, line_id):
        if line_id == 1:
            moving_to_take_off_pos_time = 17
        elif line_id == 2:
            moving_to_take_off_pos_time = 30
        elif line_id == 3:
            moving_to_take_off_pos_time = 40
        
        return moving_to_take_off_pos_time * self.time_scale
    
    def other_uav_fly_time(self, unloading_station_id):
        uav_time = self.unloading_cargo_pressure[unloading_station_id]['uav_time']
        time_list = []
        for key, item in uav_time.items():
            time_list.append(item)

        return max(time_list)

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

    def judge_landpos_have_car(self, line_id):
        # 判断这个line上 landpos是否有车
        car_id_a, car_id_b =  self.get_two_car_id(line_id)
        if car_id_a in self.land_pos_car_set or car_id_b in self.land_pos_car_set:
            return True
        else:
            return False

    def judge_to_many_gw_car(self):
        busy_list = []
        posy_l = 17 + 420
        for car in self.car_running:
            if car.have_uav is True:
                uav_id = car.uav_sn
                if self.uav_info_dict[uav_id].have_cargo is False \
                and car.pos.y < posy_l:
                    busy_list.append(car.sn)
        
        for car in self.car_waiting_go_gw:
            if car.pos.y < posy_l:
                busy_list.append(car.sn)

        if len(self.car_waiting_go_gw) >= 3:
            rospy.loginfo("gw车数量大于3个 需要回收")
            rospy.logwarn("gw车数量大于3个 需要回收")
            return True
        elif len(busy_list) >= 3:
            rospy.loginfo("gw车数量等于3个 密集 需要回收")
            rospy.logwarn("gw车数量等于3个 密集 需要回收")
            return True
        
        return False

    def judge_need_retrieve(self, car_id):
        # 判断当前车上的飞机 是否需要回收
        # 当前车上货的订单号
        # 这个卸货点 unloading cargo pressure = 1或者2
        # 找到那个最大的时间
        unloading_station_id = self.unload_station_id
        if self.unload_station_id == None:
            return False
        car = self.car_info[car_id]
        # if self.car_info[car_id].have_uav is True:
        uav_id = self.car_info[car_id].uav_sn
        line_id = self.judge_car_on_line_number(car_id)
        go_num = self.unloading_cargo_pressure[unloading_station_id]['num']
        back_num = self.car_land_pressure[line_id]['num']

        rospy.loginfo("ret-judge: car_id=" + str(car_id)+", uav_id=" + str(uav_id))
        # rospy.logwarn("ret-judge: car_id=" + str(car_id)+", uav_id=" + str(uav_id))
        # if car_id in self.line_2_car_id_set or car_id in self.line_3_car_id_set:
        #     return True

        if self.judge_to_many_gw_car() is True:
            return True

        # 去的航线上没有飞机 不需要回收，回来的航线没有飞机 不着急回收
        if back_num == 0:
            # rospy.logwarn("回来的航线没有飞机 不着急")
            rospy.loginfo("回来的航线没有飞机 不着急")
            return False

        loading_cargo_time = 15 * self.time_scale
        go_time_gap = self.para.go_time_gap_all
        flight_time = self.cal_flight_route_time(line_id, unloading_station_id)  
        moving_takeoff_time = self.determine_moving_to_takeoffpos_time(line_id)

        if go_num != 0:
            other_uav_time = self.other_uav_fly_time(unloading_station_id)  
            # （航线预计时间 -（别的飞机时间 - 上货时间 - 运动到目标点时间））< 起飞间隔时间
            # 等待时间 = 起飞间隔时间 - （航线预计时间 -（别的飞机时间 - 上货时间 - 运动到目标点时间））
            gap_time = flight_time - (other_uav_time - loading_cargo_time - moving_takeoff_time)
            wait_time = go_time_gap - gap_time
            rospy.loginfo("other_uav_time: %f", other_uav_time)
            # rospy.logwarn("other_uav_time: %f", other_uav_time)
            rospy.loginfo("flight_time : %f", flight_time)
            # rospy.logwarn("flight_time : %f", flight_time)
            rospy.loginfo("wait_time: %f", wait_time)
            # rospy.logwarn("wait_time: %f", wait_time)
            rospy.loginfo("gap_time: %f", gap_time)
            # rospy.logwarn("gap_time: %f", gap_time)
        else:
            wait_time = 0
        
        if wait_time > 0:
            bool_need_wait = True
            rospy.loginfo("it need wait! line"+ str(line_id))
            # rospy.logwarn("it need wait! line"+ str(line_id))
        else:
            bool_need_wait = False
            rospy.loginfo("no need wait! line" + str(line_id))
            # rospy.logwarn("no need wait! line" + str(line_id))

        if wait_time < 0:
            wait_time = 0

        # (等待时间 + 运动到land pos的时间 + 上货时间 + 从上货点到takeoff_pos时间) < 飞机剩余降落时间

        # 1-当前这个车上飞机后 到达起飞点后 是否能够立即起飞
        # 如果不能够立即起飞 需要等待的时间大于某个值

        # 2-这个马上要降落的飞机时间很紧急
        land_pos_have_uav = self.judge_landpos_have_car(line_id)
        moving_landpos_time = self.determine_moving_to_landpos_time(line_id)
        land_num = self.car_land_pressure[line_id]['num']
        uav_time = self.car_land_pressure[line_id]['uav_time']
        uav_list = []
        for key, item in uav_time.items():
            uav_list.append(item)
        rospy.loginfo("uav_time: " + str(uav_time))
        if len(uav_list) == 0:
            rospy.loginfo("没有飞机降落 直接上货!")
            return False
        # rospy.logwarn("land num=" + str(land_num))
        rospy.loginfo("降落 uav_list: %s", uav_list)
        # rospy.logwarn("降落 uav_list: %s", uav_list)
        max_time_drone_id = max(uav_time, key=lambda k: uav_time[k])
        uav_land_remain_time = max(uav_list)

        # 在这个line上car land pressure = 1 并且land pos上没有飞机
        # 还需要判断是否有飞机已经在前面了
        if len(uav_list) == 1:
            rospy.loginfo("land num=1")
            # rospy.logwarn("land num=1")
            if self.judge_line_need_car(car_id) is True:
                rospy.loginfo("line上car land num = 1 并且land pos上没有飞机")
                # rospy.logwarn("line上car land num = 1 并且land pos上没有飞机")
                time_last = (wait_time + moving_landpos_time + loading_cargo_time + moving_takeoff_time)
                rospy.loginfo("time_last = %f", time_last)
                rospy.loginfo("uav_land_remain_time = %f", uav_land_remain_time)
                # rospy.logwarn("time_last = %f", time_last)
                # rospy.logwarn("uav_land_remain_time = %f", uav_land_remain_time)
                rospy.loginfo("间隔时间: %f", (uav_land_remain_time - time_last))
                # rospy.logwarn("间隔时间: %f", (uav_land_remain_time - time_last))
                if time_last < uav_land_remain_time:
                    rospy.loginfo("line=" + str(line_id) + ", car_id="+ str(car_id) \
                        + " uav_id=" + str(max_time_drone_id) + " 来得及上货")
                    # rospy.logwarn("line=" + str(line_id) + ", car_id="+ str(car_id) \
                        # + " uav_id=" + str(max_time_drone_id) + " 来得及上货")                    
                    return False
                else:
                    rospy.loginfo("line=" + str(line_id) + ", car_id="+ str(car_id) \
                        + " uav_id=" + str(max_time_drone_id) + " 来不级 回收!!")
                    # rospy.logwarn("line=" + str(line_id) + ", car_id="+ str(car_id) \
                        # + " uav_id=" + str(max_time_drone_id) + " 来不及 回收!!")   
                    return True

        elif len(uav_list) == 2:
        # car land pressure = 2 land pos上有飞机 看最长的时间的飞机 
            if self.judge_line_need_car(car_id) is True:
                uav_land_remain_time = min(uav_list)
            else:
                uav_land_remain_time = max(uav_list)

            rospy.loginfo("car land num = 2 land pos上有飞机 看最长的时间的飞机")
            # rospy.logwarn("car land num = 2 land pos上有飞机 看最长的时间的飞机")
            time_last = (wait_time + moving_landpos_time + loading_cargo_time + moving_takeoff_time)
            rospy.loginfo("time_last = %f", time_last)
            rospy.loginfo("uav_land_remain_time = %f", uav_land_remain_time)
            # rospy.logwarn("time_last = %f", time_last)
            # rospy.logwarn("uav_land_remain_time = %f", uav_land_remain_time)
            rospy.loginfo("间隔时间: %f", (uav_land_remain_time - time_last))
            # rospy.logwarn("间隔时间: %f", (uav_land_remain_time - time_last))
            if time_last < uav_land_remain_time:
                rospy.loginfo("line=" + str(line_id) + ", car_id="+ str(car_id) \
                    + " uav_id=" + str(max_time_drone_id) + " 来得及上货")
                # rospy.logwarn("line=" + str(line_id) + ", car_id="+ str(car_id) \
                    # + " uav_id=" + str(max_time_drone_id) + " 来得及上货")     
                return False
            else:
                rospy.loginfo("line=" + str(line_id) + ", car_id="+ str(car_id) \
                    + " uav_id=" + str(max_time_drone_id) + " 来不级 回收!!")
                # rospy.logwarn("line=" + str(line_id) + ", car_id="+ str(car_id) \
                    # + " uav_id=" + str(max_time_drone_id) + " 来不及 回收!!")  
                return True
            
        # rospy.logwarn("情况不着急!")
        rospy.loginfo("情况不着急!")
        return False

    def judge_line_need_car(self, car_id):
        # 判断当前line的另一辆车 是否马上要去land pos
        line_id = self.judge_car_on_line_number(car_id)
        another_car_id = self.find_line_another_car_id(car_id)
        another_car = self.car_info[another_car_id]

        # 如果另一个车处于gw的状态 或者wait uav work 说明需要车马上过来
        if another_car_id in self.car_waiting_go_gw_set: 
            rospy.loginfo("another car is going to gw")
            # rospy.logwarn("another car is going to gw")
            return True
        elif another_car_id in self.car_waiting_uav_work_set:
            rospy.loginfo("another car is waiting_uav_work")
            # rospy.logwarn("another car is waiting_uav_work")
            return True
        elif another_car.have_uav is True and another_car_id in self.car_running_set:
            uav_id = another_car.uav_sn
            if self.uav_info_dict[uav_id].have_cargo == False:
                rospy.loginfo("another car is running")
                # rospy.logwarn("another car is running")
                return True
        else:
            if car_id in self.line_1_car_id_set:
                
                if another_car.pos.y < (self.para.land_p1.y - 1):
                    rospy.loginfo("another_car.pos" + str(another_car.pos))
                    rospy.loginfo("line1 need!")
                    # rospy.logwarn("line1 need!")
                    return True
                
            rospy.loginfo("dont need another car!")
            # rospy.logwarn("dont need another car!")
            return False
    def judge_battery_num(self):
        num = 0
        for key, item in self.uav_info_dict.items():
            if item.remaining_battery < 50:
                num += 1
        if num >= 4:
            return True
        else:
            return False
        
    def dis_cal(self, des_pos, cur_pos):
        des = np.array([des_pos.x, des_pos.y, des_pos.z])
        cur = np.array([cur_pos.x, cur_pos.y, cur_pos.z])
        return np.linalg.norm(np.array(des - cur))

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