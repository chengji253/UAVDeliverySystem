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
    
    def __init__(self):
        
        self.para = Para()
        
        # Location of UAVs in airport and unloading location
        self.drone_station_origin = self.para.drone_station_origin
        # self.work_station_pos = [190, 425, -16]
        self.work_station_pos = self.para.work_station_pos
        self.drone_store_pos = self.para.drone_store_pos
        
        
        # Information about all flight paths. 
        # Look up the length of the path according to the path id, whether it's the way to or from, 
        # which planes are in which locations.
        self.flight_path_info = {}
        
        # Find the path ids to and from the unloading point id.
        self.release_cargo_idx_to_path = {}
        
        # Landing point info and which landing points are available
        self.land_point = {}
        
        # Takeoff point info and which takeoff points are available
        self.take_off_point = {}
        
        # Location of unloading points
        self.release_cargo_pos = {}
        
        # Is there a AGV in the workspace now? If there is a AGV, it's True. If there isn't, it's False.
        self.work_station_free = True
        # The AGV id currently occupying the workspace
        self.work_station_car_id = None
        
        # Is the critical nei free
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
        
        # Initialization Air management class information
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
        # Update the current AGV information based on the total node information to complete the scheduling.
        self.car_info = car_info_dict
        self.car_waiting_pickup = car_waiting_pickup
        self.car_running = car_running                              
        self.car_waiting_go_aw = car_waiting_go_aw
        self.car_waiting_go_gw = car_waiting_go_gw
        self.car_waiting_uav_work = car_waiting_uav_work

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
            # If a AGV is specified to come in 
            # rospy.loginfo('AGV is cmd to come! nei pos is occupied')
            car_id = self.nei_pos_car_id
            car_pos = self.car_info[car_id].pos
            # The AGV that's coming in has an UAV, and it's loaded, and it's far away, 
            # so that means it's ready for the next AGV.
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
            # If a AGV is specified to come in 
            # rospy.loginfo('AGV is cmd to come! work station is occupied')
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
        # Find the AGV that's in the land pos wait pos and in the car_waiting_pickup state.
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
        # If the pos is assigned to come by the AGV, then it's false.
        # If the AGV is leaving and the AGV id is assigned to none, then it's dis.
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

        # It's the same for takeoff points
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
        # Select the most urgent AGV in line 2 3 to go to nei pos
        # Select the most urgent vehicle on the UAV to go
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


        # Find the smallest of all uav_times
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

        # Find the key corresponding to the minimum uav_time
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
        # Select the most urgent AGV to enter the workbench
        # For line 1, go to work wait pos.
        # For line 23, it's on the nei pos.
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

        # Find the smallest of all uav_times
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
        # Find the key corresponding to the minimum uav_time
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
        # Chosen to get the most urgent AGV on the UAV.
        if len(self.car_waiting_go_gw) == 1:
            return  self.car_waiting_go_gw[0].sn      
        else:
            car_land_pressure = {}
            for car in self.car_waiting_go_gw:
                
                # Some of the AGV are receiving airplanes, but they're not in work wait pos.
                car_id = car.sn
                if car_id not in work_wait_pos_car_set:
                    continue
                if car_id in self.line_1_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[1]
                elif car_id in self.line_2_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[2]
                elif car_id in self.line_3_car_id_set:
                    car_land_pressure[car_id] = self.car_land_pressure[3]
            
            
            # Find the minimum of all uav_times
            all_uav_times = [time for uav_times in car_land_pressure.values() \
                            for time in uav_times['uav_time'].values()]
            
            if len(all_uav_times) == 0:
                min_key = self.car_waiting_go_gw[0].sn 
                return min_key
            
            min_uav_time = min(all_uav_times)

            # Find the key corresponding to the minimum uav_time
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
        # Returns another id for the given AGV id in line
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
        # Determine if there's a AGV in gw state that's on the wait work pos #
        if len(self.car_waiting_go_gw) == 0:
            return False
        else:
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                if car_id in work_wait_pos_car_set:
                    return True
    def judge_gw_car_on_work_wait_pos_or_nei_pos(self, work_wait_pos_car_set, nei_pos_car_set):
        # Determine if there is a AGV in gw state line1 and is on wait work pos line23 is on nei pos
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
        # Determine if there's a AGV that's in a pick up state and is on a wait work pos.
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
        # Determine if any AGV on line23 are on the line from work wait pos to neighbor pos.
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
        # Is there a AGV on the line from land pos to work wait pos? 
        # True if there is, False if there isn't.
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
            # For line 1, you need to determine if there's an UAV in the take off pos.
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
                bool_can_go = True
                another_car_id = self.find_line_another_car_id(car_id)
                # Judging by the fact that there's a UAV in the other AGV and it's loaded, we can't go.
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
                rospy.loginfo("line 3 的去takeoff pos")
                mid_1 = Position(19  + 180, 5 + 420, -16)
                mid_2 = Position(19  + 180, 19 + 420, -16)
                car_goal_pos  = self.take_off_p_3
                route = [car_now_pos, mid_1, mid_2, car_goal_pos]
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 


        # 1 - The AGV returns to wait pos after the UAV takes off
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
        # On line 1, the land pos is empty, and the AGV behind it fills in. #
        # For line 1, both land pos and work wait pos are empty before the AGV behind can move up. #
        # If AGV 0 is in the wait pos and AGV 1 is neither in the land pos nor the work wait pos #
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
        # Line 2 land pos is empty. The AGV behind it, fill it in.
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
        # Line 3, land pos is empty. The AGV behind it, fill it in.
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
                
        
        # 3- Once loaded, proceed to the takeoff point as soon as possible.
        if len(self.car_waiting_go_aw) != 0:
            rospy.loginfo("航空区的尽快出发")
            for car in self.car_waiting_go_aw:
                car_id = car.sn
                car_now_pos = car.pos
                
                # For line 1, you need to determine if there's an UAV in the take off pos.
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
                    # Judging by the fact that there's a UAV in the other AGV and it's loaded, we can't go.
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
                   
        
        # 4 - When you get the UAV, go to work wait pos and wait for the workspace to become free.
        # Find the AGV that's in the gw state and in the land pos. # 
        # Here we need to determine if the work wait pos is free.
        if len(self.car_waiting_go_gw) != 0:
            # Every vehicle that gets to land an UAV needs to determine if it's going to wait to enter the workspace.
            for car in self.car_waiting_go_gw:
                car_id = car.sn
                car = self.car_info[car_id]
                # car_id = self.car_waiting_go_gw[0].sn
                car_now_pos = car.pos
                # rospy.loginfo("有飞机降落到车上 car id=" + str(car_id))
                
                # If the AGV belongs to line 1, the AGV is on land pos.
                if car_id in self.line_1_car_id_set and car_id in land_pos_car_set:
                    another_car_id = self.find_line_another_car_id(car_id)
                    another_car = self.car_info[another_car_id]
                    
                    # If the other AGV isn't on the work wait pos, then it goes straight to the work wait pos
                    # self.judge_car_is_going_from_land_pos_to_work_wait_pos(car_id) is False
                    if self.work_wait_p1_free is True:
                        rospy.loginfo("如果另一车没有在work wait pos上 那就直接去work wait pos")
                        car_goal_pos = self.work_wait_p1
                        route = [car_now_pos, car_goal_pos]
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
                    
                    # If the other vehicle is picked up and there are no more ready drones.
                    # It's going to get stuck, so it's time to fix the stuck state. 
                    # There are 4 conditions for judgment:
                    # 1 - there is a AGV in front of you
                    # 2 - this AGV doesn't have an UAV.
                    # 3- Another vehicle is in a work wait pos
                    # 4- The workspace is free # 5- No more vehicles ready
                    # 5-There are no more AGV ready, so I'm not sure if I need to add this one.
                    elif self.work_wait_p1_free is False \
                    and another_car.have_uav is False \
                    and another_car_id in work_wait_pos_car_set \
                    and self.work_station_free is True:
                        rospy.loginfo("当前car直接去工作区--前面那个车回到land pos")
                        # The current AGV goes directly to the workspace.                        
                        car_goal_pos = self.work_station_pos
                        mid_1 = Position(10 + 180, 11 + 420, -16)
                        route = [car_now_pos, mid_1, car_goal_pos]
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route} 
                        self.work_station_car_id = car_id
                        # The AGV in front of you, go back to land pos.
                        another_car_pos = another_car.pos
                        another_car_goal_pos = self.land_p1
                        mid_2 = Position(9 + 180, 7 + 420, -16)
                        mid_3 = Position(6 + 180, 7 + 420, -16)
                        ano_route = [another_car_pos, mid_2, mid_3, another_car_goal_pos]
                        cmd_res[another_car_id] = {'cmd':'MOVE_TO_TARGET', 'route':ano_route}  
                
                # 1- on line2, 2- AGV on land pos, 3- no AGV from land pos to work wait pos
                # self.judge_car_is_going_from_land_pos_to_work_wait_pos(car_id) is False
                elif car_id in self.line_2_car_id_set and car_id in land_pos_car_set \
                and self.work_wait_p2_free is True:
                    rospy.loginfo("line2 在land pos上 直接去work wait pos2")
                    # mid = Position(4.5 + 180, 17 + 420, -16)
                    car_goal_pos = self.work_wait_p2
                    route = [car_now_pos, car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                    self.work_wait_p2_car_id = car_id
                
                # 1- on line 3, 2- AGV on land pos, 3- no AGV from land pos to work wait pos
                elif car_id in self.line_3_car_id_set and car_id in land_pos_car_set \
                and self.work_wait_p3_free is True:
                    rospy.loginfo("line3 在land pos上 直接去work wait pos3")
                    # mid_1 = Position(1 + 180, 25 + 420, -16)
                    car_goal_pos = self.work_wait_p3
                    route = [car_now_pos, car_goal_pos]
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                    self.work_wait_p3_car_id = car_id
            
                # AGV that are in the waiting zone, AGV in line 23, determine if they need to enter the critical zone.
                # If the AGV belongs to line 23, the AGV is in the work wait pos, no AGV is going to the neighbor pos.
                if self.judge_line23car_on_work_wait_pos(work_wait_pos_car_set) is True \
                and self.nei_pos_free is True:
                    rospy.loginfo("处于等待进入区的车, 2 3-line的车 判断是否需要进入临界区")
                    car_id = self.choose_urgent_car_to_go_nei(work_wait_pos_car_set)
                    route = self.choose_route_for_car_go_neighbor_pos(car_id)
                    cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                    self.nei_pos_car_id = car_id
                    rospy.loginfo("car_id = " + str(car_id))
                    rospy.loginfo("route=" +  str(route))

        # 5- UAV in land pos and in pick up mode 
        if len(self.car_waiting_pickup) != 0:
            rospy.loginfo("处于pick up状态的飞机")
            car_id = self.find_pickup_car_on_land_pos(land_pos_car_set)
            
            if car_id is not None:
                car = self.car_info[car_id]
                car_now_pos = car.pos 
            
            # rospy.loginfo("AGV id = " + str(car_id))
            if car_id in self.line_1_car_id_set and car_id in land_pos_car_set:
                # mid = Position(6 + 180, 11 + 420, -16)
                # rospy.loginfo("line1")
                another_car_id  = self.find_line_another_car_id(car_id)

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

        # If the nei pos is always there, but the AGV never moves, you need to keep sending the move cmd.
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

        # Workspace is not occupied
        self.work_station_free = self.Judge_work_station_is_free()
        if self.work_station_free is True:
            rospy.loginfo("work_station_free is True")
            # Find all the car_waiting_go_gw's and get them to the work zone first. There are planes on these AGV.       
            # AGV in gw on line 1 in work wait pos and line 23 in nei pos.
            # Select to go to the workspace
            if self.judge_gw_car_on_work_wait_pos_or_nei_pos(work_wait_pos_car_set, nei_car_set) is True:
                rospy.loginfo("找到处于gw的车 且位于nei pos上的车 让去工作区")
                car_id = self.choose_urgent_car_to_go_work_station(work_wait_pos_car_set, nei_car_set)
                route = self.choose_route_for_car_go_workstation(car_id)
                rospy.loginfo("the go work station car_id = " + str(car_id))
                cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}
                self.work_station_car_id = car_id            

            # If an UAV waiting to enter the workspace doesn't have a gw, 
            # but has a pickup, and there's an UAV ready, let's get it on the UAV.
            elif self.judge_gw_car_on_work_wait_pos_or_nei_pos(work_wait_pos_car_set, nei_car_set) is False \
            and self.judge_pickup_car_on_work_wait_pos(work_wait_pos_car_set) is True \
            and len(self.uav_ready) != 0:
                rospy.loginfo("如果在等待进入工作区的飞机没有gw 但是有pickup的 且还有飞机ready 让去上飞机")
                for car_id in work_wait_pos_car_set:
                    # 这里只能让line1的车去
                    if car_id in self.line_1_car_id_set:
                        route = self.choose_route_for_car_go_workstation(car_id)
                        cmd_res[car_id] = {'cmd':'MOVE_TO_TARGET', 'route':route}    
                        self.work_station_car_id = car_id
                        break   

        else: # Workspace occupied
            rospy.loginfo("work_station is occupied")
            # Determine whether to send a command to load or return the UAV.
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
                # If there's a time conflict and the ground is about to take off, there's a conflict between the two.
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
                    # Only line 1 is allowed on the UAV.
                    if car_id in self.line_1_car_id_set and car_id != self.leave_work_station_car_id:
                        # rospy.logwarn("line1 上飞机")
                        rospy.loginfo("line1 上飞机")
                        cmd_res[car_id] = {'cmd':'RECEIVE_UAV'}
                elif bool_need_retrieve is True and self.car_info[car_id].have_uav is False \
                and bool_have_cargo is False:
                    # The AGV comes to pick up the UAV, but suddenly it was too late to get on and leave immediately.
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
        # Finding AGV that haven't taken off yet but are already on the move
        # An unloading station that's about to depart id set
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
        # Calculate a flight time based on the line and the station you're going to.
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
        # Determine if there is a AGV in the landpos on this line.
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
        # Determine if the UAV on the current truck needs to be recovered
        # The order number of the current load
        # This unloading point unloading cargo pressure = 1 or 2
        # Find the maximum time
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

        # No airplanes on the way there, no need to recycle, no airplanes on the way back, no rush to recycle.
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
            # (Estimated route time - (other UAV time - loading time - movement to target)) < takeoff interval
            # Waiting time = takeoff interval - (estimated route time - (other UAV time - loading time - movement to target))
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

        # (Waiting time + movement time to land pos + loading time + 
        # time from loading point to takeoff_pos) < Remaining landing time of the UAV

        # 1-Whether the UAV on board can take off immediately after arriving at the takeoff point.
        # 1- whether the UAV can take off immediately after arriving at the takeoff point 
        # 2- whether the UAV is going to land soon

        # 2-This UAV is about to land in an emergency.
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

        # On this line AGV land pressure = 1 and there are no planes on the land pos.
        # And need to determine if there's a UAV already in front of it
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
        # AGV land pressure = 2 land pos上有飞机 看最长的时间的飞机 
            if self.judge_line_need_car(car_id) is True:
                uav_land_remain_time = min(uav_list)
            else:
                uav_land_remain_time = max(uav_list)

            rospy.loginfo("car land num = 2 land pos上有飞机 看最长的时间的飞机")
            # rospy.logwarn("AGV land num = 2 land pos上有飞机 看最长的时间的飞机")
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
        # Determine if another AGV in the current line is going to land pos soon.
        line_id = self.judge_car_on_line_number(car_id)
        another_car_id = self.find_line_another_car_id(car_id)
        another_car = self.car_info[another_car_id]

        # If the other AGV is in gw, or wait uav work, that means it needs to come right away.
        if another_car_id in self.car_waiting_go_gw_set: 
            rospy.loginfo("another car is going to gw")
            # rospy.logwarn("another AGV is going to gw")
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