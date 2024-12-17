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
    
    def __init__(self):
        
        self.para = Para()
        
        # Location of UAVs in airport and unloading location
        self.drone_station_origin = self.para.drone_station_origin
        # self.work_station_pos = [190, 425, -16]
        self.work_station_pos = self.para.work_station_pos
        self.drone_store_pos = self.para.drone_store_pos
        
        self.safe_flight_min_height = -60
        self.safe_flight_max_height = -120
        
        # Info on all flight routes
        # According to the path ID, check whether the route is long or short, 
        # which planes are on the way back, and where they are located
        self.flight_path_info = {}
        
        # Find the destination and return route id based on the unloading point id
        self.unloadingStaion_id_to_path = {}
        
        # Landing point info and which landing points are available
        self.land_point = {}
        
        # Takeoff point info and which takeoff points are available
        self.take_off_point = {}
        
        # Location of unloading points
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
        
        # Route corresponding to the current UAV id
        self.uav_id_to_route_now = {}

    def init_info_one(self):
        # The time difference between going and coming back to the UAV
        # Must be greater than that to take off
        self.go_time_gap = []
        self.back_time_gap = []
        
        # UAV id to dict of relevant flight information
        self.uav_flight_dict = {}
        for i in range(30):
            dic = {'flying':False, 'on_go_path':False, 'on_back_path':False,
                   'remain_time': 0, 'unloading_cargo_id':None, 'land_pos_id':None}
            uav_id = self.uav_id[i]
            self.uav_flight_dict[uav_id] = dic

        
        # UAVs information at unloading points
        self.unloading_cargo_pressure ={0: {'num':0, 'uav_time': {}},
                                        1: {'num':0, 'uav_time': {}},
                                        2: {'num':0, 'uav_time': {}},
                                        3: {'num':0, 'uav_time': {}},
                                        4: {'num':0, 'uav_time': {}},
                                        5: {'num':0, 'uav_time': {}}}
        # uav_time = {id : remain_time}
        # UAVs information at landing points
        self.car_land_pressure = {1: {'num':0, 'uav_time': {}},
                                  2: {'num':0, 'uav_time': {}}, 
                                  3: {'num':0, 'uav_time': {}}}
        
        # The UAVs departure time, static
        # Behind the dictionary station id to time
        self.uav_go_flight_time = {1: {},
                                   2: {},
                                   3: {}}
        
        
        # When an UAV selects a landing point, it takes time for its state to change
        # The landing point selected by this UAV cannot be selected by other UAVs
        # When the UAV status changes to flying back, the dict is automatically deleted.
        # If the UAV state is still wait back, it's not deleted.
        self.cannot_choose_car_dict = {}
    
    def init_flight_path(self):
        # Initialize go back routes according to unloading points.
        
        # for cargo in unloading_cargo_stations:
        #     idx = cargo['index']
        #     x = cargo['position']['x']
        #     y = cargo['position']['y']
        #     z = cargo['position']['z']  
        #     self.release_cargo_pos[idx] = [x, y, z]            
        
        # Select a central location in the departure area 
        # Calculate the path from this location to the path of the cargo
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
        
        # loading points' go back mid pos
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
        # Update the current path UAV information based on the total node information
        # So that scheduling is complete
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
        # Update the current AGV information based on the total node information to complete the scheduling.
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
        # Find out which UAVs are coming back and which ones have come back to end-landing.
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
        
        # Returns false to indicate that there is no conflict
        rospy.loginfo("no conflict")
        return False
    
    def canont_choose_update(self):
        # Unselectable AGVs refreshes
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
        # Select the most urgent back of all self.uav_waiting_backs
        # Find the ones that have UAV coming and have the smallest arrival time
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
            
            # Select the landing location with the smallest num and no time conflict
            sorted_pos_id = sorted(self.car_land_pressure, key=lambda x: self.car_land_pressure[x]['num'])
            # Keep the landing time difference with all other UAVs above time_gap_all to prevent landing conflicts.
            time_list_all = []
            for key, value in self.car_land_pressure.items():
                uav_time = value['uav_time']
                for k1, v1 in uav_time.items():
                    time_list_all.append(v1)
            # rospy.loginfo("time_list_all")
            # rospy.loginfo(time_list_all)
            # No conflicts with UAV in current landing position
            cannot_choose_car_set = set()
            for key, value in self.cannot_choose_car_dict.items():
                cannot_choose_car_set.add(value)
            
            for pos_id in sorted_pos_id:
                # For each pos id, calculate the remain time
                route = self.set_route_for_uav_back(uav_id, pos_id)
                self.set_uav_to_route_dict(uav_id, route)
                remain_time = self.cal_remain_time_one(uav_pos, uav_id)
                # remain_time = self.cal_remain_time(uav_pos, path)
                uav_number = self.car_land_pressure[pos_id]['num']
                # rospy.loginfo("uav_time_list")
                # rospy.loginfo(uav_time_list)
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
                    # Can't conflict with the current line's UAVs
                    # Can't conflict with any other line's UAVs
                    uav_time_n = self.car_land_pressure[pos_id]['uav_time']
                    uav_time_list = []
                    for key, value in uav_time_n.items():
                        uav_time_list.append(value)
                    time_gap_one = self.determine_land_pos_back_time(pos_id)
                    if self.judge_time_conflict(remain_time, uav_time_list, time_gap_one) is False \
                    and self.judge_time_conflict(remain_time, time_list_all, time_gap_all) is False:
                        if pos_id not in cannot_choose_car_set:
                            rospy.loginfo("Satisfying the constraints can return line=" + str(pos_id)+ ",uav_id=" + str(uav_id) + " time_gap=" + str(time_gap_one))
                            rospy.logwarn("uav_number == 1, time_gap=" + str(time_gap_one))
                            rospy.logwarn("Satisfying the constraints can return line=" + str(pos_id)+ ",uav_id=" + str(uav_id) + " time_gap=" + str(time_gap_one)) 
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
                    
        # If the UAV hasn't taken off yet, then just calculate the total path distance divided by the speed
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
        
        # In flight. Then calculate the time remaining.
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
        # Calculate the distance from the current position to the end of the path segment
        if nearest_index >= len(path_list):
            return 0
        
        remaining_distance = self.euclidean_distance(current_position, path_list[nearest_index])

        # Calculate the total distance from the end point after this path segment
        for j in range(nearest_index, len(path_list) - 1):
            remaining_distance += self.euclidean_distance(path_list[j], path_list[j + 1])
        
        # Calculate remaining time based on speed
        remaining_time = remaining_distance / velocity
        return remaining_time


    def euclidean_distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def project_onto_segment(self, p, p1, p2):
        # computational vector
        v = np.array(p2) - np.array(p1)
        w = np.array(p) - np.array(p1)
        # Calculating Projection Scale
        c1 = np.dot(w, v)
        c2 = np.dot(v, v)
        b = c1 / c2
        # Return to projection point
        return np.array(p1) + b * v

    def is_point_on_segment(self, p, p1, p2):
        # Determine if the projected point is in the line segment
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
            # Now that we're in a straight line, the uav is officially on its way to the unloading point, 
            # so just calculate the distance to the third point, divide by the speed, and that's the flight time.
            # rospy.loginfo("--flight--")
            land_time = abs(path_height - path[3][2])/vel + uav_land_time
            remain_time = dis_n3/vel + land_time
            return remain_time
        
        if dis_n2 < dis_n3:
            # Just took off, still on the rise.
            # rospy.loginfo("--takeoff--")
            raise_time = (height_gap)/vel
            remain_time = long_dis/vel + raise_time + uav_land_time + abs(path_height - path[3][2])/vel
        else:
            # Begin land
            # rospy.loginfo("--landing--")
            remain_time = abs(uav_pos.z - path[3][2])/vel + uav_land_time
        return remain_time
    def update_uav_flight_info(self):
        # Update the status according to the current incoming UAVs status.
        
        # UAVs information at unloading points Dynamically updated
        self.unloading_cargo_pressure ={0: {'num':0, 'uav_time': {}},
                                        1: {'num':0, 'uav_time': {}},
                                        2: {'num':0, 'uav_time': {}},
                                        3: {'num':0, 'uav_time': {}},
                                        4: {'num':0, 'uav_time': {}},
                                        5: {'num':0, 'uav_time': {}}}
        # uav_time = {id : remain_time}
        # UAVs information at landing points
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
        # Initialize the departure flight times for the three lines and six stations, and store them in a dictionary.
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
        # Compute a route for the upcoming UAV departure
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
                            
        # Add current position and takeoff feedpoint
        route.append(now_pos)
        route.append(take_off_mid)
        
        # Adding a second feedforward point
        take_off_mid_two = copy.deepcopy(take_off_mid)
        take_off_mid_two.x += 12
        route.append(take_off_mid_two)
        
        # Add a third feedforward point - the first point of the route is the same xy, but the altitude is -61.
        path_1 = go_path[1]
        take_off_mid_three = Position(path_1[0], path_1[1], -61)
        route.append(take_off_mid_three)

        # Add route location
        for path in go_path[1:-1, :]:
            p_n = Position(path[0], path[1], path[2])
            route.append(p_n)
        
        # Add landing feed forward position
        go_mid_pos = self.unloading_cargo_id_to_mid_pos[unloading_station_id]['go_mid_pos']
        mid_go = Position(go_mid_pos[0], go_mid_pos[1], go_mid_pos[2])
        route.append(mid_go)
        
        # Add landing position
        p_n = go_path[-1, :]
        land_n = Position(p_n[0], p_n[1], p_n[2])
        land_n.z += -5
        route.append(land_n)

        return route        

    def set_route_for_uav_go(self, uav_id):
        # Calculate a route for the UAV that's about to depart
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
                            
        # Add current position and takeoff feedpoint
        route.append(now_pos)
        route.append(take_off_mid)
        
        # Adding a second feedforward point
        take_off_mid_two = copy.deepcopy(take_off_mid)
        take_off_mid_two.x += 12
        route.append(take_off_mid_two)
        
        # Add a third feedforward point - the first point of the route is the same xy, but the altitude is -61.
        path_1 = go_path[1]
        take_off_mid_three = Position(path_1[0], path_1[1], -61)
        route.append(take_off_mid_three)

        # Add route location
        for path in go_path[1:-1, :]:
            p_n = Position(path[0], path[1], path[2])
            route.append(p_n)
        
        # Add landing feed forward position
        go_mid_pos = self.unloading_cargo_id_to_mid_pos[unloading_station_id]['go_mid_pos']
        mid_go = Position(go_mid_pos[0], go_mid_pos[1], go_mid_pos[2])
        route.append(mid_go)
        
        # Add landing position
        p_n = go_path[-1, :]
        land_n = Position(p_n[0], p_n[1], p_n[2])
        land_n.z += -5
        route.append(land_n)

        return route
    
    def set_route_for_uav_back(self, uav_id, land_pos_id):
        # Here the land pos id is given to calculate the route
        unloading_station_id = self.uav_id_to_unloading_station_id[uav_id]
        back_path = self.unloadingStaion_id_to_path[unloading_station_id]['path_back']
        route = []
        now_pos = self.uav_info[uav_id].pos
        now_pos.z += -5
        
        route.append(now_pos)
        
        # Add back the unloading takeoff feed point  
        back_mid_pos = self.unloading_cargo_id_to_mid_pos[unloading_station_id]['back_mid_pos'] 
        mid_pos_one = Position(back_mid_pos[0], back_mid_pos[1], back_mid_pos[2])
        route.append(mid_pos_one)
        
        # Add two points on the route
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
        
        # Come back to the land before feedpoint 1
        land_mid_one = Position(path_2[0], path_2[1], path_2[2])
        # Come back to the land before feedpoint 2
        land_mid_two = copy.deepcopy(land_pos)
        # Come back to the land before feedpoint 3
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
                
        # Add back land points  
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
        
        
        # Let the UAVs that need to come back take off immediately.
        if len(self.uav_waiting_back) != 0:
            for uav in self.uav_waiting_back:
                uav_id = uav.sn
                
                if uav_id not in uav_back_id_set:
                    continue
                
                land_pos_id = uav_id_to_land_pos_id[uav_id]
                route = self.set_route_for_uav_back(uav_id, land_pos_id)
                # Copy this route to the uav id
                # self.uav_id_to_route_now[uav_id] = route
                self.set_uav_to_route_dict(uav_id, route)
                
                cmd_res[uav_id] = {"cmd": "DELIVER", "route":route}
            
        # Get those UAVs that need to go off the ground right now.
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
        # Generate discrete points
        # Extract x, y coordinates
        x1, y1 = startpos[0], startpos[1]
        x2, y2 = endpos[0], endpos[1]
        # Calculate total distance
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        # Calculate the number of points to be generated
        num_points = int(np.ceil(distance / spacing))
        # Generate discrete points
        x_points = np.linspace(x1, x2, num_points)
        y_points = np.linspace(y1, y2, num_points)
        # Combine x, y points into a list of discrete points
        points = list(zip(x_points, y_points))
        # for point in points:
        #     rospy.loginfo(point)
        return points

    def read_map_main(self):
        # Create a Map instance, assuming the map file path is "path/to/map/file"
        map_file_path = "voxel_map.bin"
        self.map_instance = pymtmap.Map(map_file_path)
        map_instance = self.map_instance

        # Check if the map is valid
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
            

        else:
            rospy.loginfo("Map is not valid.")
            
    def path_search_easy(self, start_pos, end_pos):
        # Calculate the path based on the given start and end points.
        # The calculated path may not be within the 60-120 altitude range. If it is, then the path is valid.
        # If it's not within 60-120, then we need to use jps to search for it.
        # But the good thing about this path is that it doesn't need to be turned around #

        
        line_points = self.generate_line_points(start_pos, end_pos)
        # Find the path safety height by looping.
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

        # obstacles = self.map_obs

        fig = plt.figure(figsize = (12, 9))
        ax = fig.add_subplot(111, projection='3d')

        # 绘制地图边界
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        # ax.set_zlim(z_min, z_max)
        ax.set_zlim(z_max, z_min)

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

        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')

        ax.set_title('3D Map with Obstacles')
        
        # plt.savefig("output.png")
        plt.show()
        
    def deal_with_map(self):
        rospy.loginfo("deal_with_map")
        for x in range(0, 700, 5):
            for y in range(0, 700, 5):
                for z in range(-222, 0, 5):
                    voxel = self.map_instance.Query(x, y, z)
                    if voxel.distance == 0 and voxel.semantic != 255:
                        self.map_obs.append([x, y, z])
        self.map_obs = np.array(self.map_obs)

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
        # Generate a 2D matrix at the current height matrix 0 is free 1 is obs
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
        # There's no UAV on the line at the moment. Determine if the line is capable of picking up an UAV
        # If there's no UAV in the line that's coming back 
        # If there's no AGV in land pos but there's a AGV behind it #
        if self.car_land_pressure[line_id]['num'] == 0:
            if self.judge_two_car_is_gw(line_id) is True:
                rospy.loginfo("two car is gw, need wait!")
                return False
            else:
                return True


    def determine_land_pos_back_time(self, line_id):
        # Depending on the current movement of the AGV,
        # the interval at which the UAV chooses its current return position will be determined
        # The previous interval was static, this time it's dynamic
        # This considers the case where there's already an UAV coming back
        # The interval time for the next UAV is determined.
        rospy.loginfo("line_id=" +str(line_id))
                 
        # There's an UAV in front of us, land pos, and an AGV waiting.
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
        # When the line id AGV is in gw state, determine how many AGVs are waiting in front of it
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
        # Determine if you're in a gw state
        # There's an UAV but no cargo
        if car_id in self.car_waiting_go_gw_set:
            return True
        elif car_id in self.car_running_set:
            if self.car_info[car_id].have_uav is True:
                uav_id = self.car_info[car_id].uav_sn
                if self.uav_info[uav_id].have_cargo is False:
                    return True
        return False
    
    def judge_car_in_aw_actual(self, car_id):
        # Determine if you're in an aw state
        # There's an UAV There's a cargo
        if car_id in self.car_waiting_go_gw_set:
            return True
        elif car_id in self.car_running_set:
            if self.car_info[car_id].have_uav is True:
                uav_id = self.car_info[car_id].uav_sn
                if self.uav_info[uav_id].have_cargo is True:
                    return True
        return False        

    def judge_car_in_pickup_actual(self, car_id):
        # To determine if it's in a pickup state
        # Or in motion There's no UAVs
        if car_id in self.car_waiting_pickup_set:
            return True
        elif car_id in self.car_running_set:
            if self.car_info[car_id].have_uav is False:
                return True
        return False
    def judge_two_car_is_gw(self, line_id):
        # Both AGVs are in gw
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        if self.judge_car_in_gw_actual(car_id_a) is True \
        and self.judge_car_in_gw_actual(car_id_b) is True:
            rospy.loginfo("两个车都处于gw的状态")
            rospy.logwarn("两个车都处于gw的状态")
            return True
        else:
            return False        

    def judge_one_car_is_comming(self, line_id):
        # There's no AGV in land pos but there's another AGV coming
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]
        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3    
        # One AGV is not in land pos 
        # And the other AGVs just flew an UAV and is coming over: 
        # y pos is greater than line y - 1, and the AGV is already in a wait pick up state
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
        # There's an AGV on land pos at any one time.
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        if car_id_a in self.land_pos_car_set or car_id_b in self.land_pos_car_set:
            return True        
        else:
            return False

    def judge_another_car_is_wait_gw(self, line_id):
        # No AGVs waiting in the back - AGVs not in the workspace yet!
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
        # There's no AGV waiting in the back. The other one's in the workspace.
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
        # There's no AGV waiting behind us. Another AGV is moving out of the workspace.
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]
        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3        

        # One AGV is in land pos 
        # while the other AGV is moving out of workspace:
        # y pos is greater than line y - 1, the AGV is already in waiting_go_aw_set
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
        # There's no AGV waiting in the back - other AGVs has flown the UAV and is on its way back.
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]

        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3

        # One AGV is in land pos 
        # while the other AGV is waiting for an UAV to take off: 
        # y pos is greater than line y - 1, and the AGV is already in waiting_go_aw_set
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
        # There's no AGV waiting in the back - the other AGV has flown the UAV and is on its way back.
        car_id_a, car_id_b = self.get_two_car_id(line_id)
        car_a = self.car_info[car_id_a]
        car_b = self.car_info[car_id_b]

        if line_id == 1:
            line_pos = self.para.land_p1
        elif line_id == 2:
            line_pos = self.para.land_p2
        elif line_id == 3:
            line_pos = self.para.land_p3

        # One AGV is in land pos 
        # And the other AGV just flew a UAV and is coming over: 
        # y pos is greater than line y - 1, and the AGV is already in a wait pick up state
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
        # Find the AGV that is in the land pos wait pos and is in the car_waiting_pickup state.
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