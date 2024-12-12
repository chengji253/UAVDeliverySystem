from race_demo.msg import Position

# 所有给定参数的初始化都在这里
class Para:
    
    def __init__(self) -> None:
        self.drone_station_origin = [180, 420, -16]
        # self.work_station_pos = [190, 425, -16]
        self.work_station_pos = Position(190, 425, -16)
        self.drone_store_pos = [185, 425, -16]
        
        self.safe_flight_min_height = -60
        self.safe_flight_max_height = -120
        
        self.take_off_p_1 = Position(15.5 + 180, 11 + 420, -16)
        # self.take_off_mid_p_1 = Position(14.5 + 180, 5 + 420, -16)
        
        self.take_off_p_2 = Position(15.5 + 180, 15 + 420, -16)
        # self.take_off_mid_p_2 = Position(14.5 + 180, 5 + 420, -16)
        
        self.take_off_p_3 = Position(18 + 180, 19 + 420, -16)
        # self.take_off_mid_p_2 = Position(14.5 + 180, 5 + 420, -16)

        self.land_p1 = Position(5  + 180, 11 + 420, -16)
        self.land_p2 = Position(9.5  + 180, 15 + 420, -16)
        self.land_p3 = Position(5  + 180, 19 + 420, -16)
        
        self.wait_p1 = Position(9 + 180, 11 + 420, -16)
        self.wait_p2 = Position(14  + 180, 15 + 420, -16)
        self.wait_p3 = Position(9  + 180, 19 + 420, -16)
        
        self.work_wait_p1 = Position(6 + 180, 7 + 420, -16)
        self.work_wait_p2 = Position(5 + 180, 15 + 420, -16)
        self.work_wait_p3 = Position(1 + 180, 19 + 420, -16)

        self.neighbor_pos = Position(6 + 180, 2 + 420, -16)
        
        self.line_1_car_id_set = {'SIM-MAGV-0001', 'SIM-MAGV-0002'}
        self.line_2_car_id_set = {'SIM-MAGV-0003', 'SIM-MAGV-0004'}
        self.line_3_car_id_set = {'SIM-MAGV-0005', 'SIM-MAGV-0006'}
        
        self.line_1_car_id_list = ['SIM-MAGV-0001', 'SIM-MAGV-0002']
        self.line_2_car_id_list = ['SIM-MAGV-0003', 'SIM-MAGV-0004']
        self.line_3_car_id_list = ['SIM-MAGV-0005', 'SIM-MAGV-0006']        

        self.back_time_gap_all = 5
        self.back_time_gap_land_p1 = 40
        self.back_time_gap_land_p2 = 46
        self.back_time_gap_land_p3 = 47

        self.go_time_gap_all = 40

        self.car_have_landing_uav_time_p1 = 28
        self.car_have_landing_uav_time_p2 = 6
        self.car_have_landing_uav_time_p3 = 6