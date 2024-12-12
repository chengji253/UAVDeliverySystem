# SelfCmdType
uint8 type
uint8 NO_COMMAND = 0
# for UAV
uint8 DELIVER = 1             # 必须包含route
# for CAR
uint8 MOVE_TO_TARGET = 2      # 必须包含route
uint8 LOAD_CARGO = 3          # 必须包含cargo_id 
uint8 UAV_CHARGE = 4          # 
uint8 UAV_RETRIEVE = 5        # 
uint8 RECEIVE_UAV = 6         # 必须包含route和uav_sn

Position[] route
uint32 cargo_id
string car_sn
string uav_sn
