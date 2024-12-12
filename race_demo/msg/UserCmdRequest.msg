uint32 peer_id
string task_guid

# UserCmdType
uint8 type
uint8 USER_CMD_NONE = 0
uint8 USER_CMD_DRONE_EXEC_ROUTE = 1
uint8 USER_CMD_CAR_EXEC_ROUTE = 2
uint8 USER_CMD_MOVE_DRONE_ON_CAR = 3
uint8 USER_CMD_MOVE_DRONE_ON_BIRTHPLACE = 4
uint8 USER_CMD_MOVE_CARGO_IN_DRONE = 5
uint8 USER_CMD_DRONE_RELEASE_CARGO = 6
uint8 USER_CMD_DRONE_BATTERY_REPLACEMENT = 7

DroneWayPointInfo drone_way_point_info
CarRouteInfo car_route_info
BindDrone binding_drone
UnBindInfo unbind_info
BindCargo binding_cargo
DroneMsg drone_msg