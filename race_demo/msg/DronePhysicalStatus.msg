string sn
uint64 timestamp
uint32 peer_id
string task_guid
DynamicPosition pos

# DroneWorkState
uint8 drone_work_state
uint8 UNKOWN = 0
uint8 READY = 1
uint8 TAKEOFF = 2
uint8 FLYING = 3
uint8 LANDING = 4
uint8 CHARGING_BATTERY = 5
uint8 LOADING_CARGO = 6
uint8 ERROR = 10

float64 remaining_capacity
string description
int32 bind_cargo_id