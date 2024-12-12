string sn

uint8 state
uint8 WAITING_PICKUP = 0
uint8 RUNNING = 1
uint8 WAITING_GOTO_GW = 2
uint8 WAITING_GOTO_AW = 3
uint8 WAITING_UAV_WORKING = 4

Position pos
bool have_uav
string uav_sn
float64 remaining_runtime