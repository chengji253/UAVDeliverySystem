string sn

uint8 state
uint8 READY = 0
uint8 ON_CAR = 1
uint8 FLYING_GO = 2
uint8 FLYING_BACK = 3
uint8 WAITING_GO = 4
uint8 WAITING_BACK = 5

Position pos
bool have_cargo
uint32 cargo_id
float64 remaining_battery
float64 remaining_flytime