uint64 index
Position target_pos

# WaybillStatus
uint8 status
uint8 BILL_UNKOWN = 0
uint8 NOT_STARTED = 1
uint8 DELIVERY = 2
uint8 OVER = 3
uint8 BILL_ERROR = 5

DynamicPosition dynamic_position
# 新增配送时间点
int64 orderTime     # 订单生成时间点
int64 betterTime    # 最佳配送时间点
int64 timeout       # 超时时间点
