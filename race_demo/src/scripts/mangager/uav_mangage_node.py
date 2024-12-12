#!/usr/bin/env python3
import rospy
import threading
import json
import sys
sys.path.append('/home/catkin_ws/src/race_demo/src/scripts/fsm')

from uav_fsm import UAV_FSM, UAVState

from race_demo.msg import PanoramicInfo
from race_demo.msg import SelfCommand
from race_demo.msg import Position
from race_demo.msg import SelfUAVSwarm, SelfUAVState
from race_demo.msg import SelfStateChange


class threadModule:
    def __init__(self):
        self.vehicle_list = {}
        self.thread_list = {}

    # 创建并启动一个线程
    def createThread(self, sn, config):
        if sn in self.thread_list:
            rospy.logerr("Thread already exists")
            return
        uav_fsm = UAV_FSM(config)
        thread = threading.Thread(target=uav_fsm.execCmd, daemon=True)
        self.thread_list[sn] = thread
        thread.start()
        self.vehicle_list[sn] = uav_fsm
        while not uav_fsm.getState()['state'] == UAVState.READY:
            rospy.sleep(0.1)
        rospy.loginfo(f"UAV {sn} is ready")

class uavMangageNode:
    def __init__(self):
        # 初始化ros全局变量
        rospy.init_node('uav_mangage_node')
        rospy.set_param("/rosout", "/home/catkin_ws/logs")
        self.init_uav_num = rospy.get_param('/uav_mangage_node/init_uav_num', 2)
        self.info_sub = rospy.Subscriber('/panoramic_info', PanoramicInfo, self.panoramicInfoCallback, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/self_uav_command', SelfCommand, self.commandCallback, queue_size=10)
        self.state_change_sub = rospy.Subscriber('/state_change', SelfStateChange, self.stateChangeCallback, queue_size=10)
        self.uav_swarm_pub = rospy.Publisher('/uav_swarm', SelfUAVSwarm, queue_size=100)
        
        self.uav_thread = threadModule()

        # 读取配置文件和信息
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        # 提取并存储无人机的序列号列表
        self.drone_infos = self.config['taskParam']['droneParamList']
        self.drone_sn_list = [drone['droneSn'] for drone in self.drone_infos]

        loading_pos = Position(
                self.config['taskParam']['loadingCargoPoint']['x'],
                self.config['taskParam']['loadingCargoPoint']['y'],
                self.config['taskParam']['loadingCargoPoint']['z'])
        self.temp_config = {
            "peer_id": self.config['peerId'],
            "task_guid": self.config['taskParam']['guid'],
            "get_uav_status": self.getUAVStatus,
            "loading_pos": loading_pos
        }

        rospy.loginfo("UAV mangage node started")

    # 仿真回调函数，获取实时信息
    def panoramicInfoCallback(self, panoramic_info):
        self.uavs_phys_status = panoramic_info.drones

    # 指令回调函数
    def commandCallback(self, command):
        if command.type == SelfCommand.DELIVER:
            self.uav_thread.vehicle_list[command.uav_sn].cmd_list.append(command)

    # 状态改变回调函数
    def stateChangeCallback(self, msg):
        sn = msg.uav_sn
        if sn == '':
            return
        self.uav_thread.vehicle_list[sn].stateChange(UAVState(msg.state))

    # 创建一个无人机
    def createUAV(self, sn):
        self.temp_config["sn"] = sn
        self.uav_thread.createThread(sn, self.temp_config)

    # 获取指定无人机状态
    def getUAVStatus(self, uav_sn):
        uav_status = next((uav for uav in self.uavs_phys_status if uav.sn == uav_sn), None)
        return uav_status
    
    # 快速消息赋值
    def msgWrite(self, sn, state):
        self_state = SelfUAVState()
        self_state.sn = sn
        self_state.state = state['state'].value
        self_state.pos = state['pos']
        self_state.have_cargo = state['have_cargo']
        self_state.cargo_id = state['cargo_id']
        self_state.remaining_battery = state['remaining_battery']
        self_state.remaining_flytime = state['remaining_flytime']
        return self_state

    # 发布集群状态
    def pubSwarmState(self):
        swarm_state = SelfUAVSwarm()
        for k, v in self.uav_thread.vehicle_list.items():
            state = v.getState()
            if state['state'] == UAVState.READY:
                swarm_state.READY.append(self.msgWrite(k, state))
            elif state['state'] == UAVState.ON_CAR:
                swarm_state.ON_CAR.append(self.msgWrite(k, state))
            elif state['state'] == UAVState.FLYING_GO:
                swarm_state.FLYING_GO.append(self.msgWrite(k, state))
            elif state['state'] == UAVState.FLYING_BACK:
                swarm_state.FLYING_BACK.append(self.msgWrite(k, state))
            elif state['state'] == UAVState.WAITING_GO:
                swarm_state.WAITING_GO.append(self.msgWrite(k, state))
            elif state['state'] == UAVState.WAITING_BACK:
                swarm_state.WAITING_BACK.append(self.msgWrite(k, state))
        # print(swarm_state)
        self.uav_swarm_pub.publish(swarm_state)
                
    def main(self):
        rospy.sleep(3.0)
        if self.init_uav_num > len(self.drone_sn_list):
            self.init_uav_num = len(self.drone_sn_list)
        for i in range(self.init_uav_num):
            self.createUAV(self.drone_sn_list[i])
        while not rospy.is_shutdown():
            self.pubSwarmState()
            rospy.sleep(0.5)
        

if __name__ == "__main__":
    pipeline = uavMangageNode()
    pipeline.main()
    rospy.spin()