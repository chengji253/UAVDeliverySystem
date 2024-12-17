#!/usr/bin/env python3
import rospy
import threading
import json
import sys
sys.path.append('/home/catkin_ws/src/race_demo/src/scripts/fsm')

from car_fsm import Car_FSM, CarState
from uav_fsm import UAVState

from race_demo.msg import PanoramicInfo
from race_demo.msg import SelfCommand
from race_demo.msg import Position
from race_demo.msg import SelfUAVSwarm, SelfUAVState
from race_demo.msg import SelfCarSwarm, SelfCarState
from race_demo.msg import SelfStateChange


class threadModule:
    def __init__(self):
        self.vehicle_list = {}
        self.thread_list = {}

    # Create a thread for the AGV
    def createThread(self, sn, config):
        if sn in self.thread_list:
            rospy.logerr("Thread already exists")
            return
        car_fsm = Car_FSM(config)
        thread = threading.Thread(target=car_fsm.execCmd, daemon=True)
        self.thread_list[sn] = thread
        thread.start()
        self.vehicle_list[sn] = car_fsm
        while not car_fsm.getState()['state'] == CarState.WAITING_PICKUP:
            rospy.sleep(0.1)
        rospy.loginfo(f"Car {sn} is ready")

class carMangageNode:
    def __init__(self):
        rospy.init_node('car_mangage_node')
        rospy.set_param("/rosout", "/home/catkin_ws/logs")
        self.init_car_num = rospy.get_param('/car_mangage_node/init_car_num', 2)
        self.info_sub = rospy.Subscriber('/panoramic_info', PanoramicInfo, self.panoramicInfoCallback, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/self_car_command', SelfCommand, self.commandCallback, queue_size=10)
        self.uav_swarm_sub = rospy.Subscriber('/uav_swarm', SelfUAVSwarm, self.uavSwarmCallback, queue_size=10)
        self.state_change_pub = rospy.Publisher('/state_change', SelfStateChange, queue_size=100)
        self.car_swarm_pub = rospy.Publisher('/car_swarm', SelfCarSwarm, queue_size=100)
        
        self.car_thread = threadModule()

        # Read config file and info
        with open('/config/config.json', 'r') as file:
            self.config = json.load(file)
        # Extract and store a list of AGVs info
        self.car_infos = self.config['taskParam']['magvParamList']
        self.car_sn_list = [car['magvSn'] for car in self.car_infos]

        loading_pos = Position(
                self.config['taskParam']['loadingCargoPoint']['x'],
                self.config['taskParam']['loadingCargoPoint']['y'],
                self.config['taskParam']['loadingCargoPoint']['z'])
        self.temp_config = {
            "peer_id": self.config['peerId'],
            "task_guid": self.config['taskParam']['guid'],
            "get_uav_status": self.getUAVStatus,
            "get_car_status": self.getCarStatus,
            "get_ready_uav": self.getReadyUAV,
            "publish_change_state": self.publishUAVState,
            "loading_pos": loading_pos
        }

        self.ready_uav_list = []

        rospy.loginfo("Car mangage node started")
    
    # Simulate callback functions to obtain real-time information
    def panoramicInfoCallback(self, panoramic_info):
        self.car_phys_status = panoramic_info.cars
        self.uavs_phys_status = panoramic_info.drones

    # Command callback function
    def commandCallback(self, command):
        if command.type == SelfCommand.LOAD_CARGO \
        or command.type == SelfCommand.UAV_CHARGE \
        or command.type == SelfCommand.UAV_RETRIEVE \
        or command.type == SelfCommand.RECEIVE_UAV \
        or command.type == SelfCommand.MOVE_TO_TARGET:
            self.car_thread.vehicle_list[command.car_sn].cmd_list.append(command)

    def uavSwarmCallback(self, swarm):
        self.ready_uav_list = swarm.READY

    # Create a thread for the AGV
    def createCar(self, sn):
        self.temp_config["sn"] = sn
        self.car_thread.createThread(sn, self.temp_config)

    # Get the status of the UAV
    def getUAVStatus(self, uav_sn):
        uav_status = next((uav for uav in self.uavs_phys_status if uav.sn == uav_sn), None)
        return uav_status
    
    # Get the status of the AGV
    def getCarStatus(self, car_sn):
        car_status = next((car for car in self.car_phys_status if car.sn == car_sn), None)
        return car_status
    
    def getReadyUAV(self):
        if len(self.ready_uav_list) == 0:
            return ''
        else:
            return self.ready_uav_list[0].sn
    
    def msgWrite(self, sn, state):
        self_state = SelfCarState()
        self_state.sn = sn
        self_state.state = state['state'].value
        self_state.pos = state['pos']
        self_state.have_uav = state['have_uav']
        self_state.uav_sn = state['uav_sn']
        self_state.remaining_runtime = state['remaining_runtime']
        return self_state
    
     # Publish the swarm state
    def pubSwarmState(self):
        swarm_state = SelfCarSwarm()
        for k, v in self.car_thread.vehicle_list.items():
            state = v.getState()
            if state['state'] == CarState.WAITING_PICKUP:
                swarm_state.WAITING_PICKUP.append(self.msgWrite(k, state))
            elif state['state'] == CarState.RUNNING:
                swarm_state.RUNNING.append(self.msgWrite(k, state))
            elif state['state'] == CarState.WAITING_GOTO_GW:
                swarm_state.WAITING_GOTO_GW.append(self.msgWrite(k, state))
            elif state['state'] == CarState.WAITING_GOTO_AW:
                swarm_state.WAITING_GOTO_AW.append(self.msgWrite(k, state))
            elif state['state'] == CarState.WAITING_UAV_WORKING:
                swarm_state.WAITING_UAV_WORKING.append(self.msgWrite(k, state))
        # print(swarm_state)
        self.car_swarm_pub.publish(swarm_state)

    def publishUAVState(self, uav_sn, state):
        msg = SelfStateChange()
        msg.uav_sn = uav_sn
        msg.state = state.value
        self.state_change_pub.publish(msg)

    def main(self):
        rospy.sleep(3.0)
        # print("init_car_num=")
        # print(self.init_car_num)
        if self.init_car_num > len(self.car_sn_list):
            self.init_car_num = len(self.car_sn_list)
        for i in range(self.init_car_num):
            self.createCar(self.car_sn_list[i])
        while not rospy.is_shutdown():
            self.pubSwarmState()
            rospy.sleep(0.5)
        

if __name__ == "__main__":
    pipeline = carMangageNode()
    pipeline.main()
    rospy.spin()
    
