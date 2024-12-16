#!/bin/bash
# Function to check the last command status and exit if it failed
check_status() {
if [ $? -ne 0 ]; then
echo "Error: $1 failed"
exit 1
fi
}
# Run the first container
docker run -d --entrypoint /manager/run.sh --name race_scene_sdk_container uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:scene
check_status "docker run race_scene_sdk_container"

# Connect the first container to the network
docker network connect race_net race_scene_sdk_container --ip 192.168.100.5
check_status "docker network connect race_scene_sdk_container"
# Run the second container
docker run -d --name race_car_sdk_container uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:cars
check_status "docker run race_car_sdk_container"
# Connect the second container to the network
docker network connect race_net race_car_sdk_container --ip 192.168.100.2
check_status "docker network connect race_car_sdk_container"
# Run the third container
docker run -d --name race_drone_sdk_container -v /home/race_log/drone_log:/home/drone_log uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:drones
check_status "docker run race_drone_sdk_container"
# Connect the third container to the network
docker network connect race_net race_drone_sdk_container --ip 192.168.100.3
check_status "docker network connect race_drone_sdk_container"
# 如果有权限问题，可以手动去创建
if [ ! -d "/home/race_log" ]; then
echo "Directory /home/race_log does not exist. Creating it..."
mkdir -p /home/race_log
fi
# Run the fourth container
#docker run -d --name race_user_sdk_container -v /etc/localtime:/etc/localtime:ro -v /etc/timezone:/etc/timezone:ro uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:user
docker run -d -p 8888:8888 --name race_user_sdk_container \
--network race_net --ip 192.168.100.4 \
-e ROS_MASTER_URI=http://192.168.100.4:11311 \
-e ROS_IP=192.168.100.4 \
-v /home/race_log/user_log:/home/race_log \
-v /etc/localtime:/etc/localtime:ro \
-v /etc/timezone:/etc/timezone:ro \
race_user:lh
# uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:user

check_status "docker run race_user_sdk_container"
# Connect the fourth container to the network
#docker network connect race_net race_user_sdk_container --ip 192.168.100.4
#check_status "docker network connect race_user_sdk_container"
echo "All commands executed successfully"