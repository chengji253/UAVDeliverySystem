# UAV-delivery-management-system
A Real-Time System for Scheduling and Managing UAV Delivery in Urban 



# 💫 Introduction

## Abstract
As urban logistics demand continues to grow, UAV delivery has become a key solution to improve delivery efficiency, reduce traffic congestion, and lower logistics costs. However, to fully leverage the potential of UAV  delivery networks, efficient swarm scheduling and management are crucial. In this paper, we propose a real-time scheduling and management system based on the ``Airport-Unloading Station" model, aiming to bridge the gap between high-level scheduling algorithms and low-level execution systems. This system, acting as middleware, accurately translates the requirements from the scheduling layer into specific execution instructions, ensuring that the scheduling algorithms perform effectively in real-world environments. Additionally, we implement three collaborative scheduling schemes involving autonomous ground vehicles (AGVs), unmanned aerial vehicles (UAVs), and ground staff to further optimize overall delivery efficiency. 
Through extensive experiments, this study demonstrates the rationality and feasibility of the proposed management system, providing practical solution for the commercial application of UAVs delivery in urban.

## System overview

![](/pic/frame.jpg)


# 📷 Experiments

<img src="/pic/one.jpg" width="50%">
<img src="/pic/two.jpg" width="50%">
<img src="/pic/three.jpg" width="50%">
<img src="/pic/1.jpg" width="90%">

# 🎵 Quick start

1-Pull the image through docker ([race_images.sh](/shell_file/race_images.sh)
) 

chmod +x race_images.sh
./race_images.sh

docker network create --subnet=192.168.100.0/24 race_net
docker network inspect race_net

2-Start the docker file ([start_race.sh](/shell_file/start_race.sh))

chmod +x start_race.sh
./start_race.sh

3- Put the race_demo into the container of the race_user_sdk_container/home/, then you can use ros to compile and run.

4-Stop the docker file ([stop_race.sh](/shell_file/stop_race.sh))

## Configuration Files

In the configuration folder you can see the car drone and scene parameter settings as well as the order details.

[car config](config/car/config.json)

[drone config](config/drone/drone.json)

[scene config](config/scene/scene.config)

[order config](config/user/config.json)

order four time attributes
```json
{
                "betterTime": 1500,
                "cargoParam": {
                    "birthplace": {
                        "x": 190,
                        "y": 425,
                        "z": -16
                    },
                    "index": 47,
                    "name": "测试货物47",
                    "weight": 3.947
                },
                "index": 47,
                "loadingStationInfo": {
                    "$ref": "$.taskParam.waybillParamList[46].cargoParam.birthplace"
                },
                "orderTime": 300,
                "targetPosition": {
                    "x": 490,
                    "y": 390,
                    "z": -22
                },
                "timeout": 1800
            },
```
