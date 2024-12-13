#!/bin/bash
# Function to check the last command status and exit if it failed
check_status() {
if [ $? -ne 0 ]; then

echo "Error: $1 failed"
exit 1
fi
}
# Stop and remove the first container
docker stop race_scene_sdk_container
check_status "docker stop race_scene_sdk_container"
docker rm race_scene_sdk_container
check_status "docker rm race_scene_sdk_container"
# Stop and remove the second container
docker stop race_car_sdk_container
check_status "docker stop race_car_sdk_container"
docker rm race_car_sdk_container
check_status "docker rm race_car_sdk_container"
# Stop and remove the third container
docker stop race_drone_sdk_container
check_status "docker stop race_drone_sdk_container"
docker rm race_drone_sdk_container
check_status "docker rm race_drone_sdk_container"
# Stop and remove the fourth container
docker stop race_user_sdk_container
check_status "docker stop race_user_sdk_container"
docker rm race_user_sdk_container
check_status "docker rm race_user_sdk_container"
echo "All containers have been stopped and removed successfully."