#!/bin/bash

# 登陆到腾讯云 docker 服务
docker login uav-challenge.tencentcloudcr.com --username 'tcr$user' --password gXWWpxhO9igRnXzYYV58UexxS1Gw8VQY

# 要拉取的镜像列表（优先使用腾讯云）
images=(
"uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:cars"
"uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:drones"
"uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:scene"
"uav-challenge.tencentcloudcr.com/uav_challenge_2024/sdk:user"
)

# 备用源（Docker Hub）
#images=(
# "marcobright2023/mtuav-competition-2024:cars"
# "marcobright2023/mtuav-competition-2024:drones"
# "marcobright2023/mtuav-competition-2024:scene"
# "marcobright2023/mtuav-competition-2024:user"
#)

# 循环拉取每个镜像
for image in "${images[@]}"; do
echo "Pulling $image..."
docker pull "$image"
if [ $? -ne 0 ]; then
echo "Failed to pull $image"
exit 1
fi
done
echo "All images pulled successfully!"