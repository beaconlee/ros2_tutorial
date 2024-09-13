#!/bin/bash
WORK_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"

# echo ${WORK_DIR}

# 加载 .env 文件中的环境变量
if [ -f ${WORK_DIR}/env/.env ]; then
  source ${WORK_DIR}/env/.env
else
  echo ".env file not found!"
  exit 1
fi

# 拼接完整的镜像名称 (如: beacon:humble1.0.1)
FULL_IMAGE_NAME="${IMAGE_NAME}:${IMAGE_TAG}${IMAGE_VERSION}"
# echo ${FULL_IMAGE_NAME}
CONTAINER_NAME="beaconbots"
# echo ${CONTAINER_NAME}

# docker ps -aq -f name=${CONTAINER_NAME} 命令列出所有符合指定名称的容器的 ID。
# -a 选项表示包括所有容器，不管它们是否正在运行。
# -q 选项表示仅输出容器的 ID。
# -f name=${CONTAINER_NAME} 选项用于过滤出名称包含 ${CONTAINER_NAME} 的容器。
# $(...) 是命令替换，用于获取命令的输出结果。
# 如果结果不为空（即存在指定名称的容器），if 条件为真。
# 检查是否已经有名为 beaconbots 的容器
if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
    echo "Stopping and removing existing container: ${CONTAINER_NAME}"
    docker stop ${CONTAINER_NAME}
    docker rm ${CONTAINER_NAME}
fi

# 创建并启动新的容器
echo "Creating and starting new container with image: ${FULL_IMAGE_NAME}"
docker run -it -d --privileged \
    -v /home/ros/ros2_tutorial:/workspace \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=:1 \
    --name ${CONTAINER_NAME} \
    ${FULL_IMAGE_NAME}

