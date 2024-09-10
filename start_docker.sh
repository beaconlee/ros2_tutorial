#!/bin/bash

# 检查是否已经有名为 beaconbots 的容器
if [ "$(docker ps -aq -f name=beaconbots)" ]; then
    # 如果容器已存在且处于停止状态，则启动它
    if [ "$(docker ps -q -f name=beaconbots)" ]; then
        echo "Container beaconbots is already running."
    else
        docker start beaconbots
        echo "Started existing container: beaconbots"
    fi
else
    # 否则创建并启动新容器
    docker run -d --privileged \
        -v /home/ros/humble:/workspace \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -e DISPLAY=:0 \
        --name beaconbots \
        osrf/ros:humble-desktop
    echo "Created and started new container: beaconbots"
fi
