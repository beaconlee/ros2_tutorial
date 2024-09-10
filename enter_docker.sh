#!/bin/bash

# 进入正在运行的容器
# -w /workspace：这将指定进入容器时的工作目录为 /workspace。
# 这样写有问题, -w 要在前面
# docker exec -it beaconbots -w /workspace /bin/bash

docker exec -it -w /workspace beaconbots /bin/bash
