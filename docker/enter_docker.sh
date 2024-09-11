#!/bin/bash

# 进入正在运行的容器
# -w /workspace：这将指定进入容器时的工作目录为 /workspace。
# 这样写有问题, -w 要在前面
# docker exec -it beaconbots -w /workspace /bin/bash


# 确定工作目录：通过 WORK_DIR 变量获取脚本所在目录的上级目录的绝对路径。
WORK_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P )"
# 加载环境变量：从 env/.env 文件中加载环境变量。
source ${WORK_DIR}/env/.env
DEV_CONTAINER=${CONTAINER_NAME}

# 设置开发容器名称：将 DEV_CONTAINER 变量设置为 KIT_TYPE。如果 KIT_ID 不为空，则将 DEV_CONTAINER 变量附加上 KIT_ID。
# if [ "${KIT_ID}" != "" ];then
#     DEV_CONTAINER="${DEV_CONTAINER}_${KIT_ID}"
# fi

# 检查并启动容器：检查 Docker 容器是否已经在运行，如果没有，则启动它。
# 将容器名称的输出通过管道传递给 grep -F 表示固定字符串匹配 -x 表示完全匹配整行
# docker ps --format 选项用于自定义 docker ps 命令输出的格式。
# 它通过 Go 模板语法来定义输出的内容和格式。这使得用户可以仅显示所需的信息，而不是默认的完整表格视图。
# --format 选项后跟一个 Go 模板字符串，用于指定要输出的信息。
# {{.ID}}：容器的 ID。
# {{.Image}}：容器使用的镜像。
# {{.Command}}：容器启动时运行的命令。
# {{.CreatedAt}}：容器的创建时间。
# {{.RunningFor}}：容器运行的时间。
# {{.Ports}}：容器的端口映射。
# {{.Names}}：容器的名称。
# {{.Status}}：容器的状态。
# {{.Labels}}：容器的标签。
# {{.Mounts}}：容器的挂载点。
# {{.Networks}}：容器连接的网络。

# 仅显示容器名称：  这个命令仅输出当前运行的容器的名称，每行一个容器名称。
# docker ps --format "{{.Names}}"

# 显示容器的 ID 和状态： 这个命令将输出容器的 ID 和状态。
# docker ps --format "ID: {{.ID}}, Status: {{.Status}}"

# 显示容器的名称和映像： 这个命令输出每个容器的名称和其使用的镜像。
# docker ps --format "{{.Names}} - {{.Image}}"
docker ps --format "{{.Names}}" | grep -Fx "${DEV_CONTAINER}" 1>/dev/null
# 这里的 $? 是检测 grep 的退出状态是什么
# grep 命令的退出状态（$?）用来判断容器是否存在。
# 如果 grep 返回状态码 1，表示没有找到匹配的容器名称，即容器没有运行。
# 在这种情况下，执行 docker start "${DEV_CONTAINER}" 启动指定的容器。
if [ $? == 1 ]; then
    docker start "${DEV_CONTAINER}"
fi

# 允许本地用户访问 X 服务器：允许 Docker 容器访问宿主机的 X 服务器，以便容器中的应用可以显示图形界面。
xhost +local:root 1>/dev/null 2>&1

# 在容器中执行命令：以指定用户身份（"${USER}"）进入容器并启动一个交互式 Bash shell，同时设置 HISTFILE 环境变量以指定历史记录文件的存储位置。
docker exec \
    -it  \
    -w /workspace \
    "${DEV_CONTAINER}" \
    /bin/bash

# 恢复 X 服务器的访问权限：完成操作后，恢复 X 服务器的访问权限设置。
# 是的，xhost -local:root 通常是在退出 Docker 容器之后执行的。这样做是为了在你完成容器中的操作后，撤销之前给 Docker 容器提供的 X 服务器访问权限，从而恢复 X 服务器的安全设置。
xhost -local:root 1>/dev/null 2>&1
