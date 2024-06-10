
# basics

### Create a node in ROS2

```bash
ros2 pkg create package_name --build_type 构建类型 --dependencies 依赖列表 --node_name 可执行程序名称
```


## # launch

在 ros2 中，一个机器人应用会包含很多的 node，每个 node 又都有自己的 param、options，为了方便、快捷、同步的启动这些 node，可以使用 launch 文件来快速启动。

launch 是一个 python 程序，包含了 generate_launch_description() 函数，其返回 LaunchDescription 对象，一个 LaunchDescription 对象应该包含以下主要内容：

- Node action: to run a program
- IncludeLaunchDescription action: to include other launch
- DeclareLaunchArgument action: to declare launch argument
- SetEnvironmentVariable action:  to set an environment variable

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  pub_cmd = Node(
    package = 'basics'
    executable = 'publisher'
    output = 'screen'
  )

  sub_cmd = Node(
    package = 'basics'
    executable = 'subscriber'
    output = 'screen'
  )

  ld = LaunchDescription()
  ld.add_action(pub_cmd)
  ld.add_action(sub_cmd)

  return ld
```

在 cmake 中，install命令用于将构建过程中生成的文件、目录或目标安装到指定的位置。

为了使 launch 文件生效，需要在 CMakeLists.txt 中添加

``` cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

其中：

- DIRECTORY launch：这个选项告诉 cmake，将项目根目录下面的'launch'目录及其内容作为一个整体进行安装。
- DESTINATION share/${PROJECT_NAME}：这个选项指定了安装的位置。具体来说：
  - share：是 ros2 安装包结构中的一个标准目录，用于存放共享数据，包含 launch 文件、配置文件、资源文件等。
  - ${PROJECT_NAME}：这是一个 cmake 变量，通常在 CMakeLists.txt 文件中定义，表示项目的名称。

这样做有三个好处：

1. 规范化结构：在 ros2 的生态系统中，有一套约定成俗的目录结构，将 launch 文件安装到'share/${PROJECT_NAME}'目录下，符合 ros2 的包结构规范，这样可以确保所有的依赖和工具都能正确找到并使用这些 launch 文件。
2. 可移植性：安装过程确保了launch文件在安装路径中正确存在，这样无论在开发环境还是在运行时环境中，都可以通过标准的路径访问和使用这些文件。
3. 包管理：通过CMake的安装命令，launch文件会被包含在生成的ROS2包中。这意味着当你将包发布或分发给其他用户时，这些launch文件也会随包一起分发，确保完整性和一致性。


最终调用命令来执行：

```bash 
ros2 launch package_name xxx_launch.py
```




### param

在 ros2 中，为 node 添加的 parameters:

1. 使用 declare_parameter 来声明参数的名字和默认值（使用参数类型的默认构造函数）。
2. 使用 get_parameter 获取参数的值。



可以在启动时提供参数的值：

```bash
ros2 run basics param_basics --ros-args -p num_of_params:=2 -p topics_name:='[/sensor/carame, /sensor/lisar]' -p topics_type:='[sensor, sensor]'
```

也可以将参数的值写到 launch 文件中：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  param_node = Node(
    package = "basics",
    name = "param_node",
    executable = "param_basics",

    parameters = [{
            'num_of_params': 300,
            'topics_name': ['scan', 'image'],
            'topics_type': ['sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Image']
        }],
  )

  ld = LaunchDescription()
  ld.add_action(param_node)

  return ld
```

在 launch 文件中，可以采用更容易理解和维护的参数加载方式：将参数存放到一个配置文件中，配置文件存放于 config，目录下，使用 yaml 格式，类似于 launch 文件，也需要在 CMakeLists.txt 文件中进行安装

```cmake
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

## 可以将 config 和 launch 结合写到一起
install(DIRECTORY config install DESTINATION share/${PROJECT_NAME})
```


然后在 launch 文件中调用 config 文件

```yaml
# 这个是 node 的名字，如果 node 的名字对不上也会报错
param_node:
  # 这里的缩进很严谨，之前这里没有缩进，就报错了
  # 这个必须要有，ros2 以这个标签来解析参数
  ros__parameters:
      num_of_params: 4
      topics_name: [scan, image]
      topics_type: [sensor_msgs/msg/LaserScan, sensor_msgs/msg/Image]
```

```python
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg_dir = get_package_share_directory('basics')
  param_file = os.path.join(pkg_dir, 'config', 'param.yaml')
  
  param_node = Node(
    package="basics",
    # 这个是可执行文件的名字
    executable="param_basics",
    # 这个是 node 的名字，要和 C++ 代码中的 Node 名字对应上，如果有问题，程序无法正常执行
    name="param_node",
    parameters=[param_file]
  )

  ld = LaunchDescription()
  ld.add_action(param_node)

  return ld
```

调用命令：

‵``bash
ros2 launch basics param_file_launch.py 

# 如果不使用 launch 文件的话，可以使用 --params-file 来传入参数文件
ros2 run br2 basics param_basics --ros-args --params-file install/basics/share/basics/config/params.yaml
‵``

### executors

在 ros2 中，使用 Executors 来管理同一进程中的多个节点，在其内部使用共享内存方式来加速通信。
有单线程和多线程两种：

- rclcpp::executors::SingleThreadedExecutor
- rclcpp::executors::MultiThreadedExecutor

创建好 Executors 的实例后，调用其 add_node(node) 函数将 node 加入其中，然后调用 spin() 函数（对其中的每个 node 调用 spin() 函数），保持执行。

在 ros2 中，spin函数用于处理回调。这些回调可能是订阅者接收到的消息、服务调用、定时器触发等。spin函数会使程序进入一个循环，不断地检查和执行这些回调，从而保持节点的活跃和响应。

> 回调处理：ros2 应用程序核心在于响应各种事件（如消息接收、服务调用）。'spin'函数会不断检查这些事件并调用相应的回调函数。
> 保持节点活跃：'spin' 函数使节点保持活跃状态，确保其持续处理消息和事件。
> 主循环：'spin'函数相当于 ros2 的主循环，它会阻塞当前线程，直到节点被手动关闭或者程序终止。