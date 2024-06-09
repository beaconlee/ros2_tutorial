
# basics

### Create a node in ROS2

```bash
ros2 pkg create package_name --build_type 构建类型 --dependencies 依赖列表 --node_name 可执行程序名称
```


### param


```bash
ros2 run basics param_basics --ros-args -p num_of_params:=2 -p topics_name:='[/sensor/carame, /sensor/lisar]' -p topics_type:='[sensor, sensor]'
```




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