# Overview


URDF 不能单独使用，需要结合 Rviz 或 Gazebo，URDF 只是一个文件，需要在 Rviz 或 Gazebo 中渲染成图形化的机器人模型，当前，首先演示URDF与Rviz的集成使用，因为URDF与Rviz的集成较之于URDF与Gazebo的集成更为简单。

可视化模型需要三个 node 的参与：

- joint_state_publisher_gui ：负责发布机器人关节数据信息，通过joint_states话题发布
  topic_name: joint_state_publisher_gui
  topic_type: joint_state

- robot_state_publisher_node ：负责发布机器人模型信息robot_description，并将joint_states数据转换tf信息发布
  topic_name: robot_state_publisher
  topic_type: robot_description

- rviz2_node :负责显示机器人的信息


joint_state_publisher_gui 和 joint_state_publisher 的区别在于 joint_state_publisher_gui 运行时会跳出一个界面，通过界面可以操作 URDF 中能动的关节



