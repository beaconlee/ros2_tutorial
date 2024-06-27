import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  pkg_dir = get_package_share_directory('basics')
  param_file = os.path.join(pkg_dir, 'config', 'param.yaml')
  
  param_node = Node(
    package="basics",
    executable="param_basics",
    # 这个是 node 的名字，要和 C++ 代码中的 Node 名字对应上，如果有问题，程序无法正常执行
    # 只是针对于需要传参的 Node，如果 Node 不需要传递参数，好像名字不一样也能正常启动
    name="param_node",
    parameters=[param_file]
  )

  ld = LaunchDescription()
  ld.add_action(param_node)

  return ld