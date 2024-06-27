from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

def generate_launch_description():

  pkg_dir = get_package_share_directory('basics')
  param_file = os.path.join(pkg_dir, 'config', 'param.yaml')

  publish_node = Node(
        executable="publisher_basics",
        package = "basics",
        name="first_publisher222",
  )

  subscriber_node = Node(
    executable="subscriber_basics",
    package="basics",
    name="subscriber222"
  )

  param_node = Node(
    executable="param_basics",
    package="basics",
    name="param_node",
    # parameters=[{
    #   'num_of_params':22,
    #   'topics_name':['tian', 'xia', 'wu', 'shuang'],
    #   'topics_type':['tian', 'xia', 'wu', 'di']
    # }],
    
    # 使用配置文件方式传递参数
    parameters=[param_file],
    output='screen'
  )


  ld = LaunchDescription()
  ld.add_action(publish_node)
  ld.add_action(subscriber_node)
  ld.add_action(param_node)


  return ld