from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

  # 关于一个 node 有哪些属性？
  # package、execute
  ### 自己一开始在写这里的时候忘记了有哪些属性，然后想去看之前写的代码
  ### 但是最终自己战胜了自己，去看了 Node 的构造函数，然后就知道啦
  param_node = Node(
    package = "basics",
    name = "param_node",
    executable = "param_basics",
    ## 这里关于参数的赋值格式出现了问题，没有想明白该如何赋值
    ## 在 python 语法的报错中有体现，但是自己不太熟悉 python 语法
    # ros_arguments = "num_of_params=4, topics_name=[topic1, topic2], topics_type=[topic_type1, topic_type2]"
    parameters = [{
            'num_of_params': 300,
            'topics_name': ['scan', 'image'],
            'topics_type': ['sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Image']
        }],
  )


  ld = LaunchDescription()
  ld.add_action(param_node)

  return ld