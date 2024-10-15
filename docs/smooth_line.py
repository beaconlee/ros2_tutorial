#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import csv
import os
import json
import sys
import math
import numpy as np
import copy
from math import sqrt
from scipy import interpolate
from scipy.interpolate import CubicSpline

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


class TopoMapCsv:
  def __init__(self, names, predecessors, successors, left_widths, right_widths):
    self.names = names
    self.predecessors = predecessors
    self.successors = successors
    self.left_widths = left_widths
    self.right_widths = right_widths


  # 添加一个 debug 函数来输出所有数据
  def debug(self):
    print(f'TopoMapCsv Data:')
    print(f'Names: {self.names}')
    print(f'Predecessors: {self.predecessors}')
    print(f'Successors: {self.successors}')
    print(f'Left Widths: {self.left_widths}')
    print(f'Right Widths: {self.right_widths}')


class utils:
  @staticmethod
  def check_distance_exceeds(point1, point2, distance):
    if utils.distance(point1, point2) > distance:
      return True
    else:
      return False

  @staticmethod
  def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

  @staticmethod
  def sample_point(points):
    last_point = points[0]
    sample_points = []
    sample_points.append(points[0])

    for point in points:
      # print(f'last_point:{last_point}, point:{point}')
      if utils.check_distance_exceeds(last_point, point, 0.1):
        sample_points.append(point)
        last_point = point
    
    # 确保最后一个点添加
    if points[-1] != sample_points[-1]:
      sample_points.append(points[-1])
    return sample_points

class LaneData:
  # 当 points 为 None 时, 是 junction
  def __init__(self, name, points=None):
    self.name = name
    self.sample_distance = 0.1
    self.origin_points = points
    self.predecessor_name = None
    self.successor_name = None
    self.left_width = None
    self.right_width = None
    self.width = None
    self.start_point = None
    self.end_point = None

    self.sample_points = []
    #            0                                                 end(len)
    #            |                                                  |
    #  lane      [                                                  ]
    #                     |                                 |
    #  junction           [                                 ]
    #  junction_idx   start_idx                         end_idx
    # junction start_idx, end_idx, len
    self.is_connected_junction = False
    self.is_junction_lane = False
    self.junction_idx = []
    self.junction_points = []
    self.junction_start_center_point = None
    self.junction_end_center_point = None
    print(f'LaneDat:[{self.name}], junction_points:{self.junction_points}, type:{type(self.junction_points)}')

    if points is not None and len(points) != 0:
      self.sample_point(points)
      # self.start_point = self.origin_points[0]
      # self.end_point = self.origin_points[-1]
      # 虽然采样点的起终点就是origin_points的起终点
      self.start_point = self.sample_points[0]
      self.end_point = self.sample_points[-1]

    if name == 'guai1':
      self.trim_curve_by_point([426969.394,3379763.006], 'before')
      # self.modify_and_smooth_curve([426969.293,3379763.779], 'start')

    if name == 'wai1':
      self.trim_curve_by_point([426951.391,3379722.45], 'before')
      self.trim_curve_by_point([426967.201,3379759.273], 'after')
      # self.modify_and_smooth_curve([426969.293,3379763.779], 'start')

    if name == 'wai2':
      self.trim_curve_by_point([426950.059,3379719.43], 'after')
      # self.modify_and_smooth_curve([426969.293,3379763.779], 'start')

  def init(self, topomap_csv, index):
    self.predecessor_name = topomap_csv.predecessors[index]
    self.successor_name = topomap_csv.successors[index]
    self.left_width = topomap_csv.left_widths[index]
    self.right_width = topomap_csv.right_widths[index]
    self.width = self.left_width + self.right_width
    # print(f'self.width:{self.width}')

  def sample_point(self, points):
    last_point = points[0]
    self.sample_points.append(points[0])

    for point in points:
      if utils.check_distance_exceeds(last_point, point, self.sample_distance):
        self.sample_points.append(point)
        last_point = point
    
    # 确保最后一个点添加
    if points[-1] != self.sample_points[-1]:
      self.sample_points.append(points[-1])

    # 初始化 junction points
    self.junction_idx = [0, len(self.sample_points) - 1, len(self.sample_points) - 1]
    # 最终用的都是这个 junction points, 所有每条车道最后统一更新
    # self.junction_points = self.sample_points

  def find_closest_point(self, ref_point):
    closest_idx = None
    min_dist = float('inf')

    for idx, point in enumerate(self.sample_points):
      dist = utils.distance(point, ref_point)
      if dist < min_dist:
        closest_idx = idx
        min_dist = dist
    
    return closest_idx


  def trim_curve_by_point(self, ref_point, mode='after'):
    if not self.sample_points:
      raise ValueError(f'sample_points is empty')
    
    closest_idx = self.find_closest_point(ref_point)

    if mode == 'after':
      self.sample_points = self.sample_points[:closest_idx]
    elif mode == 'before':
      self.sample_points = self.sample_points[closest_idx+1:]
    else:
      raise ValueError(f'The invalid truncation mode:{mode}, please use [before] or [after].')

    self.junction_idx = [0, len(self.sample_points) - 1, len(self.sample_points) - 1]


  def compute_parameterization(self, points):
    distances = np.zeros(len(points))

    for i in range(1, len(points)):
      distances[i] = distances[i-1] + np.linalg.norm(points[i] - points[i-1])
    
    return distances / distances[-1]


  def modify_and_smooth_curve(self, new_point, modify='end'):
    points = np.array(self.sample_points)
    # t = self.compute_parameterization(points)
    x = points[:, 0]
    y = points[:, 1]

    if modify == 'start':
      x[0], y[0] = new_point
    elif modify == 'end':
      x[-1], y[-1] = new_point
    else:
      raise ValueError(f'Invalid modification mode, use [start] or [end]')

    # cs = CubicSpline(x, y)
    # y_new = cs(x)

    # Bezier 曲线插值
    t = np.linspace(0, 1, len(x))
    bezier_x = interpolate.splrep(t, x, k=3)  # k=3 表示三次贝塞尔曲线
    bezier_y = interpolate.splrep(t, y, k=3)

    t_new = np.linspace(0, 1, 400)
    x_new = interpolate.splev(t_new, bezier_x)
    y_new = interpolate.splev(t_new, bezier_y)

    self.sample_points = list(zip(x_new, y_new))

  def get_min_bounding_circle(self, points):
    # lambda p: p[0] 表示取每个点的第一个元素，即 x 坐标。
    # [0] 表示取出这个点的 x 坐标。

    min_x = min(points, key=lambda p: p[0])[0]
    max_x = max(points, key=lambda p: p[0])[0]
    min_y = min(points, key=lambda p: p[1])[1]
    max_y = max(points, key=lambda p: p[1])[1]

    return [(min_x + max_x) / 2, (min_y + max_y) / 2]


  def init_junction_lane(self, is_junction_lane, from_points, to_points, width):
    self.is_junction_lane = is_junction_lane
    self.junction_start_center_point = self.get_min_bounding_circle(from_points)
    self.junction_end_center_point = self.get_min_bounding_circle(to_points)
    self.width = width

  def debug(self, ax):
    # x = [point[0] for point in self.origin_points]
    # y = [point[1] for point in self.origin_points]
    # ax.plot(x, y, label=f'{self.name} origin data', marker='o')
    
    # print(f'lane[{self.name}], sample_points_len[{len(self.sample_points)}]')
    # if '_' in self.name:
    #   print(f'')
    # x = [point[0] for point in self.sample_points]
    # y = [point[1] for point in self.sample_points]
    # ax.plot(x, y, label=f'{self.name} sample data', marker='^', markersize=12)
    # print(f'LaneData:[{self.name}], junction_points:{self.junction_points}, type:{type(self.junction_points)}')
    # print(f'junction:point,{self.junction_points}')
    x = [point[0] for point in self.junction_points]
    y = [point[1] for point in self.junction_points]
    ax.plot(x, y, label=f'{self.name} junction data', marker='^', markersize=12)

    # if self.name == 'wai1_guai1':
      # return

    if self.junction_end_center_point is not None:
      plt.annotate(f'junction[{self.name}]end_center_point[{self.junction_end_center_point[0], self.junction_end_center_point[1]}]', self.junction_end_center_point, xytext=(-30, -30), textcoords='offset points', ha='right', va='bottom', bbox=dict(boxstyle='round, pad=0.5', fc='blue', alpha=0.5), arrowprops=dict(arrowstyle='->', connectionstyle='arc3, rad=0'))
      plt.annotate(f'junction[{self.name}]start_center_point[{self.junction_end_center_point[0], self.junction_end_center_point[1]}]', self.junction_start_center_point, xytext=(30, 30), textcoords='offset points', ha='right', va='bottom', bbox=dict(boxstyle='round, pad=0.5', fc='green', alpha=0.5), arrowprops=dict(arrowstyle='->', connectionstyle='arc3, rad=0'))

    if self.is_junction_lane:
      # print(type(self.is_junction_lane))  # 输出: <class 'list'>
      print(f'LaneData debug: lane:[{self.name}] is junction lane')


class Junction:
  def __init__(self):
    pass

class LaneDataManager:
  def __init__(self, topomap_csv):
    self.lanes = {}  # 使用字典存储 LaneData 对象
    self.junctions_info = {} # 键是车道名字_start/end 值是这个车道端点对应的 junction 有哪些
    self.junction_threshold = 20.0
    self.junction_lane_threshold = 10.0
    self.topomap_csv = topomap_csv


  def add_lane(self, lane_data):
    # print(f'add lane name[{lane_data.name}]')
    self.lanes[lane_data.name] = lane_data  # 使用 name 作为键


  def get_lane(self, name):
    return self.lanes.get(name)  # 根据 name 获取 LaneData 对象


  def debug_all(self, ax):
    for lane in self.lanes.values():
        lane.debug(ax)  # 调用每个 LaneData 对象的 debug 方法


  ## 当前函数所做的工作
  ### 1. 调用车道的 init 函数来初始化所有车道的自身信息:即当前车道的前驱后继,车道宽度等基本属性
  ### 2. 如果车道是 junction 车道,则需要初始化 junction 车道, 标明当前车道是 junction 车道,
  ### 3.
  def init(self, topomap_csv):
    for index, name in enumerate(topomap_csv.names):
      self.lanes[name].init(topomap_csv, index)

      ##### 一个重要的观点, self.junctions_info 只会保存那些不是junction的车道信息
      ##### 也就是说,它保存的是 junction 车道的组成部分, 所以这个是应该在 if 里面的
      ##### start end 分别存放的是这个车道的首尾点与哪些车道的终点和起点相连
      # print(f'遍历的name:{name}')
      if '_' not in name:
        self.junctions_info[name] = {}
        # 这个是包含它自身
        ## end 存放的是这个车道的后面有哪些车道
        self.junctions_info[name]['start'] = [name]
        self.junctions_info[name]['end'] = [name]
        #### 这个 pre 和 suc 都是相对于其它一个车道而言的,
        #### 其它一个车道的 pre 是这个车道,其实就是这个车道后面的一个
        #### 其它一个车道的 suc 是这个车道,其实就是这个车道在别的车道后面,就是这个车道的起点
        for idx, pre_name in enumerate(topomap_csv.predecessors):
          if pre_name == name and '_' not in topomap_csv.names[idx]:
            self.junctions_info[name]['end'].append(topomap_csv.names[idx])

        ## start 存放的是这个车道的前面有哪些车道
        for idx, suc_name in enumerate(topomap_csv.successors):
          if suc_name == name and '_' not in topomap_csv.names[idx]:
            self.junctions_info[name]['start'].append(topomap_csv.names[idx])
        print(f'juncitons_info[{name}]:{self.junctions_info[name]}')
        continue

      # print(f'name[{name}] is junction lane')
      name_from, name_to =  name.split("_")
      """
      关于 junction 自己一直有一个错误的想法, 一直将它作为一个 lane 车道, 会认为一个 junction 会有两个端点,
      但更好的方式是不将 junction 作为一个车道来处理, 确定一个 junction 只需要确定 junction 的中点就可以了,
      因为,它的端点一定在 name_from 和 name_to 上, 只需要在 name_from 和 name_to 上根据距离去寻找就可以了,
      并且, 从结果来看, junction 的两个端点的中点是相同的, 虽然可能和自己的 junction 比较简单有关, 但是复杂的 junction 应该也是同理的
      """
      if not utils.check_distance_exceeds(self.lanes[name_from].end_point, 
                                          self.lanes[name_to].start_point,
                                          self.junction_threshold):
        from_points = []

        ##tips 在这里自己犯了很多的错,首先自己没有意识到,这里的起终点转换
        ## 对于 junction 的起点来说,那么对于junction 的 to 车道来说,就是它的终点
        # for fname in self.junctions_info[name_from]['start']:
        for fname in self.junctions_info[name_from]['end']:
          # print(f'junction[{name}], from:[{fname}]')
          # 如果是自身的话应该是包含自身的起点而不是自身的终点
          ## tips 其次自己忽略了,当junction中自身的起点对应的是其它车道的终点问题,导致在计算坐标时,都计算成了起点或终点
          if fname == name_from:
            print(f'junction[{name}], from:[{fname}], 获取的原始点的最后一个点')
            # 应该是采样点
            # from_points.append(self.lanes[fname].origin_points[-1])
            from_points.append(self.lanes[fname].sample_points[-1])
          else:
            print(f'junction[{name}], from:[{fname}], 获取的原始点的第一个点')
            # from_points.append(self.lanes[fname].origin_points[0])
            from_points.append(self.lanes[fname].sample_points[0])

        to_points = []
        ## 对于 junction 的终点来说,那么对于junction 的 to 车道来说,就是它的起点
        # for fname in self.junctions_info[name_to]['end']:
        for fname in self.junctions_info[name_to]['start']:
          if fname == name_to:
            print(f'junction[{name}], to:[{fname}], 获取的原始点的第一个点')
            # to_points.append(self.lanes[fname].origin_points[0])
            to_points.append(self.lanes[fname].sample_points[0])
          else:
            print(f'junction[{name}], to:[{fname}], 获取的原始点的最后一个点')
            # to_points.append(self.lanes[fname].origin_points[-1])
            to_points.append(self.lanes[fname].sample_points[-1])
          # to_points.append(self.lanes[fname].origin_points[0])

        # from_widths = [self.lanes[lane_name].width for lane_name in self.junctions_info[name_from]['start']]
        # print(f'from_widths:{from_widths}') # from_widths:['4.04.0']
        from_widths = []
        for flane_name in self.junctions_info[name_from]['start']:
          from_widths.append(self.lanes[flane_name].width)
        # from_widths = [self.lanes[lane_name].width ]
        # print(f'from_widths:{from_widths}') # from_widths:['4.04.0']
        width = 0.0
        for width_i in from_widths:
          if width < width_i:
            width = width_i
        
        # 这里有个问题时,经过上面的遍历, name 已经被赋予新的值了
        # print(f'调用lane[{name}] 的 init_junction_lane')
        self.lanes[name].init_junction_lane(True, 
                                            from_points, 
                                            to_points,
                                            width)

        self.lanes[name_from].is_connected_junction = True
        self.lanes[name_to].is_connected_junction = True
      else:
        raise ValueError(f'junction:[{name}] distance start to end too large:{utils.distance(self.lanes[name_from].end_point, self.lanes[name_to].start_point)}, threshold:{self.junction_threshold}')
  # def init(self, topomap_csv):
  #     # 初始化 predecessors 和 successors 索引
  #   pre_dict = {name: [] for name in topomap_csv.names}
  #   print(f'pre_dict:{pre_dict}')
  #   suc_dict = {name: [] for name in topomap_csv.names}
  #   print(f'suc_dict:{suc_dict}')

  #   # 构建 predecessors 和 successors 的字典
  #   for idx, pre_name in enumerate(topomap_csv.predecessors):
  #     if pre_name in pre_dict:
  #       pre_dict[pre_name].append(topomap_csv.predecessors[idx])

  #   for idx, suc_name in enumerate(topomap_csv.successors):
  #     if suc_name in suc_dict:
  #       suc_dict[suc_name].append(topomap_csv.successors[idx])
  #   print(f'pre_dict:{pre_dict}')
  #   print(f'suc_dict:{suc_dict}')

  #   # 单次遍历 names
  #   for index, name in enumerate(topomap_csv.names):
  #     self.lanes[name].init(topomap_csv, index)

  #     if '_' not in name:
  #       self.junctions_info[name] = {}
  #       self.junctions_info[name]['end'] = [name] + pre_dict.get(name, [])
  #       self.junctions_info[name]['start'] = [name] + suc_dict.get(name, [])
  #       continue

  #     # 处理带有 '_' 的名字
  #     name_from, name_to = name.split("_")
  #     if not utils.check_distance_exceeds(self.lanes[name_from].end_point, 
  #                                         self.lanes[name_to].start_point,
  #                                         self.junction_threshold):
  #       from_points = [self.lanes[lane_name].origin_points[-1] 
  #                     for lane_name in self.junctions_info[name_from]['start']]
  #       to_points = [self.lanes[lane_name].origin_points[0] 
  #                   for lane_name in self.junctions_info[name_to]['end']]

  #       from_widths = [self.lanes[lane_name].width for lane_name in self.junctions_info[name_from]['start']]
  #       width = 0.0
  #       for width_i in from_widths:
  #         if width < width_i:
  #           width = width_i

  #       self.lanes[name].init_junction_lane(True, from_points, to_points, width)

  #       self.lanes[name_from].is_connected_junction = True
  #       self.lanes[name_to].is_connected_junction = True
  #     else:
  #       raise ValueError(f'junction:[{name}] distance start to end too large:'
  #                       f'{utils.distance(self.lanes[name_from].end_point, self.lanes[name_to].start_point)}, '
  #                       f'threshold:{self.junction_threshold}')


  # 现在需要做的是:修改连接junction车道的点,以及生成 junction 车道的点
  def generate_junction_lane_idx(self):
    for index, name in enumerate(self.topomap_csv.names):
      if '_' not in name:
        continue

      junction_name = name.split("_")
      name_from = junction_name[0]
      name_to = junction_name[1]
      junction_center_point = self.lanes[name].junction_end_center_point

      cut_dis = self.lanes[name].width * 0.8
      from_dis = utils.distance(self.lanes[name_from].end_point, self.lanes[name].junction_end_center_point)

      print(f'cut_dis[{cut_dis}], from_dis[{from_dis}], cut_dis - from_dis[{cut_dis - from_dis}]')

      to_dis = utils.distance(self.lanes[name_to].start_point, self.lanes[name].junction_end_center_point)


      if utils.check_distance_exceeds(self.lanes[name_from].end_point,
            junction_center_point, cut_dis) or utils.check_distance_exceeds(self.lanes[name_to].start_point, 
            junction_center_point, cut_dis):
        print(f'from:{name_from} to:{name_to} dis to junction_center_point is too large')
        exit(1)


      ## 针对于 from, 表示当前 name_from 所余留的距离还不够使用, 需要向前遍历
      print(f'self.lanes[{name_from}].sample_points:[{len(self.lanes[name_from].sample_points)}], idx: {self.lanes[name_from].junction_idx[1]}')
      if not utils.check_distance_exceeds(self.lanes[name_from].sample_points[self.lanes[name_from].junction_idx[1]], self.lanes[name_from].end_point, cut_dis - from_dis):
        # frange = self.lanes[name_from].junction_idx[0], self.lanes[name_from].junction_idx[1][:,-1]
        print(f'进入到 {name_from}  lane, 范围[{self.lanes[name_from].junction_idx[0], self.lanes[name_from].junction_idx[1]}]')
        # frange = range(self.lanes[name_from].junction_idx[0], self.lanes[name_from].junction_idx[1])
        # frange = frange[-1]
        # for idx in range(self.lanes[name_from].junction_idx[1], self.lanes[name_from].junction_idx[0]):
        # for idx in frange:
        for idx in range(self.lanes[name_from].junction_idx[0], self.lanes[name_from].junction_idx[1])[::-1]:
          if utils.check_distance_exceeds(self.lanes[name_from].sample_points[idx], self.lanes[name_from].end_point, cut_dis - from_dis):
            self.lanes[name_from].junction_idx[1] = idx
            print(f'============================================================')
            break
          # print(f'搜索 name_from: {name_from}idx:[{idx}]')

      ## 针对于 to, 表示当前 name_to 所余留的距离还不够使用, 需要向后遍历
      if not utils.check_distance_exceeds(self.lanes[name_to].sample_points[self.lanes[name_to].junction_idx[0]], self.lanes[name_to].start_point, cut_dis - from_dis):
        print(f'进入到 {name_to}  lane, 范围[{self.lanes[name_to].junction_idx[0], self.lanes[name_to].junction_idx[1]}]')
        # for idx in range(self.lanes[name_to].junction_idx[0], self.lanes[name_to].junction_idx[1])[:]:
        for idx in range(self.lanes[name_to].junction_idx[0], self.lanes[name_to].junction_idx[1])[:]:
          if utils.check_distance_exceeds(self.lanes[name_to].sample_points[idx], self.lanes[name_to].start_point, cut_dis - from_dis):
            self.lanes[name_to].junction_idx[0] = idx
            print(f'============================================================')
            break
          # print(f'搜索 name_to:{name_to}idx:[{idx}], dis:[{utils.distance(self.lanes[name_to].sample_points[idx], self.lanes[name_to].start_point)}, cut_dis-from_dis[{cut_dis - from_dis}]]')

      print(f'在 generate_junction_lane_idx 修改之后')
      print(f'[{name}] name_from: [{name_from}] idx: {self.lanes[name_from].junction_idx[0], self.lanes[name_from].junction_idx[1], self.lanes[name_from].junction_idx[2]}')
      print(f'[{name}] name_to: [{name_to}] idx: {self.lanes[name_to].junction_idx[0], self.lanes[name_to].junction_idx[1], self.lanes[name_to].junction_idx[2]}')


  # 在进行填充时会进一步考虑一个问题,会使junction的长度保证在一个范围之内:self.width * 1.3 * 1.5
  # 这里还是修改 junction 的 from 和 to 车道的范围, 然后将他们缩小一些
  # 目前还没看到这些的必要行,先不做考虑
  def update_junction(self):
    pass

    for index, name in enumerate(self.topomap_csv.names):
      if '_' not in name:
        continue

      junction_name = name[index].split("_")
      name_from = junction_name[0]
      name_to = junction_name[1]

      max_junction_dis = self.lanes[name].width * 1.3 * 1.5


  def fill_junction_lane_points(self):
    for index, name in enumerate(self.topomap_csv.names):
      if '_' in name:
        print(f'name:{name}')
        junction_name = name.split("_")
        print(f'junction_name:{junction_name}')
        name_from = junction_name[0]
        name_to = junction_name[1]

        print(f'[{name}] name_from[{name_from}] idx: {self.lanes[name_from].junction_idx[1], self.lanes[name_from].junction_idx[2]}')
        for idx in range(self.lanes[name_from].junction_idx[1], self.lanes[name_from].junction_idx[2]):
          self.lanes[name].junction_points.append(self.lanes[name_from].sample_points[idx])
        
        print(f'[{name}] name_to[{name_to}] idx: {0, self.lanes[name_to].junction_idx[0]}')
        for idx in range(0, self.lanes[name_to].junction_idx[0]):
          self.lanes[name].junction_points.append(self.lanes[name_to].sample_points[idx])
      else:
        print(f'非 junction lane:{name}, idx[{self.lanes[name].junction_idx[0], self.lanes[name].junction_idx[1]}]')
        # print(f'******************************************************************************')
        for idx in range(self.lanes[name].junction_idx[0], self.lanes[name].junction_idx[1]):
          # print(f'非 junction lane:{name} 填充了点{self.lanes[name].sample_points[idx]}')
          self.lanes[name].junction_points.append(self.lanes[name].sample_points[idx])

      print(f'[{name}] junction_points len: {len(self.lanes[name].junction_points)}')

      inter_points = self.interpolate_points(self.lanes[name].junction_points)
      # print(f'inter_points:[{inter_points}]')
      sample_points = utils.sample_point(inter_points)
      # print(f'sample_points:[{sample_points}]')
      self.lanes[name].junction_points = sample_points


  def interpolate_points(self, points):
    interpolate_dis = 1.0
    interpolate_num = 100
    new_points = []
    for i in range(0, len(points) - 1):
      new_points.append(points[i])
      if utils.check_distance_exceeds(points[i], points[i+1], interpolate_dis):
        xi = [points[i][0], points[i+1][0]]
        yi = [points[i][1], points[i+1][1]]
        new_xi = np.linspace(xi[0], xi[-1], num=interpolate_num)
        f = interpolate.interp1d(xi, yi, kind='linear')
        new_yi=f(new_xi)
        for j in range(len(new_xi)):
          new_points.append([new_xi[j], new_yi[j]])
    print(f'len: {len(points)}')
    new_points.append(points[-1])
    return new_points
  # def gen_junction_point

class DataLoad:
  def __init__(self):
    pass


  @staticmethod
  def load_topomap_csv(filename, filter_name):
    names = []
    predecessors = []
    successors = []
    left_widths = []
    right_widths = []
    with open(filename, 'r') as file:
      line_data = csv.reader(file)
      for name, predecessor, successor, leftwidth, rightwidth in line_data:
        if name == filter_name:
          continue
        else:
          names.append(name)
          predecessors.append(predecessor)
          successors.append(successor)
          left_widths.append(float(leftwidth))
          right_widths.append(float(rightwidth))

    return TopoMapCsv(names, predecessors, successors, left_widths, right_widths)


  @staticmethod
  def load_lane_localization_point(lane_name):
    filepath = '/apollo/modules/tools/hd_map/beacon_txt/' + lane_name + '.txt'
    with open(filepath, 'r') as file:
      points = []
      for line in file:
        parts = line.strip().split(',')
        if len(parts) != 2:
          continue
        xi, yi = map(float, parts)
        points.append([xi, yi])

    return LaneData(lane_name, points=points)


  @staticmethod
  def load_all_lane_localization_point(topomap_csv):

    lane_data_manager = LaneDataManager(topomap_csv)
    for lane_name in topomap_csv.names:
      # 这里是从 localization 中加载数据, 但 junction 中并没有 localization 数据
      if '_' in lane_name:
        lane_data = LaneData(lane_name)
        lane_data_manager.add_lane(lane_data)
        continue

      lane_data = DataLoad.load_lane_localization_point(lane_name)

      lane_data_manager.add_lane(lane_data)

    return lane_data_manager 



def main():
  fig, ax = plt.subplots()

  topomap_csv = DataLoad.load_topomap_csv('/apollo/modules/tools/hd_map/conf/TopoMap.csv', 'Name')
  # topomap_csv.debug()

  lane_data_manager = DataLoad.load_all_lane_localization_point(topomap_csv)
  # print(all_lane)
  # {'wai2': <__main__.LaneData object at 0x714590ab3908>, 'wai1': <__main__.LaneData object at 0x714590ab3940>, 'guai1': <__main__.LaneData object at 0x714590ab3978>}
  # print(all_lane)
  # 这样遍历是里面有多少个元素就会有多少个元素
  # for key, value in all_lane:
  # for key, value in all_lane.items():
  #   value.debug(ax)
  lane_data_manager.init(topomap_csv)
  lane_data_manager.generate_junction_lane_idx()
  lane_data_manager.fill_junction_lane_points()


  lane_data_manager.debug_all(ax)

  plt.legend()
  plt.show()



if __name__ == '__main__':
  main()

