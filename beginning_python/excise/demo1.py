import logging
import argparse
import re

import  matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(levelname)s [%(filename)s:%(lineno)d]: %(message)s')

class PolygonDataLoader:
  @staticmethod
  def load_data_from_file(filename:str):
    try:
      with open(filename, 'r+') as file:
        polygons = []
        current_polygon = []
        for line in file:
          line = line.strip()
          if re.match('^-+$', line):
            if current_polygon:
              polygons.append(current_polygon)
              current_polygon = []
          else:
            parts = line.split(',')
            if len(parts) != 2:
              logging.error(f'invalid line format:{line}')
              continue
            try:
              x, y = map(float, parts)
              current_polygon.append((x, y))
            except ValueError as e:
              logging.error(f'invalid coordinate values:{line}')
              continue

        ## 这里自己犯了一个错
        ## 犯错不是因为自己将这个代码的缩进写错了,而是自己花了很多时间都没有注意到这里
        ## chatgpt 已经检查出了这些代码存在问题,但是自己看的时候没有太过在意
        ## 也就是说自己在看东西时,专注力,注意力都没有很多
        ## 遇到问题时,也没有思路去理性分析,自己只分析对了一半
        ## 自己做对的地方:1. 查看 update 是否被正常调用,
        ##              2. update 被调用之后查看数据是否正常
        ##              3. 发现数据比实际会多很多
        ## 自己做的不好的地方:
        ##              1. 在发现获得的数据比实际的数据多很多时,自己并没有想到去查看数据是在哪里添加的,是因为什么而被多添加的
        ##              2. 后面想到看数据是在哪里被添加时,也没有去仔细检查每一个添加的条件,或者说检查是否有缩进错误
        ##              3. 在看 chatgpt 的提示,完全没有看进去,注意力非常不够
        if current_polygon:
          polygons.append(current_polygon)

        return polygons
    except Exception as e:
      logging.error(f'failed load data form {filename}:{e}')

class PolygonAnimator:
  def __init__(self, polygons) -> None:
    self.polygons = polygons
    self.fig, self.ax = plt.subplots()
    self.polygon_patch = None
  
  def initialize_patch(self):
    cmap = plt.get_cmap('tab10')
    colors = cmap.colors
    if not self.polygons:
      raise ValueError('No polygons to initialize.')
    
    self.polygon_patch = Polygon(self.polygons[0], closed=True, edgecolor=colors[1], linewidth=2, facecolor=colors[8])
    self.ax.add_patch(self.polygon_patch)
    self.ax.set_title("show polygon")

    x_min = float('inf')
    x_max = float('-inf')
    y_min = float('inf')
    y_max = float('-inf')

    for polygon in self.polygons:
      for x, y in polygon:
        x_min = min(x_min, x)
        x_max = max(x_max, x)
        y_min = min(y_min, y)
        y_max = max(y_max, y)

    self.ax.set_xlim(x_min - 1, x_max + 1)
    self.ax.set_ylim(y_min - 1, y_max + 1)
    self.ax.set_aspect('equal')

    return self.polygon_patch

  def update(self, frame):
    if frame >= len(self.polygons):
      ## 本来这里是想抛出异常的,但是gpt建议不抛出异常
      # raise ValueError(f'') 
      return None
    self.polygon_patch.set_xy(self.polygons[frame])
    return self.polygon_patch

  def animate(self):
    
    anim = FuncAnimation(self.fig, self.update, frames=len(self.polygons), repeat=True, interval=200, init_func=self.initialize_patch)

    try:
      plt.show()
    except Exception as e:
      logging.error(f"Animation interrupted: {e}")

def main():
  parser = argparse.ArgumentParser(description="Polygon data load filename")

  parser.add_argument("filename", nargs='?', default='polygons.txt', help='input the polygon data file name')

  args = parser.parse_args()
  logging.info(f'load polygon data from {args.filename}')

  polygons = PolygonDataLoader.load_data_from_file(args.filename)

  if not polygons:
    logging.error("No polygons loaded. Exiting.")
    return  # 或者使用 sys.exit(1)

  polygon_anim = PolygonAnimator(polygons)
  polygon_anim.animate()

if __name__ == '__main__':
  main()
  