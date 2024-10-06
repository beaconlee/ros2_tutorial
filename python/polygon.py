# import matplotlib.pyplot as plt
# from matplotlib.patches import Polygon
# points = [
#   [1, 1],
#   [2, 1],
#   [3, 1.5],
#   [4, 2],
#   [2, 10],
#   [1, 2]
# ]

# fig, ax = plt.subplots()

# # closed 设置当前多边形是否是封闭的,如果不设置为True且给的点不是封闭的,那么edge就不是一个闭合的
# pgon = Polygon(points, closed=False, edgecolor='b', linewidth=10, facecolor='w')
# ax.add_patch(pgon)

# # ax.set_xlim(0, 5)
# # ax.set_ylim(0, 5)
# ax.autoscale()

# # 使用 matplotlib 进行绘图时,默认情况下，坐标轴的纵横比会根据数据自动调整，可能导致图形看起来失真。
# # ax.set_aspect('equal') 用于设置坐标轴的纵横比，使得x轴和y轴的比例相等。
# # 换句话说，1个单位的长度在x方向和y方向上将显示为相同的物理距离（无论坐标轴的刻度如何）。
# ax.set_aspect('equal')

# plt.show()

#######################################################################################
#######################################################################################
#######################################################################################
#######################################################################################
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation
import re
import sys


filename = "polygons.txt"

class PolygonAnimator:
    def __init__(self) -> None:
        self.fig, self.ax = plt.subplots()
        
        self.polygons = []
        # self.polygon_patch = Polygon(self.polygons[0], closed=True, edgecolor='b', 
        #                             linewidth=4, facecolor='g')
        self.polygon_patch = None
        self.ax.add_patch(self.polygon_patch)
        self.ax.set_title("polygon show", fontsize=30)
    
    def read_polygons_from_file(self, filename):
        with open(filename, "r") as file:
            current_polygon = []
            for line in file:
                line = line.strip()
                if re.match("^-+$"):
                    self.polygons.append(current_polygon) 
                    current_polygon = []
                else:
                    current_polygon.append(map(float, line.split(",")))

            if current_polygon:
                self.polygons.append(current_polygon)

        return self.polygons

    def update(self, frame):
        self.polygon_patch.set_xy(self.polygons[frame])
        return self.polygon_patch

    def animate(self):
        if not self.polygons:
            if not read_data_with_file(filename):
               print("error, polygons is empty")
               exit()
        FuncAnimation(self.fig, self.update, frames=len(self.polygons),
                      interval=200, repeat=True)
        plt.show()


if(len(sys.argv) == 2):
  filename = sys.argv[1]
elif (len(sys.argv) > 2):
  print("")

print(f"beginning with {filename}")

def read_data_with_file(filename:str):
  polygons = []
  with open("polygons.txt", "r") as file:
    current_poly = []
    for line in file:
      line = line.strip()
      if re.match(r"^-+$", line):
        if current_poly:
          polygons.append(current_poly)
          current_poly = []
      
        else:
          x, y = map(float, line.split(','))
          current_poly.append((x, y))
    
    if current_poly:
      polygons.append(current_poly)
  return polygons


def update(frame, polygon):
  polygon = polygons[frame]
  return []
