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
import argparse
import logging


filename = "polygons.txt"

class PolygonDataLoader:
    @staticmethod
    def load_from_file(filename):
        polygons = []
        try:
            with open(filename, "r") as file:
                current_polygon = []
                for line in file:
                    line = line.strip()
                    if re.match("^-+$", line):
                        if current_polygon:
                            polygons.append(current_polygon)
                        current_polygon = []
                    else:
                        parts = line.split(",")
                        if len(parts) != 2:
                            logging.warning(f"Invalid line format: {line}")
                            continue
                        try:
                            x, y = map(float, parts)
                            current_polygon.append((x, y))
                        except ValueError:
                            logging.warning(f"Invalid coordinate values: {line}")
                            continue
                if current_polygon:
                    polygons.append(current_polygon)
            return polygons
        except Exception as e:
            logging.error(f"Failed to load polygons from file '{filename}': {e}")
            raise



class PolygonAnimator:
    def __init__(self, polygons) -> None:
  
        self.fig, self.ax = plt.subplots()

        self.polygons = polygons
        # self.polygon_patch = Polygon(self.polygons[0], closed=True, edgecolor='b', 
        #                             linewidth=4, facecolor='g')
        self.polygon_patch = None
        # self.ax.add_patch(self.polygon_patch)
        self.ax.set_title("polygon show", fontsize=30)
    
    def read_polygons_from_file(self, filename):
        try:
          with open(filename, "r") as file:
              current_polygon = []
              for line in file:
                  line = line.strip()
                  if re.match("^-+$", line):
                      self.polygons.append(current_polygon) 
                      current_polygon = []
                  else:
                      try:
                          x, y = map(float, line.split(","))
                          current_polygon.append((x, y))
                      except ValueError:
                          raise RuntimeError(f"Invalid polygon data at line: {line}")

              if current_polygon:
                  self.polygons.append(current_polygon)

          return self.polygons
        except FileNotFoundError:
            raise RuntimeError(f"File '{filename}' not found.")
        except ValueError as e:
            raise RuntimeError(f"Data format error in file '{filename}': {str(e)}")
        except Exception as e:
            raise RuntimeError(f"An unexpected error occurred: {str(e)}")

    def update(self, frame):
        if not self.polygons or frame >= len(self.polygons):
          return None
        self.polygon_patch.set_xy(self.polygons[frame])
        return self.polygon_patch

    def initialize_patch(self):
        # 检查多边形列表是否为空
        if not self.polygons:
            raise ValueError("No polygons to initialize.")

        self.polygon_patch = Polygon(self.polygons[0], closed=True, edgecolor='b',
                                     linewidth=2, facecolor='g')
        self.ax.add_patch(self.polygon_patch)

        x_min = float('inf')
        x_max = float('-inf')
        y_min = float('inf')
        y_max = float('-inf')

        ## 不要这样写循环      
        # for i in range(len(self.polygons)):
        #     polygon = self.polygons[i]
        #     x_values = [point[0] for point in polygon]
        #     y_values = [point[1] for point in polygon]
        for polygon in self.polygons:
            
            ## 不要这样遍历
            # x_values = [point[0] for point in polygon]
            # y_values = [point[1] for point in polygon]
            
            # x_min = min(x_min, min(x_values))
            # x_max = max(x_max, max(x_values))
            # y_min = min(y_min, min(y_values))
            # y_max = max(y_max, max(y_values))
            for x, y in polygon:
                x_min = min(x_min, x)
                x_max = max(x_max, x)
                y_min = min(y_min, y)
                y_max = max(y_max, y)
        # 动态设置 x 和 y 的显示范围
        self.ax.set_xlim(x_min - 1, x_max + 1)
        self.ax.set_ylim(y_min - 1, y_max + 1)

        self.ax.set_aspect('equal') 

    def animate(self):
        logging.info("Starting animation")
        if not self.polygons:
            try:
                self.polygons = PolygonDataLoader.load_from_file(self.filename)
            except Exception as e:
                print(f"Error: {e}")
                return
        
        ## 直接使用 init_func 参数
        # self.initialize_patch()
        """
          记录一个报错:
          /workspace/matplotlib/lib/python3.10/site-packages/matplotlib/animation.py:872: UserWarning: Animation was deleted without rendering anything. This is most likely not intended. To prevent deletion, assign the Animation to a variable, e.g. anim, that exists until you output the Animation using plt.show() or anim.save().
          warnings.warn(

          用户警告: 动画在未渲染任何内容的情况下被删除。这很可能不是预期的行为。为了防止删除，请将动画分配给一个变量，例如 `anim`，并确保在使用 `plt.show()` 或 `anim.save()` 输出动画之前，该变量存在。
        """
        anim = FuncAnimation(self.fig, self.update, frames=len(self.polygons),
                            init_func=self.initialize_patch, interval=200, repeat=True)
        try:
            plt.show()
        except Exception as e:
            logging.error(f"Animation interrupted: {e}")


# if(len(sys.argv) == 2):
#     filename = sys.argv[1]
# elif (len(sys.argv) > 2):
#     print("Usage: python script.py <filename>")
#     sys.exit(1)

# print(f"beginning with {filename}")

# polygons = PolygonDataLoader.load_from_file(filename)
# pa = PolygonAnimator(polygons)
# pa.animate()



def main():
    parser = argparse.ArgumentParser(description='Polygon Animator')
    parser.add_argument('filename', nargs='?', default='polygons.txt', help='Input file containing polygon data')
    args = parser.parse_args()
    print(f"{args.filename}")
    logging.info(f"Beginning with {args.filename}")

    pa = PolygonAnimator(args.filename)
    pa.animate()

if __name__ == "__main__":
    main()