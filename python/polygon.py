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
# import sys
import argparse
import logging

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


# filename = "polygons.txt"

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
    
    # def read_polygons_from_file(self, filename):
    #     try:
    #       with open(filename, "r") as file:
    #           current_polygon = []
    #           for line in file:
    #               line = line.strip()
    #               if re.match("^-+$", line):
    #                   self.polygons.append(current_polygon) 
    #                   current_polygon = []
    #               else:
    #                   try:
    #                       x, y = map(float, line.split(","))
    #                       current_polygon.append((x, y))
    #                   except ValueError:
    #                       raise RuntimeError(f"Invalid polygon data at line: {line}")

    #           if current_polygon:
    #               self.polygons.append(current_polygon)

    #       return self.polygons
    #     except FileNotFoundError:
    #         raise RuntimeError(f"File '{filename}' not found.")
    #     except ValueError as e:
    #         raise RuntimeError(f"Data format error in file '{filename}': {str(e)}")
    #     except Exception as e:
    #         raise RuntimeError(f"An unexpected error occurred: {str(e)}")

    def update(self, frame):
        if not self.polygons or frame >= len(self.polygons):
          return None
        self.polygon_patch.set_xy(self.polygons[frame])
        return self.polygon_patch

    def initialize_patch(self):
        # print("initialize_patch called")

        cmap = plt.get_cmap("tab10")
        colors = cmap.colors

        # 检查多边形列表是否为空
        if not self.polygons:
            raise ValueError("No polygons to initialize.")
        # print("开始添加")
        # print(f"长度:{len(self.polygons)}")
        # print(self.polygons)
        self.polygon_patch = Polygon(self.polygons[0], closed=True, edgecolor=colors[0],
                                     linewidth=2, facecolor=colors[1])
        self.ax.add_patch(self.polygon_patch)
        # print("添加完成")

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
        # print("init 函数执行完毕")

    def animate(self):
        logging.info("Starting animation")
        if not self.polygons:
            try:
                self.polygons = PolygonDataLoader.load_from_file(self.filename)
            except Exception as e:
                logging.error(f"Error: {e}")

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
    # 创建一个 ArgumentParser 对象,用来解析参数
    # description 参数用于向用户提供该程序的简单描述。这个描述会在用户请求帮助信息（如使用 python script.py --help）时显示。
    parser = argparse.ArgumentParser(description='Polygon Animator')
    # parser.add_argument() 添加了一个参数，名为 filename。
    # nargs='?' 表示该参数是可选的。如果用户未提供参数，程序将使用默认值。
    # default='polygons.txt' 指定了一个默认值 polygons.txt，
    # 这意味着如果用户没有提供文件名，程序将默认使用 polygons.txt 作为输入文件。
    # help='Input file containing polygon data' 为该参数提供了帮助信息，用户在请求帮助时会看到这个描述。
    """
      nargs: number of arguments
      nargs 选项：
      nargs='?'：表示这个参数是可选的，最多接收一个值。如果未提供，则使用默认值（如果设置了 default）。

      例子：python script.py 或 python script.py filename.txt
      nargs='*'：表示可以接受零个或多个参数，结果会作为一个列表。如果未提供参数，返回一个空列表。

      例子：python script.py 或 python script.py file1.txt file2.txt
      nargs='+'：表示必须至少接收一个参数，结果会作为一个列表。如果未提供参数，会报错。

      例子：python script.py file1.txt file2.txt
      nargs=1：表示必须接收一个参数，结果会作为列表返回。

      例子：python script.py file1.txt
      nargs=2：表示必须接收两个参数，结果会作为列表返回。

      例子：python script.py file1.txt file2.txt
    """
    parser.add_argument('filename', nargs='?', default='polygons.txt', help='Input file containing polygon data')
    """
    参数分为两种:
        位置参数：直接在命令行中按顺序传递。
        可选参数：使用 --parameter_name value 的形式传递。
    """
    parser.add_argument('--speed', type=int, default=200, help='Animation speed in milliseconds')

    args = parser.parse_args()

    logging.info(f"Beginning with {args.filename}")

    polygons = PolygonDataLoader.load_from_file(args.filename)

    pa = PolygonAnimator(polygons)
    pa.animate()

if __name__ == "__main__":
    main()