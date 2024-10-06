import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
points = [
  [1, 1],
  [2, 1],
  [3, 1.5],
  [4, 2],
  [2, 10],
  [1, 2]
]

fig, ax = plt.subplots()

# closed 设置当前多边形是否是封闭的,如果不设置为True且给的点不是封闭的,那么edge就不是一个闭合的
pgon = Polygon(points, closed=False, edgecolor='b', linewidth=10, facecolor='w')
ax.add_patch(pgon)

# ax.set_xlim(0, 5)
# ax.set_ylim(0, 5)
ax.autoscale()

# 使用 matplotlib 进行绘图时,默认情况下，坐标轴的纵横比会根据数据自动调整，可能导致图形看起来失真。
# ax.set_aspect('equal') 用于设置坐标轴的纵横比，使得x轴和y轴的比例相等。
# 换句话说，1个单位的长度在x方向和y方向上将显示为相同的物理距离（无论坐标轴的刻度如何）。
ax.set_aspect('equal')

plt.show()
