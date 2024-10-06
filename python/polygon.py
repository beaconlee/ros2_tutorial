import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
points = [
  [1, 1],
  [2, 1],
  [3, 1.5],
  [4, 2],
  [2, 4],
  [1, 2]
]

fig, ax = plt.subplots()

# closed 设置当前多边形是否是封闭的,如果不设置为True且给的点不是封闭的,那么edge就不是一个闭合的
pgon = Polygon(points, closed=False, edgecolor='b', linewidth=10, facecolor='w')
ax.add_patch(pgon)

# ax.set_xlim(0, 5)
# ax.set_ylim(0, 5)
ax.autoscale()


plt.show()
