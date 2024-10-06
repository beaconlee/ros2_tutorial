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

pgon = Polygon(points)
ax.add_patch(pgon)

plt.show()
