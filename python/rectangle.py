import matplotlib.pyplot as plt
# Patches are `.Artist`\s with a face color and an edge color.
import matplotlib.patches as patches

# 创建一个 figure 和子图 ax
fig, ax = plt.subplots()

# 创建一个矩形对象
# Rectangle((x, y), width, height)
rect = patches.Rectangle((0.2, 0.3), 0.4, 0.6, linewidth=2, edgecolor='r', facecolor='g')

# 将矩形添加到 ax 中
ax.add_patch(rect)

# 设置轴的范围
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)

# 显示图形
plt.show()
