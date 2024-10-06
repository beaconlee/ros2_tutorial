"""
记录 matplotlib 中的颜色设置
"""
import matplotlib.pyplot as plt
import numpy as np
import logging

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
# linspace: linearly spaced 线性间隔
x = np.linspace(1, 10, 100)

y_data = [np.sin(x + item) for item in np.linspace(np.pi/2, 2*np.pi, 3)]

# y2 = [1, 2, 3, 4, 5, 6, 7]
colors = np.random.rand(len(y_data), 3)

fig, ax = plt.subplots()

for i, y in enumerate(y_data):
    logging.info(f"第{i}条线")
    # y2 = []
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # ax.plot(x, y, y2,label=f"numer{i}")

    # plot([x], y, [fmt], *, data=None, **kwargs)
    # plot([x], y, [fmt], [x2], y2, [fmt2], ..., **kwargs)
    # 如果没有传入 x,则默认以0为开始,1为补偿
    ax.plot(x, y, color=colors[i],label=f"lin1_{i}")

cam = plt.get_cmap("tab10")
colors2 = cam.colors

y_data2 = [np.sin(x + item) + 2 for item in np.linspace(np.pi/2, 2*np.pi, 3)]


for i, y in enumerate(y_data2):
    logging.info(f"第{i}条线")
    # y2 = []
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # y2.append(i)
    # 这里会画出两条线来
    # ax.plot(x, y, y2,label=f"numer{i}")

    # plot([x], y, [fmt], *, data=None, **kwargs)
    # plot([x], y, [fmt], [x2], y2, [fmt2], ..., **kwargs)
    # 如果没有传入 x,则默认以0为开始,1为补偿
    """
    fmt 参数

    fmt 参数是一个便捷的方式来定义基本的格式，如颜色、标记符号（如点、星、圆）和线条样式（如实线、虚线）。这是一个快捷字符串，可以很方便地定义图形样式。
    例如，'ro-' 表示红色（red）的圆点标记（o）和实线（-）。
    """
    y2 = np.ones(20) + 4
    # 在 matplotlib 中，如果你使用了组合格式字符串（如 "ro-"）来指定第一条线的样式，
    # 但又在同一个 plot() 函数调用中使用关键字参数（如 color=colors2[i]）为第二条线指定颜色，
    # 关键字参数会应用于整个 plot() 函数调用。
    # 因此，后面的 color=colors2[i] 参数将覆盖之前使用的组合格式字符串中的颜色部分。

    # 当你同时传递了 fmt（格式字符串，如 "ro-"）和关键字参数（如 color=colors2[i]），
    # matplotlib 将关键字参数视为更高优先级，因此它会覆盖由格式字符串设置的颜色部分。
    # ax.plot(x, y, "ro-", y2, color=colors2[i],label=f"line2_{i}")
    ax.plot(x, y, y2, color=colors2[i],label=f"line2_{i}")


plt.legend()
plt.show()