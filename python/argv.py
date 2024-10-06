import sys

num_args = len(sys.argv) - 1  # 减去 1 是因为 sys.argv[0] 是脚本名

print(sys.argv[0])

# 'int' object is not iterable
# 这里的 num_args 是一个整数,不是一个可迭代的对象(例如列表)
# for i in num_args:
# 下面这个语法是左闭右开区间的, 所以不能再减1了
# for i in range(1, num_args):
for i in range(1, len(sys.argv)):

  print(sys.argv[i])


