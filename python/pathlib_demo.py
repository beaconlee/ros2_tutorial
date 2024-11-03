from pathlib import Path
from datetime import datetime


p = Path('/workspace/python')


filename = p.name
print(f'filename={filename}')

new_path = p / 'argv.py'

print(f'new_path:{new_path}')

# 判断是否存在
if p.exists():
  print(f'p[{p}] is exists')
else:
  print(f'p[{p}] is not exists')



# 判断是文件还是文件夹
if p.is_file():
  print(f'p is file')
if p.is_dir():
  print(f'p is dir')


# 创建目录
# 创建一个目录

p = Path('/workspace/python')

"""
parents=True:

当设置为 True 时，如果路径中的父目录不存在，会自动创建这些父目录。例如，如果你要创建 /home/user/documents/new_folder，而 documents 目录不存在，设置 parents=True 会自动创建 documents 目录，然后再创建 new_folder。
如果 parents=False，则在父目录不存在时，会抛出 FileNotFoundError 错误。
exist_ok=True:

当设置为 True 时，如果目标目录已经存在，不会抛出 FileExistsError 错误。相当于说 "如果目录已经存在，也没关系"。
如果 exist_ok=False（默认值），且目标目录已经存在，会抛出 FileExistsError。
"""
p.mkdir(parents=True, exist_ok=True)

p = Path('/workspace/python/beacon')
p.mkdir(parents=True, exist_ok=True)

p = Path('/workspace/python/beacon2')
p.mkdir(parents=True, exist_ok=True)

# 删除目录
p.rmdir()



# 遍历目录中的文件
for file in p.iterdir():
  print(f'file:{file}')

if p.is_file():
  print(f'file name:{p.name}')
  print(f'file name without suffix:{p.stem}')
  print(f'file suffix:{p.suffix}')
  print(f'file parent dir:{p.parent}')


# 拼接路径, 使用 / 运算符来拼接路径。
p = Path('/workspace/python')
new_file = p / 'log.py'
print(f'new file:{new_file}')


file_path = Path('/workspace/python/demo.txt')

file_path.write_text('hello beacon')

content = file_path.read_text()
print(f'file content:{content}')

file_path.rename('/workspace/python/beacon_demo.txt')

file_path = Path('/workspace/python/demo.txt')
file_path.write_text('hello beacon')

# 获取文件大小
file_size = file_path.stat()
print(f'file size:{file_size}')

# 查找匹配文件
# 在当前目录查找所有 .txt 文件
p = Path('workspace/python')
if p.is_dir():
  for file_path in p.glob('*.txt'):
      print(file_path)

  # 递归查找所有子目录中的 .txt 文件
  for file_path in p.rglob('*.txt'):
      print(file_path)

# 获取绝对路径
absolute_path = p.resolve()
print(f'file absolute path:{absolute_path}')


# 获取文件的修改时间
modification_time = file_path.stat().st_mtime
print(f'file modiy time:{modification_time}')

# 删除文件
file_path.unlink()
