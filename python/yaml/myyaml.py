import yaml
from pprint import pprint

# open 函数有三个使用目的:
#      1. 读取
#      2. 写入
#      3. 追加
# open(file, mode='r', buffering=-1, encoding=None, errors=None, newline=None, closefd=True, opener=None)
# file: 要打开的文件名或文件路径（可以是相对路径或绝对路径）。
# mode: 文件打开模式（见下文详细说明）。默认是 r
# buffering: 设置缓冲策略，-1 为默认，0 为无缓冲，1 为行缓冲，其他正整数为缓冲区大小。
# encoding: 文件的编码方式（如 'utf-8'），通常在以文本模式读取或写入时使用。
# errors: 处理编码错误的方式（如 'ignore'、'replace'）。
# newline: 控制换行符的处理方式（仅适用于文本模式）。
# closefd: 是否在文件关闭时关闭文件描述符（默认 True）。
# opener: 自定义打开文件的函数。
"""
'r'	只读模式（默认模式）。文件必须存在，否则会抛出 FileNotFoundError。
'w'	写入模式。如果文件不存在会创建新文件，如果文件存在则会覆盖原有内容。
'a'	追加模式。在文件末尾添加内容。如果文件不存在，会创建新文件。
'b'	二进制模式。与其他模式一起使用，如 'rb'（二进制读取），'wb'（二进制写入）。
't'	文本模式（默认模式）。可以与其他模式结合使用，如 'rt'（文本读取）。
'x'	创建新文件并写入内容。如果文件已存在，则会抛出 FileExistsError。
'+'	读写模式，可以与其他模式结合使用，如 'r+'、'w+'、'a+'。
"""
# 使用 with 语句，确保文件在使用后自动关闭。
with open('conf.yaml', 'r') as file:
  """
  当文件以 open 打开后，你可以使用以下方法来操作文件：
    read(size): 读取指定大小的内容（单位：字节）。如果省略 size,则读取整个文件。
    readline(): 读取一行内容。
    readlines(): 读取所有行并返回一个列表。
    write(string): 写入字符串到文件。
    writelines(lines): 将一个字符串列表写入文件。
    seek(offset, whence): 移动文件指针到指定位置。
    tell(): 获取当前文件指针的位置。
    close(): 关闭文件（通常不需要手动调用,with 语句会自动关闭）。
  在读取文件时，确保文件存在且有读取权限，否则可能抛出 FileNotFoundError 或 PermissionError。
  在写入文件时，如果不想覆盖原有内容，可以使用追加模式 'a'。
  在处理不同编码的文件时，显式指定 encoding 以避免编码错误。

  """
  # yaml 文件会被加载成为一个字典,因此可以像一个字典样来使用
  # tips 这里是最重要的, 返回的是一个字典
  conf = yaml.safe_load(file)

pprint(conf)
pprint(conf['trim_info']['guai1'][0]['point'])
# print(f'hostname:{conf["database"]["host"]}')
# conf['guai1']['mode'] = 'beacon'

# with open('beacon.yaml', 'w') as file:
  # yaml.dump(conf, file)

