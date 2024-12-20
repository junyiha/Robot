'''
    打包源代码文件，方便拷贝源码到工控机电脑
'''
import os
import zipfile
from pathlib import Path
from get_path import GetRootDir
from ignore_items import is_ignored

root_dir = GetRootDir()
print("当前项目所在根目录目录是:", root_dir)

file_list = os.listdir(root_dir)
new_list = []
for i in file_list:
    if is_ignored(i):
        continue
    temp_path = Path(root_dir) / i
    new_list.append(i)

script_dir = Path(root_dir) / "package/"
if not os.path.exists(script_dir):
    os.makedirs(script_dir)

script_dir = Path(script_dir) / "Robot.zip"
print(script_dir)

if os.path.exists(script_dir):
    os.remove(script_dir)

with zipfile.ZipFile(script_dir, 'a') as zipfile:
    for i in new_list:
        if os.path.isdir(i):
            for root, dirs, files in os.walk(i):
                for file in files:
                    file_path = os.path.join(root, file)
                    zipfile.write(file_path)
        else:
            zipfile.write(i)