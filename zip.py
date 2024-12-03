'''
    打包源代码文件，方便拷贝源码到工控机电脑
'''
import os
import zipfile
from pathlib import Path

root_dir = Path(__file__).resolve().parent
print("当前脚本所在目录是:", root_dir)

file_list = os.listdir(root_dir)
new_list = []
for i in file_list:
    if i == '.vscode' or i == 'build' or i == "package":
        continue

    new_list.append(i)

script_dir = Path(root_dir) / "package/robot.zip"
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