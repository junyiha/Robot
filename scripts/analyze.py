'''
    分析对边阶段耗时原因
'''
from datetime import datetime
import re
import os
from pathlib import Path

save_path = "D:/Robot/logs/定位阶段日志收集.log"

def FindLogFiles():
    '''
        查找日志文件，返回文件路径
    '''
    root_dir = Path(__file__).resolve().parent.parent
    print("当前脚本所在目录是:", root_dir)
    root_dir = root_dir / "logs"

    log_list = []
    file_list = os.listdir(root_dir)
    for i in file_list:
        log_list.append(root_dir / i)

    for i in log_list:
        if i == Path(save_path):
            log_list.remove(i)
        print(i)

    return log_list

def CaculateTime(log_str):
    '''
        计算定位时间
    '''
    # 使用正则表达式提取时间戳
    timestamp_pattern = r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]"
    match = re.search(timestamp_pattern, log_str)

    if match:
        timestamp = match.group(1)
        # 将时间戳转换为 datetime 对象
        dt = datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f")

        # 将 datetime 对象转换为秒
        seconds = dt.timestamp()
        print("Timestamp in seconds:", seconds)
        return seconds,timestamp
    else:
        print("No timestamp found")
        return None


def SaveData(duration, result):
    '''
        保存数据
    '''
    if len(result) == 0:
        return

    begin_seconds,begin_time = CaculateTime(result[0])
    end_seconds,end_time = CaculateTime(result[-1])
    duration = end_seconds - begin_seconds

    with open(save_path, 'a') as f:
        for i in result:
            f.write(i)
        f.write("定位阶段开始时间: {}\n".format(begin_time))
        f.write("定位阶段结束时间: {}\n".format(end_time))
        f.write("定位阶段耗时: {}\n".format(duration))
        f.write("---------------------------------------------\n\n")

def Collection(path):
    begin_str = "状态跳转: 定位--待定位 ==> 定位--检测"
    end_str = "状态跳转: 定位--检测 ==> 贴合--待贴合"

    caculate_target_log = "[error]: 边线距离需要继续调整"
    caculate_target_log2 = "[info]: 完成装板定位"

    flag = False
    result = []
    buffer = []
    with open(path, 'r') as f:
        while True:
            lines = f.readline()
            if not lines:
                print(len(result))
                break
            if begin_str in lines:
                buffer.clear()
                flag = True

            if flag:
                if caculate_target_log in lines or caculate_target_log2 in lines:
                    buffer.append(lines)

            if end_str in lines:
                buffer.append(lines)
                flag = False
                result.append(buffer.copy())
                print(len(result))

    return result


if __name__ == "__main__":
    '''
        主函数
    '''
    if os.path.exists(save_path):
        os.remove(save_path)

    log_list = FindLogFiles()

    data_dict = {}

    for i in log_list:
        data = Collection(i)
        for i in data:
            begin_seconds,begin_time = CaculateTime(i[0])
            end_seconds,end_time = CaculateTime(i[-1])
            duration = end_seconds - begin_seconds
            data_dict[duration] = i

    sorted_dict = dict(sorted(data_dict.items(), reverse=True))
    for i in sorted_dict:
        if i < 1:
            continue
        SaveData(i, sorted_dict[i])