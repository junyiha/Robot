'''

'''
import os
import shutil

destination_dir = "C:/Users/anony/Desktop/碰钉软件程序发布/"
program_path = "D:/Robot/out/build/PDRobot-master.exe"

def CopyLibrary():
    BaseDir = "D:"
    list = []

    list.append( BaseDir + "/packages/ffmpeg/bin/avcodec-59.dll")
    list.append( BaseDir + "/packages/ffmpeg/bin/avformat-59.dll")
    list.append( BaseDir + "/packages/ffmpeg/bin/avformat-59.dll")
    list.append( BaseDir + "/packages/ffmpeg/bin/swresample-4.dll")
    list.append( BaseDir + "/packages/opencv4.55/build/x64/vc15/bin/opencv_world455d.dll")
    list.append( BaseDir + "/packages/opencv4.55/build/x64/vc15/bin/opencv_world455.dll")
    list.append( BaseDir + "/packages/HK_SDK/Libraries/win64/MvCameraControl.lib")
    list.append( BaseDir + "/packages/GocatorSDK/bin/win64/kApi.dll")
    list.append( BaseDir + "/packages/GocatorSDK/bin/win64/GoSdk.dll")
    list.append( BaseDir + "/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.1.7.0/runtimes/win-x64/native/onnxruntime.dll")
    list.append(program_path)

    for i in list:
        print(i)
        shutil.copy(i, destination_dir)


os.system("windeployqt " + program_path + " --dir " + destination_dir)

CopyLibrary()