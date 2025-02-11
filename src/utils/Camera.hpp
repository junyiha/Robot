/**
 * @file Camera.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-01-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "utils/basic_header.hpp"

#include "MvCameraControl.h"

namespace Utils
{
    struct CameraData_t
    {
        std::string ip;
        void* handle;
        MV_CC_DEVICE_INFO* device_info;
        std::thread* thread;
    };

    class Camera
    {
    public:
        Camera()
        {
            SearchDevice();
        }
        ~Camera()
        {

        }

        std::map<int, CameraData_t> GetCameraMap() const
        {
            return m_camera_map;
        }

        bool Open(int camera_index)
        {
            auto it = m_camera_map.find(camera_index);
            if (it == m_camera_map.end())
            {
                SPDLOG_ERROR("camera index: {} is invalid", camera_index);
                return -1;
            }

            int res{ 0 };
            res = MV_CC_CreateHandle(&it->second.handle, it->second.device_info);
            if (res != MV_OK)
            {
                SPDLOG_ERROR("Failed to create handle, error code: {}", res);
                return -1;
            }

            res = MV_CC_OpenDevice(it->second.handle);
            if (res != MV_OK)
            {
                SPDLOG_ERROR("Failed to open device, error code: {}", res);
                return -1;
            }

            unsigned char* pConvertData = NULL;
            unsigned int nConvertDataSize = 0;

            // Detection network optimal package size(It only works for the GigE camera)
            if (it->second.device_info->nTLayerType == MV_GIGE_DEVICE)
            {
                int nPacketSize = MV_CC_GetOptimalPacketSize(it->second.handle);
                if (nPacketSize > 0)
                {
                    res = MV_CC_SetIntValue(it->second.handle, "GevSCPSPacketSize", nPacketSize);
                    if (res != MV_OK)
                    {
                        SPDLOG_ERROR("Warning: Set Packet Size fail res [0x{}]!", res);
                    }
                }
                else
                {
                    SPDLOG_ERROR("Warning: Get Packet Size fail res [0x{}]!", nPacketSize);
                }
            }

            res = MV_CC_SetEnumValue(it->second.handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
            if (MV_OK != res)
            {
                SPDLOG_ERROR("Set Trigger Mode fail! res [0x{}]\n", res);
                return -1;
            }
        }

        bool StartCapture(int camera_index)
        {
            int res{ 0 };
            void* handle = nullptr;
            MV_CC_DEVICE_INFO* device_info{ nullptr };

            auto it = m_camera_map.find(camera_index);
            if (it == m_camera_map.end())
            {
                SPDLOG_ERROR("camera index: {} is invalid", camera_index);
                return -1;
            }

            handle = it->second.handle;
            device_info = it->second.device_info;

            res = MV_CC_StartGrabbing(handle);
            if (MV_OK != res)
            {
                SPDLOG_ERROR("Start Grabbing fail! res [0x{}]\n", res);
                return -1;
            }

            it->second.thread = new std::thread([this](std::pair<int, CameraData_t> camera_data) {
                while (true)
                {   // 87ms
                    int res{ 0 };
                    MV_FRAME_OUT stImageInfo = { 0 };
                    auto begin = std::chrono::steady_clock::now();
                    res = MV_CC_GetImageBuffer(camera_data.second.handle, &stImageInfo, 1000);
                    if (res != MV_OK)
                    {
                        SPDLOG_ERROR("No data[0x{}]\n", res);
                        return -1;
                    }

                    SPDLOG_INFO("Get One Frame: Width[{}], Height[{}], nFrameNum[{}]\n",
                        stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nFrameNum);

                    cv::Mat src_img = Convert2Mat(&stImageInfo.stFrameInfo, stImageInfo.pBufAddr);  // 0ms
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin);
                    SPDLOG_INFO("camera[{}] duration: {} ms", camera_data.first, duration.count());

                    MV_CC_FreeImageBuffer(camera_data.second.handle, &stImageInfo);
                }
            }, *it);
        }

        void Stop(int camera_index)
        {
            int res{ 0 };
            void* handle = nullptr;
            MV_CC_DEVICE_INFO* device_info{ nullptr };

            auto it = m_camera_map.find(camera_index);
            if (it == m_camera_map.end())
            {
                SPDLOG_ERROR("camera index: {} is invalid", camera_index);
                return;
            }

            handle = it->second.handle;
            // Stop grab image
            res = MV_CC_StopGrabbing(handle);
            if (MV_OK != res)
            {
                SPDLOG_ERROR("Stop Grabbing fail! res [0x{}]\n", res);
                return;
            }

            // Close device
            res = MV_CC_CloseDevice(handle);
            if (MV_OK != res)
            {
                SPDLOG_ERROR("ClosDevice fail! res [0x{}]\n", res);
                return;
            }

            // Destroy handle
            res = MV_CC_DestroyHandle(handle);
            if (MV_OK != res)
            {
                SPDLOG_ERROR("Destroy Handle fail! res [0x{}]\n", res);
                return;
            }

            if (res != MV_OK)
            {
                if (handle != NULL)
                {
                    MV_CC_DestroyHandle(handle);
                    handle = NULL;
                }
            }
        }

    private:
        void SearchDevice()
        {
            int res{ 0 };
            res = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
            if (res != MV_OK)
            {
                SPDLOG_ERROR("Failed to enumerate devices, error code: {}", res);
                return;
            }

            SPDLOG_INFO("find {} device", device_list.nDeviceNum);
            for (int i = 0; i < device_list.nDeviceNum; i++)
            {
                if (device_list.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
                {
                    int nIp1 = ((device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
                    int nIp2 = ((device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
                    int nIp3 = ((device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
                    int nIp4 = (device_list.pDeviceInfo[i]->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

                    std::stringstream os_out;
                    os_out << "ip: " << nIp1 << "." << nIp2 << "." << nIp3 << "." << nIp4;

                    CameraData_t camera_data;
                    camera_data.handle = nullptr;
                    camera_data.device_info = device_list.pDeviceInfo[i];
                    camera_data.ip = os_out.str();
                    m_camera_map.insert(std::make_pair(i, camera_data));
                }
            }
        }

        cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
        {
            cv::Mat srcImage;
            if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
            {
                srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
            }
            else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
            {
                srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
            }
            else
            {
                printf("unsupported pixel format\n");
            }
            return srcImage;
        }

    private:
        MV_CC_DEVICE_INFO_LIST device_list;
        std::map<int, CameraData_t> m_camera_map;
    };
}  // namespace Utils