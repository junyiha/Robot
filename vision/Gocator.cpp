//
// Created by csh_i on 2024/4/28.
//
#include "Gocator.h"

Gocator::Gocator()
{

}

/**
 * @brief TestLidar
 * @return
 *激光雷达检测线缆，得到精确坐标
 */
int Gocator::getData()
{

    kAssembly api = kNULL;
    kStatus status;
    unsigned int i, j, k, arrayIndex;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;
    ProfilePoint* profileBuffer = NULL;
    GoStamp *stamp =kNULL;
    GoDataMsg dataObj;
    kIpAddress ipAddress;
    GoSetup setup = kNULL;
    k32u profilePointCount;

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return 0;
    }

    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return 0;
    }

    // Parse IP address into address data structure
    kIpAddress_Parse(&ipAddress, SENSOR_IP);

    // obtain GoSensor object by sensor IP address
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return 0;
    }

    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return 0;
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return 0;
    }

    // retrieve setup handle
    if ((setup = GoSensor_Setup(sensor)) == kNULL)
    {
        printf("Error: GoSensor_Setup: Invalid Handle\n");
    }

    // retrieve total number of profile points prior to starting the sensor
    if (GoSetup_UniformSpacingEnabled(setup))
    {
        // Uniform spacing is enabled. The number is based on the X Spacing setting
        profilePointCount = GoSetup_XSpacingCount(setup, GO_ROLE_MAIN);
    }
    else
    {
        // non-uniform spacing is enabled. The max number is based on the number of columns used in the camera.
        profilePointCount = GoSetup_FrontCameraWidth(setup, GO_ROLE_MAIN);
    }

    if ((profileBuffer = (ProfilePoint*)malloc(profilePointCount * sizeof(ProfilePoint))) == kNULL)
    {
        printf("Error: Cannot allocate profileData, %d points\n", profilePointCount);
        return 0;
    }

    // start Gocator sensor
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSensor_Start:%d\n", status);
        return 0;
    }

    int validPointCount = 0;
    std::vector<cv::Point2f> pointClouds;
    if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {
        printf("Data message received:\n");
        printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));
        // each result can have multiple data items
        // loop through all items in result message
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            dataObj = GoDataSet_At(dataset, i);
            //Retrieve GoStamp message
            switch(GoDataMsg_Type(dataObj))
            {
                case GO_DATA_MESSAGE_TYPE_STAMP:
                {std::cout<<"GO_DATA_MESSAGE_TYPE_STAMP"<<std::endl;
                    GoStampMsg stampMsg = dataObj;

                    printf("  Stamp Message batch count: %u\n", (k32u)GoStampMsg_Count(stampMsg));
                    for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
                    {
                        stamp = GoStampMsg_At(stampMsg, j);
                        printf("  Timestamp: %llu\n", stamp->timestamp);
                        printf("  Encoder: %lld\n", stamp->encoder);
                        printf("  Frame index: %llu\n", stamp->frameIndex);
                    }
                }
                    break;
                case GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE:
                {std::cout<<"GO_DATA_MESSAGE_TYPE_UNIFORM_PROFILE"<<std::endl;
                    GoResampledProfileMsg profileMsg = dataObj;

                    printf("  Resampled Profile Message batch count: %u\n", (k32u)GoResampledProfileMsg_Count(profileMsg));

                    for (k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
                    {
                        short* data = GoResampledProfileMsg_At(profileMsg, k);
                        double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
                        double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
                        double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
                        double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));

                        //translate 16-bit range data to engineering units and copy profiles to memory array
                        for (arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
                        {
                            if (data[arrayIndex] != INVALID_RANGE_16BIT )
                            {
                                profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
                                profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];
                                validPointCount++;
                                pointClouds.push_back({(float)profileBuffer[arrayIndex].x,(float)profileBuffer[arrayIndex].z});
                            }
                            else
                            {
                                profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
                                profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
                            }
                        }
                        printf("  Profile Valid Point %d out of max %d\n", validPointCount, profilePointCount);
                    }
                }
                    break;
                case GO_DATA_MESSAGE_TYPE_PROFILE_POINT_CLOUD: // Note this is NON resampled profile
                {
                    GoProfileMsg profileMsg = dataObj;

                    printf("Profile Message batch count: %u\n", (k32u)GoProfileMsg_Count(profileMsg));

                    for (k = 0; k < GoProfileMsg_Count(profileMsg); ++k)
                    {
                        kPoint16s* data = GoProfileMsg_At(profileMsg, k);
                        double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
                        double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
                        double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
                        double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));
                        std::cout<<1<<std::endl;
                        //translate 16-bit range data to engineering units and copy profiles to memory array
                        for (arrayIndex = 0; arrayIndex < GoProfileMsg_Width(profileMsg); ++arrayIndex)
                        {
                            if (data[arrayIndex].x != INVALID_RANGE_16BIT)
                            {
                                profileBuffer[arrayIndex].x = XOffset + XResolution * data[arrayIndex].x;
                                profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;
                                validPointCount++;
                            }
                            else
                            {
                                profileBuffer[arrayIndex].x = INVALID_RANGE_DOUBLE;
                                profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
                            }
                        }
                        printf("  Profile Valid Point %d out of max %d\n", validPointCount, profilePointCount);
                    }
                }
                    break;
                case GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY:
                {
                    //kSize validPointCount = 0;
                    GoProfileIntensityMsg intensityMsg = dataObj;
                    printf("Intensity Message batch count: %u\n", (k32u)GoProfileIntensityMsg_Count(intensityMsg));

                    for (k = 0; k < GoProfileIntensityMsg_Count(intensityMsg); ++k)
                    {
                        unsigned char* data = GoProfileIntensityMsg_At(intensityMsg, k);
                        for (arrayIndex = 0; arrayIndex < GoProfileIntensityMsg_Width(intensityMsg); ++arrayIndex)
                        {
                            profileBuffer[arrayIndex].intensity = data[arrayIndex];
                        }
                    }
                }
                    break;
            }
        }
        GoDestroy(dataset);
    }
    else
    {
        printf ("Error: No data received during the waiting period\n");
    }

    // stop Gocator sensor
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        printf("Error: GoSensor_Stop:%d\n", status);
        return 0;
    }

    if(validPointCount==0)
        return 0;

    //@todo: 先不考虑遮挡，若有遮挡，也许可以先考虑通过dfs分割不同的实体轮廓，opencv里有个寻找轮廓的算法findContours，效果好像不太好

    //做到上一步就已经完了，接下来是画出图像进行验证
    float minY=1000000;
    float maxY=0;
    float minX=1000000;
    float maxX=0;
    for(int i=0;i<validPointCount;i++)
    {
        minY=std::min(pointClouds[i].y,minY); //这里的y就是z
        maxY=std::max(pointClouds[i].y,maxY);
        minX=std::min(pointClouds[i].x,minX);
        maxX=std::max(pointClouds[i].x,maxX);
    }
    std::cout<<"minY:"<<minY<<"maxY"<<maxY<<"minX"<<minX<<"maxX"<<maxX<<std::endl;
    //规定row是z，col是x，且计算时要先减去最小值再乘10（放大）+偏置（为了看出整体形状）
    int rows=(maxY-minY)*10+100;
    int cols=(maxX-minX)*10+100;
    cv::Mat m(rows,cols,CV_8UC1,cv::Scalar(0));
    int bias = 50;
    for(int i=0;i<validPointCount;i++){
        m.at<char>((pointClouds[i].y-minY)*10+bias,(pointClouds[i].x-minX)*10+bias)=255; //at <类型> (行,列) [通道(如果有通道的话)]
    }
    std::cout<<m.size()<<std::endl;
    lidar_data=m;

    cv::Size dsize = cv::Size(m.cols/2, m.rows/2);
    cv::Mat shrink,shrink_c;
    resize(m, shrink, dsize, 0, 0, cv::INTER_AREA);
    cvtColor(shrink,shrink_c, cv::COLOR_GRAY2BGR);
    return 0;
}
