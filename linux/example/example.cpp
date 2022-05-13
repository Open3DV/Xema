// example.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream> 
#include <string.h>
#include "open_cam3d_sdk.h"

int main()
{
    /*****************************************************************************************************/

    //连接相机
    int ret_code = DfConnect("192.168.0.103");

    int width = 0, height = 0;

    if (0 == ret_code)
    {
        //必须连接相机成功后，才可获取相机分辨率
        ret_code = DfGetCameraResolution(&width, &height);
        std::cout << "Width: " << width << "    Height: " << height << std::endl;
    }
    else
    {
        std::cout << "Connect Camera Error!";
        return -1;
    }

    //获取相机的标定参数
    CalibrationParam calib_param;
    ret_code = DfGetCalibrationParam(&calib_param);

    if (0 == ret_code)
    {
        std::cout << "intrinsic: " << std::endl;
        for (int r = 0; r < 3; r++)
        {
            for (int c = 0; c < 3; c++)
            {
                std::cout << calib_param.intrinsic[3 * r + c] << "\t";
            }
            std::cout << std::endl;
        }

        std::cout << "extrinsic: " << std::endl;
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                std::cout << calib_param.extrinsic[4 * r + c] << "\t";
            }
            std::cout << std::endl;
        }

        std::cout << "distortion: " << std::endl;
        for (int r = 0; r < 1; r++)
        {
            for (int c = 0; c < 12; c++)
            {
                std::cout << calib_param.distortion[1 * r + c] << "\t";
            }
            std::cout << std::endl;
        }
    }
    else
    {
        std::cout << "Get Calibration Data Error!" << std::endl;
        return -1;
    }

    //分配内存保存采集结果
    float* point_cloud_data = (float*)malloc(sizeof(float) * width * height * 3);
    memset(point_cloud_data, 0, sizeof(float) * width * height * 3);

    float* height_map_data = (float*)malloc(sizeof(float) * width * height);
    memset(height_map_data, 0, sizeof(float) * width * height);

    unsigned short* depth_data = (unsigned short*)malloc(sizeof(unsigned short) * width * height);
    memset(depth_data, 0, sizeof(unsigned short) * width * height);

    char* timestamp_data = (char*)malloc(sizeof(char) * 30);
    memset(timestamp_data, 0, sizeof(char) * 30);

    unsigned char* brightness_data = (unsigned char*)malloc(sizeof(unsigned char) * width * height);
    memset(brightness_data, 0, sizeof(unsigned char) * width * height);

    int capture_num = 0;

    if (0 == ret_code)
    {
        //采集单曝光数据
        if (false)
        {
            //设置投影亮度参数
            ret_code = DfSetParamLedCurrent(1023);
            if (0 != ret_code)
            {
                std::cout << "Set LED Current Error!" << std::endl;
            } 

            //设置相机曝光时间（us）
            ret_code = DfSetParamCameraExposure(12000);
            if (0 != ret_code)
            {
                std::cout << "Set Camera Exposure Error!" << std::endl;
            }

            //采集一帧单次曝光的数据
            ret_code = DfCaptureData(1, timestamp_data);
            std::cout << "Capture Single Exposure Data" << std::endl;
            std::cout << "timestamp: " << timestamp_data << std::endl;
        }
		else
		{
			//采集HDR模式数据 
            int num = 2;
            int led_param[6] = { 100,1023,1023,1023,1023,1023 };
            int exposure_param[6] = { 6000,30000,60000,60000,60000,60000 };

            //设置多曝光参数
            ret_code = DfSetParamMixedHdr(num, exposure_param, led_param);

			if (0 != ret_code)
			{
                std::cout << "Set HDR Param Error;" << std::endl; 
			}

            //采集一帧多曝光的数据
            ret_code = DfCaptureData(num, timestamp_data);
            std::cout << "Capture HDR Data" << std::endl;
            std::cout << "timestamp: " << timestamp_data << std::endl;
		}
          

        if (0 == ret_code)
        {
            //获取亮度图数据
            ret_code = DfGetBrightnessData(brightness_data);
            if (0 == ret_code)
            {
                std::cout << "Get Brightness!" << std::endl;
            }

            //获取深度图数据
            ret_code = DfGetDepthData(depth_data);

            if (0 == ret_code)
            {
                std::cout << "Get Depth!" << std::endl;
            } 

            //获取高度映射图数据
            ret_code = DfGetHeightMapData(height_map_data);

            if (0 == ret_code)
            {
                std::cout << "Get Height Map!" << std::endl;
            }

            //获取点云数据
            ret_code = DfGetPointcloudData(point_cloud_data);
            if (0 == ret_code)
            {
                std::cout << "Get Pointcloud!" << std::endl;
            }

            //动态获取基准平面高度映射图
            float plane_R[9] = { 1,0,0,0,1,0,0,0,1 };
            float plane_T[3] = { 0,0,0};

            //动态获取高度映射图数据
            ret_code = DfGetHeightMapDataBaseParam(plane_R, plane_T, height_map_data);
            if (0 == ret_code)
            {
                std::cout << "Get Height Map Base Param!" << std::endl;
            }

            capture_num++;
            std::cout << "Capture num: " << capture_num << std::endl;

        }
        else
        {

            std::cout << "Capture Data Error!" << std::endl;

        }

    }
     
    free(brightness_data);
    free(depth_data);
    free(point_cloud_data);
    free(height_map_data);
    free(timestamp_data);

    DfDisconnect("192.168.88.106");
}

 