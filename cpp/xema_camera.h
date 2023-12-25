#pragma once 
#ifndef __XEMA_CAMERA_H__
#define __XEMA_CAMERA_H__

#include "xcamera.h"
#include <mutex>
#include <thread>
#include "../sdk/socket_tcp.h" 
#include "../sdk/xema_enums.h"
#include "../firmware/camera_param.h" 
#include "../firmware/system_config_settings.h"

extern "C" {
	namespace XEMA {

		class XemaCamera : public XCamera
		{
		public:
			XemaCamera();
			~XemaCamera();
			XemaCamera(const XemaCamera&) = delete;
			XemaCamera& operator=(const XemaCamera&) = delete;

			//功能： 连接相机
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
			int connect(const char* camera_id)override;

			//功能： 断开相机连接
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
			int disconnect(const char* camera_id)override;

			//功能： 获取相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int getCameraResolution(int* width, int* height)override;

			//函数名： DfGetCameraChannels
			//功能： 获取相机图像通道数
			//输入参数： 无
			//输出参数： channels(通道数)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int getCameraChannels(int* channels)override;

			//函数名： DfSetCaptureEngine
			//功能： 设置采集引擎
			//输入参数：engine
			//输出参数：  
			//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
			int setCaptureEngine(XemaEngine engine)override;

			//函数名： DfGetCaptureEngine
			//功能： 设置采集引擎
			//输入参数：
			//输出参数：engine
			//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
			int getCaptureEngine(XemaEngine& engine)override;

			//功能： 采集一帧数据并阻塞至返回状态
			//输入参数： exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
			//输出参数： timestamp(时间戳)
			//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
			int captureData(int exposure_num, char* timestamp)override;

			//功能： 获取去畸变深度图
			//输入参数：无
			//输出参数： undistort_depth(深度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getUndistortDepthData(float* undistort_depth)override;

			//功能： 获取深度图
			//输入参数：无
			//输出参数： depth(深度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败. 
			int getDepthData(float* depth)override;
			
			//功能： 获取点云
			//输入参数：无
			//输出参数： point_cloud(点云)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getPointcloudData(float* point_cloud)override;

			//功能： 获取去畸变亮度图
			//输入参数：无
			//输出参数： undistort_brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getUndistortBrightnessData(unsigned char* undistort_brightness)override;

			//功能： 获取亮度图
			//输入参数：无
			//输出参数： brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getBrightnessData(unsigned char* brightness)override;

			//函数名： getColorBrightnessData
			//功能： 获取亮度图
			//输入参数：无
			//输出参数： brightness(亮度图),color(亮度图颜色类型)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getColorBrightnessData(unsigned char* brightness, XemaColor color)override;

			//函数名： getUndistortColorBrightnessData
			//功能： 获取去畸变后的彩色亮度图
			//输入参数：无
			//输出参数： brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getUndistortColorBrightnessData(unsigned char* brightness, XemaColor color)override;

			//功能： 获取校正到基准平面的高度映射图
			//输入参数：无  
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getHeightMapData(float* height_map)override;

			//功能： 获取基准平面参数
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getStandardPlaneParam(float* R, float* T)override;

			//功能： 获取校正到基准平面的高度映射图
			//输入参数：R(旋转矩阵)、T(平移矩阵)
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getHeightMapDataBaseParam(float* R, float* T, float* height_map)override;


			 
			//功能： 获取相机标定参数
			//输入参数： 无
			//输出参数： calibration_param（相机标定参数结构体）
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			int getCalibrationParam(struct CalibrationParam* calibration_param)override;
 
			/***************************************************************************************************************************************************************/
			//参数设置

			//功能： 设置LED电流
			//输入参数： led（电流值）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamLedCurrent(int led)override;

			//功能： 设置LED电流
			//输入参数： 无
			//输出参数： led（电流值）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamLedCurrent(int& led)override;
			 
			//功能： 设置基准平面的外参
			//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamStandardPlaneExternal(float* R, float* T)override;

			//功能： 获取基准平面的外参
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamStandardPlaneExternal(float* R, float* T)override;

			//功能： 设置生成亮度图参数
			//输入参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamGenerateBrightness(int model, float exposure)override;

			//功能： 获取生成亮度图参数
			//输入参数： 无
			//输出参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamGenerateBrightness(int& model, float& exposure)override;

			//功能： 设置相机曝光时间
			//输入参数：exposure(相机曝光时间)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraExposure(float exposure)override;

			//功能： 获取相机曝光时间
			//输入参数： 无
			//输出参数：exposure(相机曝光时间)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraExposure(float& exposure)override;

			//功能： 设置混合多曝光参数（最大曝光次数为6次）
			//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamMixedHdr(int num, int exposure_param[6], int led_param[6])override;

			//功能： 获取混合多曝光参数（最大曝光次数为6次）
			//输入参数： 无
			//输出参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效）、led_param[6]（6个led亮度参数、前num个有效）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamMixedHdr(int& num, int exposure_param[6], int led_param[6])override;

			//功能： 设置相机曝光时间
			//输入参数：confidence(相机置信度)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraConfidence(float confidence)override;

			//功能： 获取相机曝光时间
			//输入参数： 无
			//输出参数：confidence(相机置信度)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraConfidence(float& confidence)override;

			//功能： 设置相机增益
			//输入参数：gain(相机增益)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraGain(float gain)override;

			//功能： 获取相机增益
			//输入参数： 无
			//输出参数：gain(相机增益)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraGain(float& gain)override;

			//功能： 设置点云平滑参数
			//输入参数：smoothing(0:关、1-5:平滑程度由低到高)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamSmoothing(int smoothing)override;

			//功能： 设置点云平滑参数
			//输入参数：无
			//输出参数：smoothing(0:关、1-5:平滑程度由低到高)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamSmoothing(int& smoothing)override;

			//功能： 设置点云半径滤波参数
			//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamRadiusFilter(int use, float radius, int num)override;

			//功能： 获取点云半径滤波参数
			//输入参数：无
			//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamRadiusFilter(int& use, float& radius, int& num)override;

			//函数名： setParamDepthFilter
			//功能： 设置深度图滤波参数
			//输入参数：use(开关：1开、0关)、depth_filterthreshold(深度图在1000mm距离过滤的噪声阈值)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamDepthFilter(int use, float depth_filter_threshold)override;

			//函数名： getParamDepthFilter
			//功能： 设置深度图滤波参数
			//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamDepthFilter(int& use, float& depth_filter_threshold)override;

			//功能： 设置外点过滤阈值
			//输入参数：threshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamOutlierFilter(float threshold)override;

			//功能： 获取外点过滤阈值
			//输入参数： 无
			//输出参数：threshold(阈值0-100)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamOutlierFilter(float& threshold)override;

			//功能： 设置多曝光模式
			//输入参数： model(1：HDR(默认值)、2：重复曝光)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamMultipleExposureModel(int model)override;

			//功能： 设置重复曝光数
			//输入参数： num(2-10)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamRepetitionExposureNum(int num)override;


			//函数名： setParamGrayRectify
			//功能： 设置点云灰度补偿参数
			//输入参数：use(开关：1开、0关)、radius(半径：3、5、7、9）、sigma（补偿强度，范围0-100）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamGrayRectify(int use, int radius, float sigma)override;

			//函数名： getParamGrayRectify
			//功能： 获取点云灰度补偿参数
			//输入参数：无
			//输出参数：use(开关：1开、0关)、radius(半径：3、5、7、9）、sigma（补偿强度，范围0-100）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamGrayRectify(int& use, int& radius, float& sigma)override;

			//函数名： setParamBrightnessHdrExposure
			//功能： 设置亮度图多曝光参数（最大曝光次数为10次）
			//输入参数： num（曝光次数）、exposure_param[6]（6个曝光参数、前num个有效））
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamBrightnessHdrExposure(int num, int exposure_param[10])override;

			//函数名：getParamBrightnessHdrExposure
			//功能： 设置亮度图多曝光参数（最大曝光次数为10次）
			//输入参数：无 
			//输出参数：num（曝光次数）、exposure_param[10]（10个曝光参数、前num个有效））
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int getParamBrightnessHdrExposure(int& num, int exposure_param[10])override;

			//函数名： setParamBrightnessExposureModel
			//功能： 设置亮度图曝光模式
			//输入参数： model（1：单曝光、2：曝光融合）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamBrightnessExposureModel(int model)override;

			//函数名： getParamBrightnessExposureModel
			//功能： 获取亮度图曝光模式
			//输入参数： 无
			//输出参数： model（1：单曝光、2：曝光融合）
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int getParamBrightnessExposureModel(int& model)override;

			//函数名： setParamBrightnessGain
			//功能： 设置亮度图增益
			//输入参数：gain(亮度图增益)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamBrightnessGain(float gain)override;

			//函数名： getParamBrightnessGain
			//功能： 获取亮度图增益
			//输入参数：无
			//输出参数：gain(亮度图增益)
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int getParamBrightnessGain(float& gain)override;

			//函数名： getSdkVersion
			//功能： 获取sdk版本
			//输入参数：无
			//输出参数：version(版本)
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int getSdkVersion(char version[64])override;

			//函数名： DfCaptureBrightnessData
			//功能： 获取亮度图
			//输入参数： color(图像颜色类型)
			//输出参数： brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int captureBrightnessData(unsigned char* brightness, XemaColor color)override;

			//函数名： setParamReflectFilter
			//功能： 设置反射滤波参数
			//输入参数：use(开关：1开、0关)、param_b（过滤系数：范围0-100）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamReflectFilter(int use, float param_b) override;

			//函数名： getParamReflectFilter
			//功能： 获取反射滤波参数
			//输入参数：无
			//输出参数：use(开关：1开、0关)、param_b（过滤系数：范围0-100）
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int getParamReflectFilter(int& use, float& param_b) override;

		public:

			int rgbToGray(unsigned char* src, unsigned char* dst);

			int bayerToRgb(unsigned char* src, unsigned char* dst);

			int undistortColorBrightnessMap(unsigned char* brightness_map);

			int undistortBrightnessMap(unsigned char* brightness_map);

			int undistortDepthMap(float* depth_map);

			bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation);

			int depthTransformPointcloud(float* depth_map, float* point_cloud_map);

			int getBrightness(unsigned char* brightness, int brightness_buf_size);

			int getFrame04(float* depth, int depth_buf_size,unsigned char* brightness, int brightness_buf_size);

			int getRepetitionFrame04(int count, float* depth, int depth_buf_size, unsigned char* brightness, int brightness_buf_size);

			int getFrame04Hdr(float* depth, int depth_buf_size, unsigned char* brightness, int brightness_buf_size);
 
			int getFrame06(float* depth, int depth_buf_size,unsigned char* brightness, int brightness_buf_size);

			int getFrame06Hdr(float* depth, int depth_buf_size,unsigned char* brightness, int brightness_buf_size);

			int getRepetitionFrame06(int count, float* depth, int depth_buf_size,unsigned char* brightness, int brightness_buf_size);
 
			int getFrame06Mono12(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size);

			int getFrame06HdrMono12(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size);

			int getRepetitionFrame06Mono12(int count, float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size); 

			int getSystemConfigParam(struct SystemConfigParam& config_param);
 
			int setParamBilateralFilter(int use, int param_d);

			int getParamBilateralFilter(int& use, int& param_d);

			int getCalibrationParam(struct CameraCalibParam& calibration_param);
 
			int getFrameStatus(int& status);
 
			int getCameraPixelType(int& type);

			std::string get_timestamp();

			std::time_t getTimeStamp(long long& msec);

			std::tm* gettm(long long timestamp);
		public:

			int registerOnDropped(int (*p_function)(void*));

			int connectNet(const char* ip);

			int disconnectNet();
			 
			int HeartBeat();

			int HeartBeat_loop();

		private:
			int (*p_OnDropped_)(void*) = 0;

			XemaEngine engine_ = XemaEngine::Normal;

			XemaPixelType pixel_type_ = XemaPixelType::Mono;

			std::string camera_ip_;
			int multiple_exposure_model_ = 1;
			int repetition_exposure_model_ = 2;


			std::timed_mutex command_mutex_;
			std::timed_mutex undistort_mutex_;

			std::thread heartbeat_thread;
			int heartbeat_error_count_ = 0;

			SOCKET g_sock_heartbeat;
			SOCKET g_sock;
			bool connected = false;
			long long token = 0;

			int camera_width_ = 1920;
			int camera_height_ = 1200;

			struct CameraCalibParam calibration_param_;
			bool connected_flag_ = false;

			int depth_buf_size_ = 0;
			int pointcloud_buf_size_ = 0;
			int brightness_bug_size_ = 0;
			float* point_cloud_buf_ = NULL;
			float* trans_point_cloud_buf_ = NULL;
			bool transform_pointcloud_flag_ = false;
			float* depth_buf_ = NULL;
			unsigned char* brightness_buf_ = NULL;
			float* undistort_map_x_ = NULL;
			float* undistort_map_y_ = NULL;
			float* distorted_map_x_ = NULL;
			float* distorted_map_y_ = NULL;

			unsigned char* rgb_buf_ = NULL;
			bool bayer_to_rgb_flag_ = false;
		};
		 

	}
}
#endif
