#pragma once 
#ifndef __XEMA_CAMERA_H__
#define __XEMA_CAMERA_H__

#include "xcamera.h"
#include <mutex>
#include <thread>
#include "../sdk/socket_tcp.h" 
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

			//���ܣ� �������
			//��������� camera_id�����ip��ַ��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
			int connect(const char* camera_id)override;

			//���ܣ� �Ͽ��������
			//��������� camera_id�����ip��ַ��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ�Ͽ��ɹ�;����-1��ʾ�Ͽ�ʧ��.
			int disconnect(const char* camera_id)override;

			//���ܣ� ��ȡ����ֱ���
			//��������� ��
			//��������� width(ͼ���)��height(ͼ���)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����-1��ʾ��ȡ����ʧ��.
			int  getCameraResolution(int* width, int* height)override;

			//���ܣ� �ɼ�һ֡���ݲ�����������״̬
			//��������� exposure_num���ع������������ֵΪ1Ϊ���ع⣬����1Ϊ���ع�ģʽ��������������gui�����ã�.
			//��������� timestamp(ʱ���)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�ɼ����ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			int captureData(int exposure_num, char* timestamp)override;

			//���ܣ� ��ȡ���ͼ
			//�����������
			//��������� depth(���ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��. 
			int getDepthData(float* depth)override;
			
			//���ܣ� ��ȡ����
			//�����������
			//��������� point_cloud(����)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			int getPointcloudData(float* point_cloud)override;

			//���ܣ� ��ȡ����ͼ
			//�����������
			//��������� brightness(����ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			int getBrightnessData(unsigned char* brightness)override;

			//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
			//�����������  
			//��������� height_map(�߶�ӳ��ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			int getHeightMapData(float* height_map)override;

			//���ܣ� ��ȡ��׼ƽ�����
			//�����������
			//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			int getStandardPlaneParam(float* R, float* T)override;

			//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
			//���������R(��ת����)��T(ƽ�ƾ���)
			//��������� height_map(�߶�ӳ��ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			int getHeightMapDataBaseParam(float* R, float* T, float* height_map)override;


			 
			//���ܣ� ��ȡ����궨����
			//��������� ��
			//��������� calibration_param������궨�����ṹ�壩
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
			int getCalibrationParam(struct CalibrationParam* calibration_param)override;
 
			/***************************************************************************************************************************************************************/
			//��������

			//���ܣ� ����LED����
			//��������� led������ֵ��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamLedCurrent(int led)override;

			//���ܣ� ����LED����
			//��������� ��
			//��������� led������ֵ��
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamLedCurrent(int& led)override;
			 
			//���ܣ� ���û�׼ƽ������
			//���������R(��ת����3*3)��T(ƽ�ƾ���3*1)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamStandardPlaneExternal(float* R, float* T)override;

			//���ܣ� ��ȡ��׼ƽ������
			//�����������
			//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamStandardPlaneExternal(float* R, float* T)override;

			//���ܣ� ������������ͼ����
			//���������model(1:������ͼͬ�������ع⡢2�����������ع⡢3�������ⵥ���ع�)��exposure(����ͼ�ع�ʱ��)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamGenerateBrightness(int model, float exposure)override;

			//���ܣ� ��ȡ��������ͼ����
			//��������� ��
			//���������model(1:������ͼͬ�������ع⡢2�����������ع⡢3�������ⵥ���ع�)��exposure(����ͼ�ع�ʱ��)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamGenerateBrightness(int& model, float& exposure)override;

			//���ܣ� ��������ع�ʱ��
			//���������exposure(����ع�ʱ��)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamCameraExposure(float exposure)override;

			//���ܣ� ��ȡ����ع�ʱ��
			//��������� ��
			//���������exposure(����ع�ʱ��)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamCameraExposure(float& exposure)override;

			//���ܣ� ���û�϶��ع����������ع����Ϊ6�Σ�
			//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч����led_param[6]��6��led���Ȳ�����ǰnum����Ч��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamMixedHdr(int num, int exposure_param[6], int led_param[6])override;

			//���ܣ� ��ȡ��϶��ع����������ع����Ϊ6�Σ�
			//��������� ��
			//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч����led_param[6]��6��led���Ȳ�����ǰnum����Ч��
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamMixedHdr(int& num, int exposure_param[6], int led_param[6])override;

			//���ܣ� ��������ع�ʱ��
			//���������confidence(������Ŷ�)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamCameraConfidence(float confidence)override;

			//���ܣ� ��ȡ����ع�ʱ��
			//��������� ��
			//���������confidence(������Ŷ�)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamCameraConfidence(float& confidence)override;

			//���ܣ� �����������
			//���������gain(�������)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamCameraGain(float gain)override;

			//���ܣ� ��ȡ�������
			//��������� ��
			//���������gain(�������)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamCameraGain(float& gain)override;

			//���ܣ� ���õ���ƽ������
			//���������smoothing(0:�ء�1-5:ƽ���̶��ɵ͵���)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamSmoothing(int smoothing)override;

			//���ܣ� ���õ���ƽ������
			//�����������
			//���������smoothing(0:�ء�1-5:ƽ���̶��ɵ͵���)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamSmoothing(int& smoothing)override;

			//���ܣ� ���õ��ư뾶�˲�����
			//���������use(���أ�1����0��)��radius(�뾶����num����Ч�㣩
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamRadiusFilter(int use, float radius, int num)override;

			//���ܣ� ��ȡ���ư뾶�˲�����
			//�����������
			//���������use(���أ�1����0��)��radius(�뾶����num����Ч�㣩
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamRadiusFilter(int& use, float& radius, int& num)override;

			//�������� setParamDepthFilter
			//���ܣ� �������ͼ�˲�����
			//���������use(���أ�1����0��)��depth_filterthreshold(���ͼ��1000mm������˵�������ֵ)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual int setParamDepthFilter(int use, float depth_filter_threshold)override;

			//�������� getParamDepthFilter
			//���ܣ� �������ͼ�˲�����
			//���������use(���أ�1����0��)��depth_filterthreshold(���ͼ��1000mm������˵�������ֵ)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual int getParamDepthFilter(int& use, float& depth_filter_threshold)override;

			//���ܣ� ������������ֵ
			//���������threshold(��ֵ0-100)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamOutlierFilter(float threshold)override;

			//���ܣ� ��ȡ��������ֵ
			//��������� ��
			//���������threshold(��ֵ0-100)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			int getParamOutlierFilter(float& threshold)override;

			//���ܣ� ���ö��ع�ģʽ
			//��������� model(1��HDR(Ĭ��ֵ)��2���ظ��ع�)
			//�����������
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamMultipleExposureModel(int model)override;

			//���ܣ� �����ظ��ع���
			//��������� num(2-10)
			//�����������
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			int setParamRepetitionExposureNum(int num)override;


		public:


			bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation);

			int depthTransformPointcloud(float* depth_map, float* point_cloud_map);
			  
			int getFrame04(float* depth, int depth_buf_size,unsigned char* brightness, int brightness_buf_size);

			int getRepetitionFrame04(int count, float* depth, int depth_buf_size, unsigned char* brightness, int brightness_buf_size);

			int getFrameHdr(float* depth, int depth_buf_size, unsigned char* brightness, int brightness_buf_size);

			int getSystemConfigParam(struct SystemConfigParam& config_param);
 
			int setParamBilateralFilter(int use, int param_d);

			int getParamBilateralFilter(int& use, int& param_d);

			int getCalibrationParam(struct CameraCalibParam& calibration_param);

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


			std::string camera_ip_;
			int multiple_exposure_model_ = 1;
			int repetition_exposure_model_ = 2;


			std::timed_mutex command_mutex_;

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
		};
		 

	}
}
#endif
