#pragma once
#ifndef __XEMA_XCAMERA_H__
#define __XEMA_XCAMERA_H__
#include<vector>
#include<string>

extern "C" {
    namespace XEMA {

        using namespace std;
#ifdef _WIN32 
#define XEMA_API __declspec(dllexport)

#elif __linux
#define XEMA_API 
#endif
		 
#define MAX_CAM_SIZE 10

        //typedef enum CameraStateEnum
        //{
        //    CS_Ready = 0,       //����ʵ����ת����״̬
        //    CS_Connecting,      //����connect��δ���ӳɹ�ʱת����״̬
        //    CS_Connected,       //���ӳɹ���,δ��ȡ��֡����ʱת����״̬
        //    CS_Disconnected,    //��������disconnect�ҳɹ�ʱת����״̬
        //    CS_Working,         //���ӳɹ����ѻ�ȡ��֡����ʱת����״̬
        //    CS_ProcessError,    //�������sdkʧ������Ҫ�������Իָ�����ʱת����״̬(һ���ڴ������ݳ��ִ���ʱ)
        //    CS_Error            //�������sdkʧ���Ҳ���Ҫ�������Իָ�����ʱת����״̬
        //}CameraStateEnum;

        //typedef enum CameraErrorEnum
        //{
        //    CE_NONE = 0,        //�޴���
        //    CE_NOTSUPPORTED,    //��֧��
        //    CE_NOTINWORKING,    //δ������״̬
        //    CE_SDKFAILED,       //����sdkʧ��
        //    CE_JSONFORMAT,      //json��ʽ����
        //    CE_CAMNOTFOUND      //���δ�ҵ�
        //}CameraErrorEnum;

        //typedef struct CameraBaseInfo
        //{
        //    char id[50];
        //    int port;
        //    bool isIpType;
        //    bool is2D;
        //}CameraBaseInfo;
		 
            //����궨�����ṹ��
        typedef struct CalibrationParam
        {
            //����ڲ�
            float intrinsic[3 * 3];
            //������
            float extrinsic[4 * 4];
            //������䣬ֻ��ǰ5��
            float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

        }CalibrationParam;

        class XCamera
        {
        public:
            XCamera() = default;
            virtual ~XCamera() = default;
            XCamera(const XCamera&) = delete;
            XCamera& operator=(const XCamera&) = delete;
			 
			//���ܣ� �������
			//��������� camera_id�����ip��ַ��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ӳɹ�;����-1��ʾ����ʧ��.
			virtual int connect(const char* camera_id) = 0;
			 
			//���ܣ� ��ȡ����ֱ���
			//��������� ��
			//��������� width(ͼ���)��height(ͼ���)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����-1��ʾ��ȡ����ʧ��.
			virtual  int  getCameraResolution(int* width, int* height) = 0;
			 
			//���ܣ� �ɼ�һ֡���ݲ�����������״̬
			//��������� exposure_num���ع������������ֵΪ1Ϊ���ع⣬����1Ϊ���ع�ģʽ��������������gui�����ã�.
			//��������� timestamp(ʱ���)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�ɼ����ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int captureData(int exposure_num, char* timestamp) = 0;
			 
			//���ܣ� ��ȡ���ͼ
			//�����������
			//��������� depth(���ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int getDepthData(float* depth) = 0;
			
			//���ܣ� ��ȡ����
			//�����������
			//��������� point_cloud(����)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int getPointcloudData(float* point_cloud) = 0;
			 
			//���ܣ� ��ȡ����ͼ
			//�����������
			//��������� brightness(����ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int getBrightnessData(unsigned char* brightness)  = 0;
			 
			//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
			//�����������  
			//��������� height_map(�߶�ӳ��ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int getHeightMapData(float* height_map)  = 0;
			 
			//���ܣ� ��ȡ��׼ƽ�����
			//�����������
			//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int getStandardPlaneParam(float* R, float* T)  = 0;
			 
			//���ܣ� ��ȡУ������׼ƽ��ĸ߶�ӳ��ͼ
			//���������R(��ת����)��T(ƽ�ƾ���)
			//��������� height_map(�߶�ӳ��ͼ)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ���ݳɹ�;����-1��ʾ�ɼ�����ʧ��.
			virtual  int getHeightMapDataBaseParam(float* R, float* T, float* height_map)  = 0;
			  
			//���ܣ� �Ͽ��������
			//��������� camera_id�����ip��ַ��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ�Ͽ��ɹ�;����-1��ʾ�Ͽ�ʧ��.
			virtual  int disconnect(const char* camera_id) = 0;
			 
			//���ܣ� ��ȡ����궨����
			//��������� ��
			//��������� calibration_param������궨�����ṹ�壩
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
			virtual  int getCalibrationParam(struct CalibrationParam* calibration_param) = 0;


			/***************************************************************************************************************************************************************/
			//��������
			 
			//���ܣ� ����LED����
			//��������� led������ֵ��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamLedCurrent(int led) = 0;
			 
			//���ܣ� ����LED����
			//��������� ��
			//��������� led������ֵ��
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamLedCurrent(int& led) = 0;
			  
			 
			//���ܣ� ���û�׼ƽ������
			//���������R(��ת����3*3)��T(ƽ�ƾ���3*1)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamStandardPlaneExternal(float* R, float* T) = 0;
			 
			//���ܣ� ��ȡ��׼ƽ������
			//�����������
			//��������� R(��ת����3*3)��T(ƽ�ƾ���3*1)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamStandardPlaneExternal(float* R, float* T) = 0;
			 
			//���ܣ� ������������ͼ����
			//���������model(1:������ͼͬ�������ع⡢2�����������ع⡢3�������ⵥ���ع�)��exposure(����ͼ�ع�ʱ��)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamGenerateBrightness(int model, float exposure) = 0;
			 
			//���ܣ� ��ȡ��������ͼ����
			//��������� ��
			//���������model(1:������ͼͬ�������ع⡢2�����������ع⡢3�������ⵥ���ع�)��exposure(����ͼ�ع�ʱ��)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamGenerateBrightness(int& model, float& exposure) = 0;
			 
			//���ܣ� ��������ع�ʱ��
			//���������exposure(����ع�ʱ��)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamCameraExposure(float exposure) = 0;
			 
			//���ܣ� ��ȡ����ع�ʱ��
			//��������� ��
			//���������exposure(����ع�ʱ��)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamCameraExposure(float& exposure) = 0;
			 
			//���ܣ� ���û�϶��ع����������ع����Ϊ6�Σ�
			//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч����led_param[6]��6��led���Ȳ�����ǰnum����Ч��
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamMixedHdr(int num, int exposure_param[6], int led_param[6]) = 0;
			 
			//���ܣ� ��ȡ��϶��ع����������ع����Ϊ6�Σ�
			//��������� ��
			//��������� num���ع��������exposure_param[6]��6���ع������ǰnum����Ч����led_param[6]��6��led���Ȳ�����ǰnum����Ч��
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamMixedHdr(int& num, int exposure_param[6], int led_param[6]) = 0;
			 
			//���ܣ� ��������ع�ʱ��
			//���������confidence(������Ŷ�)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamCameraConfidence(float confidence) = 0;
			 
			//���ܣ� ��ȡ����ع�ʱ��
			//��������� ��
			//���������confidence(������Ŷ�)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamCameraConfidence(float& confidence) = 0;
			 
			//���ܣ� �����������
			//���������gain(�������)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamCameraGain(float gain) = 0;
			 
			//���ܣ� ��ȡ�������
			//��������� ��
			//���������gain(�������)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamCameraGain(float& gain) = 0;
			 
			//���ܣ� ���õ���ƽ������
			//���������smoothing(0:�ء�1-5:ƽ���̶��ɵ͵���)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamSmoothing(int smoothing) = 0;
			 
			//���ܣ� ���õ���ƽ������
			//�����������
			//���������smoothing(0:�ء�1-5:ƽ���̶��ɵ͵���)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamSmoothing(int& smoothing) = 0;
			 
			//���ܣ� ���õ��ư뾶�˲�����
			//���������use(���أ�1����0��)��radius(�뾶����num����Ч�㣩
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamRadiusFilter(int use, float radius, int num) = 0;
			 
			//���ܣ� ��ȡ���ư뾶�˲�����
			//�����������
			//���������use(���أ�1����0��)��radius(�뾶����num����Ч�㣩
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamRadiusFilter(int& use, float& radius, int& num) = 0;
			 
			//���ܣ� ������������ֵ
			//���������threshold(��ֵ0-100)
			//��������� ��
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamOutlierFilter(float threshold) = 0;
			 
			//���ܣ� ��ȡ��������ֵ
			//��������� ��
			//���������threshold(��ֵ0-100)
			//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�����ɹ�;����ʧ�ܡ�
			virtual  int getParamOutlierFilter(float& threshold) = 0;
			 
			//���ܣ� ���ö��ع�ģʽ
			//��������� model(1��HDR(Ĭ��ֵ)��2���ظ��ع�)
			//�����������
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamMultipleExposureModel(int model) = 0;
			 
			//���ܣ� �����ظ��ع���
			//��������� num(2-10)
			//�����������
			//����ֵ�� ���ͣ�int��:����0��ʾ���ò����ɹ�;����ʧ�ܡ�
			virtual  int setParamRepetitionExposureNum(int num) = 0;
        };

        XEMA_API void* createCamera();
        XEMA_API void destroyCamera(void*);

    }
}
#endif