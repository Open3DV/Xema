#pragma once

#include <QWidget>
#include "ui_camera_capture_gui.h"
#include "settings_file_function.h"
#include <opencv2/core.hpp>
#include "../sdk/open_cam3d.h"
#include "../sdk/xema_enums.h"
#include "../calibration/calibrate_function.h"
//#include "../firmware/system_config_settings.h"
#include "../firmware/protocol.h"
#include <QThread>
#include <QDebug>
#include <QtCore/QTimer>
#include "../firmware/version.h"
#include "save_gui.h"

const int  SELECT_BRIGHTNESS_FLAG_ = 1;
const int  SELECT_HEIGHT_MAP_FLAG_ = 2;
const int  SELECT_COLOR_DEPTH_FLAG_ = 3;

const int  GENERATE_BRIGHTNESS_DEFAULT_  = 1;
const int  GENERATE_BRIGHTNESS_ILLUMINATION_ =  2;
const int  GENERATE_BRIGHTNESS_DARKNESS_ =  3;
//const int  GENERATE_BRIGHTNESS_HDR_ = 4;

const int  GENERATE_BRIGHTNESS_MODEL_SINGLE_ = 1;
const int  GENERATE_BRIGHTNESS_MODEL_HDR_ = 2;

const int EXPOSURE_MODEL_SINGLE_ = 0;
const int EXPOSURE_MODEL_HDR_ = 1;
const int EXPOSURE_MODEL_REPETITION_ = 2;


//const int EXPOSURE_TIME_MIN_ = 1700;
//const int EXPOSURE_TIME_MAX_ = 100000;

class CameraCaptureGui : public QWidget
{
	Q_OBJECT

public:
	CameraCaptureGui(QWidget* parent = Q_NULLPTR);
	~CameraCaptureGui();
	void updateLanguage();

	void getFirmwareVersion(QString& version);

	void getSdkVersionNumber(QString& version);

	void getProductInfo(QString& info);

	void setOnDrop(int (*p_function)(void*));

	bool setShowImages(cv::Mat brightness, cv::Mat depth);

	void setGuiSettingData(GuiConfigDataStruct& settings_data_);

	bool saveSettingData(QString path);

	bool loadSettingData(QString path);

	void setUiData();

	bool connectCamera(QString ip);

	bool stopCapturingOneFrameBaseThread();

	void captureOneFrameBaseThread(bool hdr);

	bool captureOneFrameData();

	bool captureOneFrameAndRender();

	bool captureBrightness();

	bool initializeFunction();

	bool saveOneFrameData(QString path_name);

	void addLogMessage(QString str);
	 
	//更新多曝光参数
	void updateManyExposureParam();

	void updateBrightnessHdrParam();

	bool manyExposureParamHasChanged();

	bool brightnessHdrParamHasChanged();

	bool isConnect();

	bool setCameraConfigParam();

	bool getCameraConfigParam();

	void sleep(int sectime);

	bool getShowCalibrationMessage(struct SystemConfigParam& config_param, struct CameraCalibParam& calibration_param);
 
	void setGuiConfigParam(GuiConfigDataStruct param);

	void getGuiConfigParam(struct GuiConfigDataStruct& gui_param);

	void getFirmwareConfigParam(struct FirmwareConfigParam& param); 

	void updateOutlierRemovalConfigParam(struct FirmwareConfigParam param);

	//更新生成亮度图参数
	void updateGenerateBrightnessParam();

	void setCalibrationBoard(int flag);

	void getCameraIp(QString& ip);

	void hideLoadingForm();

	void showLoadingForm();

	void updateDepthFilter(int use, float threshold);

	void updateRadiusFilter();

	void updateReflectFilter();

	void updateRectifyGray(int use, int r, float s);
private:
	bool showImage();

	bool setImage(cv::Mat img);

	bool renderHeightImage(cv::Mat height);

	bool renderDepthImage(cv::Mat depth);

	bool renderBrightnessImage(cv::Mat brightness);

	bool renderColorBrightnessImage(cv::Mat brightness);

	void undateSystemConfigUiData();

	double computePointsDistance(cv::Point2f p_0, cv::Point2f p_1, cv::Mat point_cloud);

	bool bilinearInterpolationFeaturePoints(std::vector<cv::Point2f> feature_points, std::vector<cv::Point3f>& point_3d, cv::Mat point_cloud);


signals:
	void send_temperature_update(float val);

	void send_images_update();

	void send_log_update(QString str);

	void send_update_language(QString val);

public slots:

	void do_timeout_capture_slot();

	void do_undate_show_slot();

	void do_pushButton_connect();

	void do_pushButton_connect_async();

	void do_pushButton_disconnect();

	void do_pushButton_refresh();

	void do_handleLogMessage(QString str);

private slots:
	void do_QRadioButton_toggled_brightness(bool state);

	void do_QRadioButton_toggled_color_depth(bool state);

	void do_QRadioButton_toggled_color_height(bool state);

	void do_QRadioButton_toggled_signal(bool state);

	void do_QRadioButton_toggled_hdr(bool state);

	void do_QRadioButton_toggled_repetition(bool state); 

	void do_QRadioButton_toggled_generate_brightness_default(bool state);

	void do_QRadioButton_toggled_generate_brightness_illumination(bool state);

	void do_QRadioButton_toggled_generate_brightness_darkness(bool state);

	void do_QRadioButton_toggled_generate_brightness_single(bool state);

	void do_QRadioButton_toggled_generate_brightness_hdr(bool state);

	void add_exposure_item(int row, int exposure, int led);

	bool remove_exposure_item(int row);

	double get_exposure_item_value(int row);

	void add_brightness_hdr_exopsure_item(int row, int exposure, float gain);

	bool remove_brightness_hdr_exposure_item(int row);

	

private slots:
	//void do_checkBox_toggled_bilateral_filter(bool state);
	void do_comboBox_activated_ip(int index);

	void do_comboBox_activated_engine(int index);

	void do_checkBox_toggled_hdr(bool state);

	void do_checkBox_toggled_auto_save(bool state);

	void do_checkBox_toggled_rectify_phase_base_gray(bool state);

	void do_checkBox_toggled_depth_filter(bool state);

	void do_checkBox_toggled_radius_filter(bool state);

	void do_checkBox_toggled_reflect_filter(bool state);

	void do_checkBox_toggled_generate_brightness(bool state);

	void do_checkBox_toggled_over_exposure(bool state);


	void do_spin_smoothing_changed(int val);

	void do_spin_exposure_num_changed(int val);

	void do_spin_brightness_hdr_num_changed(int val);

	void do_more_exposure_param_changed(int val);

	void do_brightness_hdr_param_changed(int val);

	void do_spin_repetition_count_changed(int val);

	void do_spin_min_z_changed(double val);

	void do_spin_max_z_changed(double val);

	void do_doubleSpin_gain(double val);

	void do_doubleSpin_confidence(double val);

	void do_doubleSpin_fisher(double val);

	void do_doubleSpin_depth_fisher(double val);

	void do_spin_gray_rectify_r(int val);

	void do_doubleSpin_gray_rectify_s(double val);

	void do_pushButton_capture_one_frame();

	void do_pushButton_test_accuracy();

	void do_pushButton_calibrate_external_param();

	void do_pushButton_capture_continuous();

	void do_spin_led_current_changed(int val);

	void do_spin_camera_exposure_changed(int val);

	void do_spin_camera_exposure_define_changed(int val);

	void do_spin_brightness_gain_changed(double val);

	void do_spin_generate_brightness_exposure_changed(int val);

	void do_double_spin_radius_filter_r_changed(double val);

	void do_spin_radius_filter_num_changed(int val); 

	void do_double_spin_reflect_filter_b_changed(double val);

	void do_spin_reflect_filter_threshold_changed(int val);
	/******************************************************************************************/

	void do_pushButton_save_as();

	void do_pushButton_open_folder();
	
protected:
	virtual void resizeEvent(QResizeEvent* event) override;
		 
private:
	Ui::CameraCaptureGui ui;

	QWidget* m_pMaskLayer = nullptr;

	bool capture_show_flag_;

	std::vector<QString> device_mac_list_;
	std::vector<QString> device_ip_list_;

	//std::mutex	capture_m_mutex_;
	bool capturing_flag_;
	bool camera_setting_flag_;

	//6个exposure输入框
	std::vector<QSpinBox*> exposure_time_list_;
	std::vector<QSpinBox*> led_current_list_;


	//10个exposure输入框
	std::vector<QSpinBox*> brightness_hdr_exposure_time_list_;
	//std::vector<QDoubleSpinBox*> brightness_hdr_gain_list_;

	int radio_button_flag_;

	cv::Mat depth_map_;
	cv::Mat pointcloud_map_;
	cv::Mat brightness_map_;
	cv::Mat height_map_;
	cv::Mat undistort_depth_map_;
	cv::Mat undistort_brightness_map_;
	cv::Mat undistort_color_brightness_map_;
	cv::Mat undistort_pointcloud_map_;
	cv::Mat render_image_brightness_;
	cv::Mat render_image_gray_depth_;
	cv::Mat render_image_color_depth_;
	cv::Mat render_image_color_height_;
	cv::Mat color_brightness_map_; 

	int min_depth_value_;
	int max_depth_value_;


	//Camera 
	bool connected_flag_;
	int camera_width_;
	int camera_height_;


	CalibrationParam calib_param_;
	GuiConfigDataStruct processing_gui_settings_data_;
	SettingsFileFunction config_system_param_machine_;

	//相机系统配置参数
	struct SystemConfigParam system_config_param_;
	struct FirmwareConfigParam firmware_config_param_;
	//相机标定参数
	struct CameraCalibParam camera_calibration_param_;

	QString last_path_;
	QString sys_path_;

	QThread* capture_thread_;

	bool start_timer_flag_;
	QTimer capture_timer_;
	QString camera_ip_;

	std::vector<cv::Point3f> center_points_list_;
	std::vector<float> rms_list_;
	std::vector<std::vector<float>> plane_list_;
	std::vector<std::vector<cv::Point3f>> feature_points_list_;

	//生成亮度图模式
	//int generate_brightness_model_;
	//float generate_brightness_exposure_;
	//float generate_brightness_gain_;

	//cv::Size2f board_size_;
	struct BoardMessage board_message_;

	int calibration_board_flag_;
	int camera_version_;


	int (*m_p_OnDropped_)(void*);


	int exposure_model_;

	int exposure_time_min_ = 1700;
	int exposure_time_max_ = 100000;

	QString firmware_version_;
	char info_[INFO_SIZE] = {'\0'};
	QString firmware_v_num_ = "";
	QString sdk_v_num_ = "";

	SaveDataType save_data_type_ = SaveDataType::Origin;
	bool hide_save_gui_flag_ = false;

	std::mutex mtx_save_;

	XemaPixelType pixel_type_ = XemaPixelType::Mono;

	
};
