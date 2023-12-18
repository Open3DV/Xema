#pragma once

#include "iostream"
#include "encode.h" 
#include <opencv2/highgui.hpp>
#include "reconstruct.h" 
#include "support_function.h"

//struct _finddata_t
//{
//	unsigned attrib;
//	time_t time_create;
//	time_t time_access;
//	time_t time_write;
//	_fsize_t size;
//	char name[_MAX_FNAME];
//};

class DfSolution
{

public:

	DfSolution();
	~DfSolution();

	bool reconstructMixedVariableWavelengthPatternsBaseXYSR(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructMixedVariableWavelengthXPatternsBaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructMixedVariableWavelengthXPatternsBaseTableAndConfidence(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");


	bool reconstructPatterns04RepetitionBaseTable(std::vector <std::vector<cv::Mat>> patterns_list, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructPatterns04Repetition01BaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructBasePhase02(cv::Mat phase_x, cv::Mat phase_y, cv::Mat brightness, struct CameraCalibParam calib_param, std::string pointcloud_path);

	bool reconstructPatterns06BaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructPatterns05(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructPatterns06(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructPatterns0602(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructPatternsTest(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool reconstructPatterns08(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./"); 

	bool reconstructMixedVariableWavelengthXPatternsBaseMiniTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path = "./");

	bool testCalibrationParamBasePlane(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path = "./");

	bool testCalibrationParamBaseBoard(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path = "./");

	bool readCameraCalibData(std::string path, struct CameraCalibParam& param);

	bool getCameraCalibData(std::string ip, struct CameraCalibParam& param);

	std::string& replaceAll(std::string& str, const   std::string& old_value, const   std::string& new_value);

	bool savePatterns(std::string dir, std::vector<cv::Mat> patterns);

	bool saveCameraCalibData(std::string path, struct CameraCalibParam& param);

	bool captureMixedVariableWavelengthPatterns(std::string ip, std::vector<cv::Mat>& patterns);

	bool captureModel04Patterns(std::string ip, std::vector<cv::Mat>& patterns);

	bool captureModel04RepetitionPatterns(std::string ip, int repetition_count, std::vector<cv::Mat>& patterns);

	void getFiles(std::string path, std::vector<std::string>& files);

	bool readFolderImages(std::string dir, std::vector<std::vector<cv::Mat>>& patterns_list);

	bool readImages(std::string dir, std::vector<cv::Mat>& patterns);

	bool readColorImages(std::string dir, std::vector<cv::Mat>& patterns);

	bool setProjectorVersion(int version);

	bool findMaskBaseConfidence(cv::Mat confidence_map, int threshold, cv::Mat& mask);

	bool findMaskBaseConfidenceLocalGrads(cv::Mat confidence_map, float threshold, cv::Mat& mask);

	bool test_calib_shake();

public:
	int confidence_level_ = 50;

private:
	int projector_version_;

};

