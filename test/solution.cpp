#ifdef _WIN32  
#include <windows.h>
#include <io.h>
#elif __linux 
#include <cstring>
#include "iostream" 
#include <fstream> 
#include <sys/types.h>
#include <dirent.h>
#include <sys/io.h>
#endif 

#include "solution.h" 
#include "support_function.h" 
#include <iostream>  
#include "../sdk/open_cam3d.h"
#include <assert.h>
#include <fstream>
#include <iomanip>
#include "../firmware/protocol.h"
#include "AnalyseError.h"
#include "../calibration/calibrate_function.h" 
#include "../gui/PrecisionTest.h"
#include "LookupTableFunction.h"
//#include "../cmd/getopt.h" 
#include "FilterModule.h" 
#include <opencv2/photo/photo.hpp>
#include "../firmware/easylogging++.h"
/**************************************************************************/

DfSolution::DfSolution()
{
	projector_version_ = 3010;
}

DfSolution::~DfSolution()
{

}


bool DfSolution::setProjectorVersion(int version)
{
	projector_version_ = version;

	switch (version)
	{
	case DF_PROJECTOR_3010:
	{
		return true;
	}
	break;

	case DF_PROJECTOR_4710:
	{
		return true;
	}
	break;

	default:
		break;
	}

	return false;
}

int on_dropped_solution(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}

std::string& DfSolution::replaceAll(std::string& str, const   std::string& old_value, const   std::string& new_value)
{
	while (true) {
		std::string::size_type   pos(0);
		if ((pos = str.find(old_value)) != std::string::npos)
			str.replace(pos, old_value.length(), new_value);
		else   break;
	}
	return   str;
}

bool DfSolution::savePatterns(std::string dir, std::vector<cv::Mat> patterns)
{
	std::string new_dir = replaceAll(dir, "/", "\\");

	std::string folderPath = new_dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());


	//std::cout << mkdir_cmd << std::endl;

	for (int i = 0; i < patterns.size(); i++)
	{
		std::stringstream ss;
		cv::Mat image = patterns[i];
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "\\pattern_" + ss.str() + ".bmp";
		bool ret = cv::imwrite(filename, image);
		std::cout << "save: " << filename << " " << ret << std::endl;
	}

	return true;
}


bool DfSolution::saveCameraCalibData(std::string path, struct CameraCalibParam& param)
{
	std::ofstream ofile;
	ofile.open(path);
	for (int i = 0; i < sizeof(param) / sizeof(float); i++)
	{
		ofile << ((float*)(&param))[i] << std::endl;
	}
	ofile.close();

	return true;
}

bool DfSolution::readCameraCalibData(std::string path, struct CameraCalibParam& param)
{
	std::ifstream myfile(path);

	if (!myfile.is_open())
	{
		std::cout << "can not open this file" << std::endl;
		return 0;
	}

	float I[40] = { 0 };


	for (int i = 0; i < 40; i++)
	{

		myfile >> I[i];
		//std::cout << I[i] << std::endl;

	}
	myfile.close();


	param.camera_intrinsic[0] = I[0];
	param.camera_intrinsic[1] = I[1];
	param.camera_intrinsic[2] = I[2];
	param.camera_intrinsic[3] = I[3];
	param.camera_intrinsic[4] = I[4];
	param.camera_intrinsic[5] = I[5];
	param.camera_intrinsic[6] = I[6];
	param.camera_intrinsic[7] = I[7];
	param.camera_intrinsic[8] = I[8];

	param.camera_distortion[0] = I[9];
	param.camera_distortion[1] = I[10];
	param.camera_distortion[2] = I[11];
	param.camera_distortion[3] = I[12];
	param.camera_distortion[4] = I[13];


	param.projector_intrinsic[0] = I[14];
	param.projector_intrinsic[1] = I[15];
	param.projector_intrinsic[2] = I[16];
	param.projector_intrinsic[3] = I[17];
	param.projector_intrinsic[4] = I[18];
	param.projector_intrinsic[5] = I[19];
	param.projector_intrinsic[6] = I[20];
	param.projector_intrinsic[7] = I[21];
	param.projector_intrinsic[8] = I[22];


	param.projector_distortion[0] = I[23];
	param.projector_distortion[1] = I[24];
	param.projector_distortion[2] = I[25];
	param.projector_distortion[3] = I[26];
	param.projector_distortion[4] = I[27];


	param.rotation_matrix[0] = I[28];
	param.rotation_matrix[1] = I[29];
	param.rotation_matrix[2] = I[30];
	param.rotation_matrix[3] = I[31];
	param.rotation_matrix[4] = I[32];
	param.rotation_matrix[5] = I[33];
	param.rotation_matrix[6] = I[34];
	param.rotation_matrix[7] = I[35];
	param.rotation_matrix[8] = I[36];


	param.translation_matrix[0] = I[37];
	param.translation_matrix[1] = I[38];
	param.translation_matrix[2] = I[39];



	return true;
}




bool DfSolution::getCameraCalibData(std::string ip, struct CameraCalibParam& param)
{
	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	//struct CameraCalibParam calibration_param;
	//DfGetCalibrationParam(calibration_param);

	ret = DfGetCalibrationParam(param);


	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfDisconnectNet();

	return true;
}


bool DfSolution::captureModel04RepetitionPatterns(std::string ip, int repetition_count, std::vector<cv::Mat>& patterns)
{
	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int capture_num = 19 + 6 * (repetition_count - 1);

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData04Repetition(raw_buf, image_size * capture_num, repetition_count);


	patterns.clear();

	for (int i = 0; i < capture_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(1200, 1920, CV_8UC1, raw_buf + (long)(image_size * i));

		patterns.push_back(image.clone());
	}

	delete[] raw_buf;

	DfDisconnectNet();

	return true;
}

bool DfSolution::captureModel04Patterns(std::string ip, std::vector<cv::Mat>& patterns)
{
	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int capture_num = 19;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData04(raw_buf, image_size * capture_num);


	patterns.clear();

	for (int i = 0; i < capture_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(1200, 1920, CV_8UC1, raw_buf + (long)(image_size * i));

		patterns.push_back(image.clone());
	}

	delete[] raw_buf;

	DfDisconnectNet();

	return true;
}

bool DfSolution::captureMixedVariableWavelengthPatterns(std::string ip, std::vector<cv::Mat>& patterns)
{
	DfRegisterOnDropped(on_dropped_solution);

	int ret = DfConnectNet(ip.c_str());
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int capture_num = 31;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData03(raw_buf, image_size * capture_num);


	patterns.clear();

	for (int i = 0; i < capture_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(1200, 1920, CV_8UC1, raw_buf + (long)(image_size * i));

		patterns.push_back(image.clone());
	}

	delete[] raw_buf;

	DfDisconnectNet();

	return true;
}

/**************************************************************************/


void  DfSolution::getFiles(std::string path, std::vector<std::string>& files)
{

#ifdef _WIN32 

	intptr_t    hFile = 0;

	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*.bmp").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				//	getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
 
			else
			{
				files.push_back(p.assign(path).append("/").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);

		_findclose(hFile);
	}


#elif __linux
	DIR* pDir;
	struct dirent* ptr;
	if (!(pDir = opendir(path.c_str())))
		return;
	while ((ptr = readdir(pDir)) != 0) {
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
			files.push_back(path + "/" + ptr->d_name);
	}
	closedir(pDir);

#endif 


}


bool DfSolution::readFolderImages(std::string dir, std::vector<std::vector<cv::Mat>>& patterns_list)
{
	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(dir, files_list);

	for (int l_i = 0; l_i < files_list.size(); l_i++)
	{
		std::vector<std::string> files = files_list[l_i];
		std::vector<cv::Mat> patterns;

		for (int p_i = 0; p_i < files.size(); p_i++)
		{
			cv::Mat img = cv::imread(files[p_i], 0);

			patterns.push_back(img.clone());
			std::cout << "read: " << files[p_i] << std::endl;
		}

		patterns_list.push_back(patterns);
	}

	return true;
}


bool DfSolution::readColorImages(std::string dir, std::vector<cv::Mat>& patterns)
{
	//if (dir.empty())
	//{
	//	return false;
	//}

	std::vector<std::string> files;

	getFiles(dir, files);

	//for (int i = 0; i < files.size(); i++)
	//{
	//	std::cout<< files[i].c_str() << std::endl;
	//}

	patterns.clear();

	for (int i = 0; i < files.size(); i++)
	{
		//std::string path = dir + "/phase";
		std::string path = files[i];
		//if (i < 10)
		//{
		//	path += "0";
		//}

		//path +=  std::to_string(i) + "_p.bmp";



		cv::Mat img = cv::imread(path, 0);
		if (img.empty())
		{
			return false;
		}
		std::cout << path << std::endl;


		cv::Mat color_mat;
		LOG(INFO) << "start bayer 2 rgb cv";
		cv::cvtColor(img, color_mat, cv::COLOR_BayerBG2BGR);
		LOG(INFO) << "finished bayer 2 rgb cv";

		std::vector < cv::Mat> channels;
		cv::split(color_mat, channels);
		cv::Mat b = channels[0].clone();

		cv::Mat test_rgb(img.rows, img.cols, CV_8UC3,cv::Scalar(0));

		LOG(INFO) << "start bayer 2 rgb";

		bayerBg2Rgb(img.cols, img.rows,img.data, test_rgb.data);
		LOG(INFO) << "finished bayer 2 rgb";

		cv::Mat diff = test_rgb - color_mat;

		//cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 1);

		if (files.size() - 1 == i)
		{
			patterns.push_back(color_mat.clone());
		}
		else
		{
			patterns.push_back(b.clone());
		}


		//patterns[i] = img.clone();
	}



	return true;
}

bool DfSolution::readImages(std::string dir, std::vector<cv::Mat>& patterns)
{
	//if (dir.empty())
	//{
	//	return false;
	//}

	std::vector<std::string> files;

	getFiles(dir, files);

	//for (int i = 0; i < files.size(); i++)
	//{
	//	std::cout<< files[i].c_str() << std::endl;
	//}

	patterns.clear();

	for (int i = 0; i < files.size(); i++)
	{
		//std::string path = dir + "/phase";
		std::string path = files[i];
		//if (i < 10)
		//{
		//	path += "0";
		//}

		//path +=  std::to_string(i) + "_p.bmp";



		cv::Mat img = cv::imread(path, 0);
		if (img.empty())
		{
			return false;
		}
		std::cout << path << std::endl;


		//cv::GaussianBlur(img, img, cv::Size(5, 5), 0, 1);

		patterns.push_back(img.clone());

		//patterns[i] = img.clone();
	}



	return true;
}

bool DfSolution::testCalibrationParamBaseBoard(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path)
{


	if (31 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;
	int hor_patterns_num = 12;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);
	std::vector<cv::Mat> hor_patterns_img(patterns.begin() + ver_pstterns_num, patterns.end() - 1);

	std::vector<cv::Mat> ver_wrap_img_4;
	std::vector<cv::Mat> hor_wrap_img_4;
	cv::Mat ver_confidence_map_4;
	cv::Mat hor_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	std::vector<cv::Mat> hor_wrap_img_6;
	cv::Mat ver_confidence_map_6;
	cv::Mat hor_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> hor_patterns_img_4(hor_patterns_img.begin(), hor_patterns_img.begin() + hor_patterns_num);

	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);


	DF_Encode encode_machine_;
	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();


	int discard_num = 0;

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_hor, unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}

	variable_wrap_rate.pop_back();

	float hor_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		hor_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}



	float confidence_val = 10;

	float ver_period = ver_period_num;
	float hor_period = hor_period_num * 720.0 / 1280.0;


	unwrap_ver /= ver_period;
	unwrap_hor /= hor_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);


	encode_machine_.maskMap(unwrap_mask, unwrap_ver);
	encode_machine_.maskMap(unwrap_mask, unwrap_hor);



	cv::Mat deep_map;
	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);

	cv::Mat err_map;

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	cv::Mat texture_map = patterns[30];
	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();


	/**************************************************************************************************/
		//ICP 
	Calibrate_Function calib_function;
	std::vector<cv::Point2f> undist_circle_points;
	bool found = calib_function.findCircleBoardFeature(undistort_img, undist_circle_points);

	if (!found)
	{
		return false;
	}



	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map, deep_channels);
	cv::Mat depth_map;
	deep_channels[2].convertTo(depth_map, CV_32F);
	cv::medianBlur(depth_map, depth_map, 3);


	cv::Mat points_map(depth_map.size(), CV_32FC3, cv::Scalar(0., 0., 0.));
	reconstruct_machine_.depthTransformPointcloud(depth_map, points_map);

	/*******************************************************************************************/
	std::vector<cv::Point3f> point_3d;
	calib_function.bilinearInterpolationFeaturePoints(undist_circle_points, point_3d, points_map);

	PrecisionTest precision_machine;
	cv::Mat pc1(point_3d.size(), 3, CV_64F, cv::Scalar(0));
	cv::Mat pc2(point_3d.size(), 3, CV_64F, cv::Scalar(0));

	std::vector<cv::Point3f> world_points = calib_function.generateAsymmetricWorldFeature(20, 10);

	for (int i = 0; i < point_3d.size(); i++)
	{
		pc2.at<double>(i, 0) = point_3d[i].x;
		pc2.at<double>(i, 1) = point_3d[i].y;
		pc2.at<double>(i, 2) = point_3d[i].z;
	}
	for (int i = 0; i < world_points.size(); i++)
	{
		pc1.at<double>(i, 0) = world_points[i].x;
		pc1.at<double>(i, 1) = world_points[i].y;
		pc1.at<double>(i, 2) = world_points[i].z;
	}

	cv::Mat r(3, 3, CV_64F, cv::Scalar(0));
	cv::Mat t(3, 3, CV_64F, cv::Scalar(0));

	precision_machine.svdIcp(pc1, pc2, r, t);

	std::vector<cv::Point3f> transform_points;

	precision_machine.transformPoints(point_3d, transform_points, r, t);

	double diff = precision_machine.computeTwoPointSetDistance(world_points, transform_points);

	std::cout << "重投影误差: " << diff << " mm" << std::endl;

	if (diff > 0.1)
	{
		std::cout << "误差过大！" << std::endl;

		if (0 == calib_function.testOverExposure(undistort_img, undist_circle_points))
		{
			std::cout << "图像过曝！" << std::endl;
			std::cout << "请重新拍摄！" << std::endl;
		}
	}
	else
	{
		std::cout << "精度合格！" << std::endl;
	}
	/****************************************************************************************************/


	cv::Mat draw_color_img;
	cv::Size board_size = calib_function.getBoardSize();
	cv::cvtColor(undistort_img, draw_color_img, cv::COLOR_GRAY2BGR);
	cv::drawChessboardCorners(draw_color_img, board_size, undist_circle_points, found);


	cv::Mat render_brightness;
	renderBrightnessImage(texture_map, render_brightness);
	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = err_map_path + "/test_calibration_param";


	std::string save_draw_board_dir = work_path_ + "_draw_board.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness.bmp";


	cv::imwrite(save_brightness_dir, render_brightness);
	cv::imwrite(save_draw_board_dir, draw_color_img);

	std::cout << "save image: " << save_brightness_dir << std::endl;
	std::cout << "save image: " << save_draw_board_dir << std::endl;

	return true;

}


bool DfSolution::testCalibrationParamBasePlane(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string err_map_path)
{

	if (31 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;
	int hor_patterns_num = 12;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);
	std::vector<cv::Mat> hor_patterns_img(patterns.begin() + ver_pstterns_num, patterns.end() - 1);

	std::vector<cv::Mat> ver_wrap_img_4;
	std::vector<cv::Mat> hor_wrap_img_4;
	cv::Mat ver_confidence_map_4;
	cv::Mat hor_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	std::vector<cv::Mat> hor_wrap_img_6;
	cv::Mat ver_confidence_map_6;
	cv::Mat hor_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> hor_patterns_img_4(hor_patterns_img.begin(), hor_patterns_img.begin() + hor_patterns_num);

	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);


	DF_Encode encode_machine_;
	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();


	int discard_num = 0;

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_hor, unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}

	variable_wrap_rate.pop_back();

	float hor_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		hor_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}



	float confidence_val = 10;

	float ver_period = ver_period_num;
	float hor_period = hor_period_num * 720.0 / 1280.0;


	unwrap_ver /= ver_period;
	unwrap_hor /= hor_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);


	encode_machine_.maskMap(unwrap_mask, unwrap_ver);
	encode_machine_.maskMap(unwrap_mask, unwrap_hor);



	cv::Mat deep_map;
	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);

	cv::Mat err_map;

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	cv::Mat color_err_map;
	cv::Mat gray_err_map;
	renderErrorMap(err_map, color_err_map, gray_err_map, 0., 0.1);

	AnalyseError analyse_err_machine;

	double err_value = analyse_err_machine.computeError(err_map);
	//std::cout << "calibrate err: " << err_value << std::endl;

	std::cout << "calibrate err: " << err_value << " mm" << std::endl;

	if (err_value > 0.1)
	{
		std::cout << "精度不合格！" << std::endl;
	}
	else
	{
		std::cout << "精度合格！" << std::endl;

	}


	cv::Mat texture_map = patterns[30];

	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat render_brightness;
	renderBrightnessImage(texture_map, render_brightness);

	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = err_map_path + "/test_calibration_param";


	std::string save_err_tiff = work_path_ + "_err.tiff";
	std::string save_color_err_tiff = work_path_ + "_color_err.tiff";
	std::string save_brightness_dir = work_path_ + "_brightness.bmp";



	cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_brightness_dir, render_brightness);
	cv::imwrite(save_color_err_tiff, gray_err_map);

	std::cout << "save image: " << save_err_tiff << std::endl;
	std::cout << "save image: " << save_brightness_dir << std::endl;
	std::cout << "save image: " << save_color_err_tiff << std::endl;

	return true;
}


bool DfSolution::findMaskBaseConfidenceLocalGrads(cv::Mat confidence_map, float threshold, cv::Mat& mask)
{
	if (confidence_map.empty())
	{
		return true;
	}

	int nr = confidence_map.rows;
	int nc = confidence_map.cols;

	cv::Mat sobel_map;

	cv::Sobel(confidence_map, sobel_map, -1, 1, 1, 3);


	cv::Mat bin_map;

	cv::threshold(confidence_map, bin_map, threshold, 255, cv::THRESH_BINARY);
	//bin_map.convertTo(bin_map, CV_8UC1);
}

bool  DfSolution::findMaskBaseConfidence(cv::Mat confidence_map, int threshold, cv::Mat& mask)
{
	if (confidence_map.empty())
	{
		return true;
	}

	int nr = confidence_map.rows;
	int nc = confidence_map.cols;


	cv::Mat bin_map;

	cv::threshold(confidence_map, bin_map, threshold, 255, cv::THRESH_BINARY);
	bin_map.convertTo(bin_map, CV_8UC1);

	std::vector<std::vector<cv::Point>> contours;

	cv::findContours(
		bin_map,
		contours,
		cv::noArray(),
		cv::RETR_EXTERNAL,
		cv::CHAIN_APPROX_SIMPLE
	);

	std::vector<cv::Point> max_contours;
	int max_contours_size = 0;

	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > max_contours_size)
		{
			max_contours_size = contours[i].size();
			max_contours = contours[i];
		}

	}

	contours.clear();
	contours.push_back(max_contours);

	cv::Mat show_contours(nr, nc, CV_8U, cv::Scalar(0));
	cv::drawContours(show_contours, contours, -1, cv::Scalar(255), -1);

	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::Mat result;
	cv::erode(show_contours, result, element);

	mask = result.clone();


	return true;
}



bool DfSolution::reconstructPatterns04Repetition01BaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	MiniLookupTableFunction mini_lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	mini_lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	mini_lookup_table_machine_.setProjectorVersion(projector_version_);
	 
	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;

	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);
	mini_lookup_table_machine_.generateBigLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping, pattern_minimapping);
	lookup_table_machine_.setLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);


	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/************************************************************************************/

	std::vector<cv::Mat> six_shift_patterns;

	int repetition_count = (patterns.size() - 13) / 6;

	if (patterns.size() != 13 + 6 * repetition_count)
	{
		return false;
	}

	std::vector<std::vector<cv::Mat>> six_shift_patterns_list;

	for (int i = 0; i < repetition_count; i++)
	{
		std::vector<cv::Mat> six_patterns;
		six_patterns.push_back(patterns[12 + 6 * i + 0]);
		six_patterns.push_back(patterns[12 + 6 * i + 1]);
		six_patterns.push_back(patterns[12 + 6 * i + 2]);
		six_patterns.push_back(patterns[12 + 6 * i + 3]);
		six_patterns.push_back(patterns[12 + 6 * i + 4]);
		six_patterns.push_back(patterns[12 + 6 * i + 5]);

		six_shift_patterns_list.push_back(six_patterns);
	}

	DF_Encode encode_machine_;
	std::vector<cv::Mat> ushort_patterns;
	encode_machine_.mergePatterns(six_shift_patterns_list, ushort_patterns);

	std::vector <cv::Mat> mixed_patterns(patterns.begin(), patterns.begin() + 12);
	mixed_patterns.insert(mixed_patterns.end(), ushort_patterns.begin(), ushort_patterns.end());
	mixed_patterns.push_back(patterns.back());

	patterns.clear();
	patterns = mixed_patterns;

	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);




	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}
	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();

	float confidence_val = 5;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	encode_machine_.maskMap(unwrap_mask, unwrap_ver);

	cv::Mat confidence_mask;
	//findMaskBaseConfidence(ver_confidence_map_6, 3, confidence_mask);
	////findMaskBaseConfidenceLocalGrads(ver_confidence_map_6, 3.0, confidence_mask);
	//encode_machine_.maskMap(confidence_mask, unwrap_ver);

	cv::Mat texture_map = patterns[18];
	texture_map.convertTo(texture_map, CV_8U);
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat z_map_table;

	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);


	//startTime = clock();
	//FilterModule filter_machine;
	////相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.8, 3, 4);
	////filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	//endTime = clock();
	//std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
}

bool DfSolution::reconstructPatterns04RepetitionBaseTable(std::vector<std::vector<cv::Mat>> patterns_list, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);


	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/************************************************************************************/

	std::vector<cv::Mat> patterns;

	DF_Encode encode_machine_;
	encode_machine_.mergePatterns(patterns_list, patterns);

	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);




	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}
	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();

	float confidence_val = 5;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	encode_machine_.maskMap(unwrap_mask, unwrap_ver);

	cv::Mat confidence_mask;
	//findMaskBaseConfidence(ver_confidence_map_6, 3, confidence_mask);
	////findMaskBaseConfidenceLocalGrads(ver_confidence_map_6, 3.0, confidence_mask);
	//encode_machine_.maskMap(confidence_mask, unwrap_ver);

	cv::Mat texture_map = patterns[18];
	texture_map /= patterns_list.size();
	texture_map.convertTo(texture_map, CV_8U);
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat z_map_table;

	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);


	//startTime = clock();
	//FilterModule filter_machine;
	////相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.8, 3, 4);
	////filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	//endTime = clock();
	//std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);



	std::cout << "pointcloud: " << save_points_dir;

	return true;
}

bool DfSolution::reconstructMixedVariableWavelengthXPatternsBaseMiniTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();//计时开始


	MiniLookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);


	endTime = clock();//计时结束
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/************************************************************************************/



	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);



	DF_Encode encode_machine_;

	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}


	float confidence_val = 10;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	encode_machine_.maskMap(unwrap_mask, unwrap_ver);


	cv::Mat texture_map = patterns[18];
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	cv::Mat z_map_table;
	//查表重建、deep_map 三通道为x y z三通道的double 数据

	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	//deep_map_table是点云数据
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);




	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 400, 800);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);

	std::cout << "pointcloud: " << save_points_dir;

	return true;
}



bool DfSolution::reconstructMixedVariableWavelengthXPatternsBaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	lookup_table_machine_.setImageResolution(nc, nr);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;
	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);

	//MiniLookupTableFunction minilooktable_machine;
	//minilooktable_machine.setCameraResolution(nc, nr);
	//minilooktable_machine.setProjectorVersion(camera_version_);
	//minilooktable_machine.setCalibData(calib_param);
	cv::Mat xL_rotate_x_new;
	cv::Mat xL_rotate_y_new;
	cv::Mat R1_new;
	cv::Mat pattern_mapping_new;
	cv::Mat pattern_minimapping_new;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = lookup_table_machine_.generateLookTable(xL_rotate_x_new, xL_rotate_y_new, R1_new, pattern_mapping_new);

	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	/************************************************************************************/






	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;


	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);



	DF_Encode encode_machine_;

	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}
	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();

	float confidence_val = 5;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	encode_machine_.maskMap(unwrap_mask, unwrap_ver);

	cv::Mat confidence_mask;
	//findMaskBaseConfidence(ver_confidence_map_6, 3, confidence_mask);
	////findMaskBaseConfidenceLocalGrads(ver_confidence_map_6, 3.0, confidence_mask);
	//encode_machine_.maskMap(confidence_mask, unwrap_ver);

	cv::Mat texture_map = patterns[18];
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	//cv::Mat balance_white_dst;
	//cv::SimpleWB simpleWB = cv::SimpleWB.Create();
	//simpleWB.BalanceWhite(texture_map, balance_white_dst);

	//if (3 == texture_map.channels())
	//{
	//	std::vector<Mat> imageRGB;

	//	//RGB三通道分离
	//	split(texture_map, imageRGB);

	//	//求原始图像的RGB分量的均值
	//	double R, G, B;
	//	B = mean(imageRGB[0])[0];
	//	G = mean(imageRGB[1])[0];
	//	R = mean(imageRGB[2])[0];

	//	//需要调整的RGB分量的增益
	//	double KR, KG, KB;
	//	KB = (R + G + B) / (3 * B);
	//	KG = (R + G + B) / (3 * G);
	//	KR = (R + G + B) / (3 * R);

	//	//调整RGB三个通道各自的值
	//	imageRGB[0] = imageRGB[0] * KB;
	//	imageRGB[1] = imageRGB[1] * KG;
	//	imageRGB[2] = imageRGB[2] * KR;
	//	//RGB三通道图像合并
	//	merge(imageRGB, texture_map);
	//}

	cv::Mat z_map_table;

	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);


	startTime = clock(); 
	FilterModule filter_machine;
	//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12 m，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.5, 3, 3);
	//filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	endTime = clock();
	std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
}

bool DfSolution::reconstructMixedVariableWavelengthXPatternsBaseTableAndConfidence(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (19 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	lookup_table_machine_.setImageResolution(nc, nr);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;
	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);

	//MiniLookupTableFunction minilooktable_machine;
	//minilooktable_machine.setCameraResolution(nc, nr);
	//minilooktable_machine.setProjectorVersion(camera_version_);
	//minilooktable_machine.setCalibData(calib_param);
	cv::Mat xL_rotate_x_new;
	cv::Mat xL_rotate_y_new;
	cv::Mat R1_new;
	cv::Mat pattern_mapping_new;
	cv::Mat pattern_minimapping_new;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = lookup_table_machine_.generateLookTable(xL_rotate_x_new, xL_rotate_y_new, R1_new, pattern_mapping_new);

	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	/************************************************************************************/






	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);

	std::vector<cv::Mat> ver_wrap_img_4;
	cv::Mat ver_confidence_map_4;


	std::vector<cv::Mat> ver_wrap_img_6;
	cv::Mat ver_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);



	DF_Encode encode_machine_;

	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);

	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);


	cv::Mat unwrap_ver;
	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	// mask for fisher
	cv::Mat mask_4_coefficents(nr, nc, CV_64F, cv::Scalar(0));

	ret = encode_machine_.unwrapVariableWavelengthPatternsBaseConfidence(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, mask_4_coefficents);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}
	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();

	float confidence_val = 5;

	float ver_period = ver_period_num;
	unwrap_ver /= ver_period;

	// Generate Fisher Mask
	cv::Mat fisherMask(wrap_0.size(), CV_8U, cv::Scalar(255));
	double center_1 = -1.0705727873145959e-05;
	double center_2 = -3.118464936149469e-05;
	double center = center_2 + (center_1 - center_2) * (this->confidence_level_ / 100.);

	for (int r = 0; r < fisherMask.rows; r += 1)
	{
		unsigned char* fisherMaskPtr = fisherMask.ptr<unsigned char>(r);
		double* fisherVal = mask_4_coefficents.ptr<double>(r);
		for (int c = 0; c < fisherMask.cols; c += 1)
		{
			if (fisherVal[c] < center)
			{
				fisherMaskPtr[c] = 0;
			}
		}
	}

	encode_machine_.maskMap(fisherMask, unwrap_ver);


	cv::Mat texture_map = patterns[18];
	cv::Mat undistort_img;
	lookup_table_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	//cv::Mat balance_white_dst;
	//cv::SimpleWB simpleWB = cv::SimpleWB.Create();
	//simpleWB.BalanceWhite(texture_map, balance_white_dst);

	//if (3 == texture_map.channels())
	//{
	//	std::vector<Mat> imageRGB;

	//	//RGB三通道分离
	//	split(texture_map, imageRGB);

	//	//求原始图像的RGB分量的均值
	//	double R, G, B;
	//	B = mean(imageRGB[0])[0];
	//	G = mean(imageRGB[1])[0];
	//	R = mean(imageRGB[2])[0];

	//	//需要调整的RGB分量的增益
	//	double KR, KG, KB;
	//	KB = (R + G + B) / (3 * B);
	//	KG = (R + G + B) / (3 * G);
	//	KR = (R + G + B) / (3 * R);

	//	//调整RGB三个通道各自的值
	//	imageRGB[0] = imageRGB[0] * KB;
	//	imageRGB[1] = imageRGB[1] * KG;
	//	imageRGB[2] = imageRGB[2] * KR;
	//	//RGB三通道图像合并
	//	merge(imageRGB, texture_map);
	//}

	cv::Mat z_map_table;

	lookup_table_machine_.rebuildData(unwrap_ver, 1, z_map_table, unwrap_mask);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, unwrap_mask, deep_map_table);


	startTime = clock();
	FilterModule filter_machine;
	//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12 m，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.5, 3, 3);
	//filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	endTime = clock();
	std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table.tiff";
	std::string save_points_dir = work_path_ + "_points_table.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, unwrap_mask);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, texture_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
}



bool DfSolution::reconstructBasePhase02(cv::Mat phase_x, cv::Mat phase_y, cv::Mat brightness, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (phase_x.empty() || phase_y.empty() || brightness.empty())
	{
		return false;
	}

	phase_x.convertTo(phase_x, CV_64FC1);
	phase_y.convertTo(phase_y, CV_64FC1);

	cv::Mat deep_map;


	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);
	reconstruct_machine_.setProjectorVersion(projector_version_);

	cv::Mat err_map;

	bool ret = reconstruct_machine_.rebuildData(phase_x, phase_y, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}

	cv::Mat color_err_map;
	cv::Mat gray_err_map;
	renderErrorMap(err_map, color_err_map, gray_err_map, 0., 0.1);

	AnalyseError analyse_err_machine;

	double err_value = analyse_err_machine.computeError(err_map);
	std::cout << "calibrate err: " << err_value << std::endl;

	cv::Mat texture_map = brightness.clone();

	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = pointcloud_path + "/test_";



	std::vector<cv::Mat> deep_channels;

	cv::split(deep_map, deep_channels);

	cv::Mat depth_map;

	deep_channels[2].convertTo(depth_map, CV_32F);


	std::string save_err_tiff = work_path_ + "phase_02_err.tiff";
	std::string save_depth_tiff = work_path_ + "phase_02_depth.tiff";
	std::string save_points_dir = work_path_ + "phase_02_points.xyz";
	std::string save_depth_txt_dir = work_path_ + "phase_02_depth.txt";
	std::string save_confidence_dir = work_path_ + "phase_02_confidence.bmp";
	std::string save_depth_dir = work_path_ + "phase_02_depth.bmp";
	std::string save_brightness_dir = work_path_ + "phase_02_brightness.bmp";
	std::string save_points_z_dir = work_path_ + "phase_02_point_z.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map, color_map, grey_map, 300, 2000);
	//MaskZMap(color_map, unwrap_mask);



	cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, depth_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map, save_points_dir, texture_map);
	//file_io_machine.saveDepthMapToTxt(deep_map, save_depth_txt_dir);



	std::cout << "pointcloud: " << save_points_dir;

	return true;
}


bool DfSolution::reconstructPatterns05(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (patterns.empty())
	{
		return false;
	}

	if (16 != patterns.size())
	{
		std::cout << "err patterns numbers: " << patterns.size() << std::endl;
	}

	std::vector<cv::Mat> phase_shift_patterns_img(patterns.begin(), patterns.begin() + 6); 
	std::vector<cv::Mat> xor_code_patterns_img(patterns.begin() + 6, patterns.begin() + 6 + 8);

 

	cv::Mat white_map = patterns[14].clone();
	cv::Mat black_map = patterns[15].clone();

	xor_code_patterns_img.push_back(black_map.clone());
	xor_code_patterns_img.push_back(white_map.clone());

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat threshold_map(nr, nc, CV_8U, cv::Scalar(128));



	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = black_map.ptr<uchar>(r);
		uchar* ptr_w = white_map.ptr<uchar>(r);
		uchar* ptr_t = threshold_map.ptr<uchar>(r); 
		for (int c = 0; c < nc; c++)
		{
			float d = ptr_w[c] - ptr_b[c];
			ptr_t[c] = ptr_b[c] + 0.5 + d / 2.0;
		} 
	}


	DF_Encode encode;
 
	  
	cv::Mat k1_map, k2_map;
	cv::Mat mask;
	bool ret = encode.computeXOR05(xor_code_patterns_img, k1_map, k2_map, mask);

	cv::Mat wrap, confidence, average, brightness;


	ret = encode.computePhaseShift(phase_shift_patterns_img, wrap, confidence, average, brightness, mask);

	cv::Mat unwrap_x;
	ret = encode.unwrapBase2Kmap(wrap, k1_map, k2_map, unwrap_x);

	cv::Mat decode_color_map= k2_map.clone();
	decode_color_map.convertTo(decode_color_map, CV_8U);
	cv::applyColorMap(decode_color_map, decode_color_map, cv::COLORMAP_JET);
	

	std::string save_decode_dir = pointcloud_path + "/decode_05.bmp";
	cv::imwrite(save_decode_dir, decode_color_map);
	/********************************************************************************************************/


	float confidence_val = 2; 
	float ver_period = 128; 

	cv::Mat mask_phase(nr,nc,CV_8UC1,cv::Scalar(255));

	unwrap_x /= ver_period; 
	encode.selectMaskBaseConfidence(confidence, confidence_val, mask_phase);
	//encode_machine_.selectMaskBaseConfidence(hor_confidence_map_4, confidence_val, unwrap_mask);


	encode.maskMap(mask_phase, unwrap_x);

	unwrap_x.convertTo(unwrap_x, CV_64FC1);

	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	lookup_table_machine_.setImageResolution(nc, nr);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;
	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);

	//MiniLookupTableFunction minilooktable_machine;
	//minilooktable_machine.setCameraResolution(nc, nr);
	//minilooktable_machine.setProjectorVersion(camera_version_);
	//minilooktable_machine.setCalibData(calib_param);
	cv::Mat xL_rotate_x_new;
	cv::Mat xL_rotate_y_new;
	cv::Mat R1_new;
	cv::Mat pattern_mapping_new;
	cv::Mat pattern_minimapping_new;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = lookup_table_machine_.generateLookTable(xL_rotate_x_new, xL_rotate_y_new, R1_new, pattern_mapping_new);

	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	/************************************************************************************/



	cv::Mat z_map_table;
	//LookupTableFunction lookup_table_machine_;

	lookup_table_machine_.rebuildData(unwrap_x, 1, z_map_table, mask_phase);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, mask_phase, deep_map_table);


	startTime = clock();
	FilterModule filter_machine;
	//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12 m，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.5, 3, 3);
	//filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	endTime = clock();
	std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table_05.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table_05.tiff";
	std::string save_points_dir = work_path_ + "_points_table_05.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table_05.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table_05.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table_05.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table_05.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table_05.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, mask_phase);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, white_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, white_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
	 

}

bool DfSolution::reconstructPatternsTest(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (patterns.empty())
	{
		return false;
	}

	std::vector<cv::Mat> phase_shift_patterns_img(patterns.begin() + +2 , patterns.begin() + 8);
	std::vector<cv::Mat> minsw_gray_code_patterns_img(patterns.begin() + 8, patterns.begin() + 8 + 8);
	std::vector<cv::Mat> inv_minsw_gray_code_patterns_img(patterns.begin() + 16, patterns.begin() + 16 + 8);


	DF_Encode encode;
	cv::Mat mask;

	cv::Mat wrap, confidence,global, average, brightness;


	bool ret = encode.computePhaseShift(phase_shift_patterns_img, wrap, confidence, global, average, brightness, mask);

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;


	cv::Mat white_map = patterns[0].clone();
	cv::Mat black_map = patterns[1].clone();

	cv::Mat test_max_map(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat test_min_map(nr, nc, CV_8U, cv::Scalar(255));

	for (int i = 2; i < 8; i++)
	{

		std::cout << "i: " << i << std::endl;
		cv::Mat image = patterns[i].clone();
	  
		cv::blur(image, image, cv::Size(5, 5));

		for (int r = 0; r < nr; r++)
		{ 
			uchar* ptr_p = image.ptr<uchar>(r);
			uchar* ptr_d = test_max_map.ptr<uchar>(r);
			uchar* ptr_g = test_min_map.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				if (ptr_p[c] > ptr_d[c])
				{
					ptr_d[c] = ptr_p[c];
				}

				if (ptr_p[c] < ptr_g[c])
				{
					ptr_g[c] = ptr_p[c];
				}
			}

		}

		std::cout << "i: " << i << std::endl;
	}

	cv::imwrite("../global_all.bmp", test_max_map);
	cv::imwrite("../direct_all.bmp", test_min_map);


	cv::Mat test_direct_map(nr, nc, CV_8U, cv::Scalar(0));
	cv::Mat test_global_map(nr, nc, CV_8U, cv::Scalar(255));

	float b = 0.30;

	for (int r = 0; r < nr; r++)
	{ 
		uchar* ptr_max = test_max_map.ptr<uchar>(r);
		uchar* ptr_min = test_min_map.ptr<uchar>(r);

		uchar* ptr_direct = test_direct_map.ptr<uchar>(r);
		uchar* ptr_global = test_global_map.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{

			float d = (ptr_max[c] - ptr_min[c]) / (1 - b);

			if (d > 255)
			{
				ptr_direct[c] = 255;
			}
			else
			{
				ptr_direct[c] = d;
			}
   
			float g = 2 * (ptr_min[c] - ptr_max[c] *
				b) / (1 - b * b);

			if (g < 0)
			{
				ptr_global[c] = 0 ;
			}
			else
			{
				ptr_global[c] = g;
			}
		 
		}

	}


	//cv::Mat white_map = minsw_gray_code_patterns_img[0] + inv_minsw_gray_code_patterns_img[0];

	std::vector<cv::Mat> threshold_list;

	cv::Mat threshold_mask(nr, nc, CV_8U, cv::Scalar(255));

	for (int p_i = 0; p_i < minsw_gray_code_patterns_img.size(); p_i++)
	{

		cv::Mat threshold_map(nr, nc, CV_8U, cv::Scalar(128));

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_b = inv_minsw_gray_code_patterns_img[p_i].ptr<uchar>(r);
			uchar* ptr_w = minsw_gray_code_patterns_img[p_i].ptr<uchar>(r);
			uchar* ptr_t = threshold_map.ptr<uchar>(r);
			uchar* ptr_m = threshold_mask.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				uchar diff = std::abs(ptr_w[c] - ptr_b[c]);

				if (diff < ptr_m[c])
				{
					ptr_m[c] = diff;
				}

				float d = ptr_w[c] + ptr_b[c];
				ptr_t[c] = 0.5 + d / 2.0;
			}

		}

		threshold_list.push_back(threshold_map.clone());
	}



	cv::Mat threshold_map = (test_max_map - test_min_map) * 0.5 + test_min_map;

	cv::Mat  sw_k_map;
	cv::Mat  sw_mask;
	cv::Mat  test_uncertain;
	//minsw_gray_code_patterns_img.pop_back();
	//minsw_gray_code_patterns_img.pop_back();
	ret = encode.decodeMinswGrayCode(minsw_gray_code_patterns_img, threshold_map,
		test_direct_map,test_global_map, test_uncertain,sw_k_map);

	cv::Mat test_global = global.clone();
	cv::Mat test_direct = confidence.clone();

	cv::Mat minsw_map(nr, nc, CV_32F, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		float* ptr_sw = minsw_map.ptr<float>(r);
		ushort* ptr_k2 = sw_k_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			int bin_value = -1;
			bool ret = encode.minsw8CodeToValue(ptr_k2[c], bin_value);

			if (ret)
			{
				ptr_sw[c] = bin_value;
			}
			else
			{
				ptr_sw[c] = -1;
			}
		}
	}


	cv::Mat k2_map;

	minsw_map.convertTo(k2_map, CV_16U);

	cv::Mat k1_map(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k2 = k2_map.ptr<ushort>(r);
		ushort* ptr_k1 = k1_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			ptr_k1[c] = ptr_k2[c] / 2;
		}
	}



	// 
	cv::Mat unwrap;
	ret = encode.unwrapBase2Kmap(wrap, k1_map, k2_map, unwrap);

	cv::Mat test_confidence = confidence.clone();
	cv::Mat test_average = average.clone();
	cv::Mat test_brightness = brightness.clone(); 

	/*****************************************************************************************************/

	cv::Mat decode_color_map = k2_map.clone();
	decode_color_map.convertTo(decode_color_map, CV_8U);
	cv::applyColorMap(decode_color_map, decode_color_map, cv::COLORMAP_JET);


	std::string save_decode_dir = pointcloud_path + "/decode_test.bmp";
	cv::imwrite(save_decode_dir, decode_color_map);
	/********************************************************************************************************/


	float confidence_val = 1;
	float ver_period = 128;

	cv::Mat mask_phase(nr, nc, CV_8UC1, cv::Scalar(255));

	cv::Mat threshold_uncertain = 255 - test_uncertain;
	
	unwrap /= ver_period;
	encode.selectMaskBaseConfidence(confidence, confidence_val, mask_phase);
	//encode.selectMaskBaseConfidence(threshold_mask, confidence_val, mask_phase);
	encode.selectMaskBaseConfidence(threshold_uncertain, 40, mask_phase);
	//encode_machine_.selectMaskBaseConfidence(hor_confidence_map_4, confidence_val, unwrap_mask);


	encode.maskMap(mask_phase, unwrap);

	unwrap.convertTo(unwrap, CV_64FC1);

	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	lookup_table_machine_.setImageResolution(nc, nr);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;
	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);

	//MiniLookupTableFunction minilooktable_machine;
	//minilooktable_machine.setCameraResolution(nc, nr);
	//minilooktable_machine.setProjectorVersion(camera_version_);
	//minilooktable_machine.setCalibData(calib_param);
	cv::Mat xL_rotate_x_new;
	cv::Mat xL_rotate_y_new;
	cv::Mat R1_new;
	cv::Mat pattern_mapping_new;
	cv::Mat pattern_minimapping_new;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = lookup_table_machine_.generateLookTable(xL_rotate_x_new, xL_rotate_y_new, R1_new, pattern_mapping_new);

	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	/************************************************************************************/



	cv::Mat z_map_table;
	//LookupTableFunction lookup_table_machine_;

	lookup_table_machine_.rebuildData(unwrap, 1, z_map_table, mask_phase);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, mask_phase, deep_map_table);


	startTime = clock();
	FilterModule filter_machine;
	//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12 m，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.5, 3, 3);
	//filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	endTime = clock();
	std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table_test.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table_test.tiff";
	std::string save_points_dir = work_path_ + "_points_table_test.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table_test.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table_test.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table_test.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table_test.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table_test.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, mask_phase);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, white_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, white_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
}


bool DfSolution::reconstructPatterns0602(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (patterns.empty())
	{
		return false;
	}

	std::vector<cv::Mat> phase_shift_patterns_img(patterns.begin(), patterns.begin() + 6);
	std::vector<cv::Mat> minsw_gray_code_patterns_img(patterns.begin() + 6, patterns.begin() + 6 + 10);
	std::vector<cv::Mat> inv_minsw_gray_code_patterns_img(patterns.begin() + 16, patterns.begin() + 16 + 10);


	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat white_map = minsw_gray_code_patterns_img[0] + inv_minsw_gray_code_patterns_img[0];

	std::vector<cv::Mat> threshold_list;

	cv::Mat threshold_mask(nr, nc, CV_8U, cv::Scalar(255));

	for (int p_i = 0; p_i < minsw_gray_code_patterns_img.size(); p_i++)
	{

		cv::Mat threshold_map(nr, nc, CV_8U, cv::Scalar(128));

		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_b = inv_minsw_gray_code_patterns_img[p_i].ptr<uchar>(r);
			uchar* ptr_w = minsw_gray_code_patterns_img[p_i].ptr<uchar>(r);
			uchar* ptr_t = threshold_map.ptr<uchar>(r);
			uchar* ptr_m = threshold_mask.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				uchar diff = std::abs(ptr_w[c] - ptr_b[c]);

				if (diff < ptr_m[c])
				{
					ptr_m[c] = diff;
				}

				float d = ptr_w[c] + ptr_b[c];
				ptr_t[c] = 0.5 + d / 2.0;
			}

		}

		threshold_list.push_back(threshold_map.clone());
	}





	DF_Encode encode;
	cv::Mat  sw_k_map;
	cv::Mat  sw_mask;
	//minsw_gray_code_patterns_img.pop_back();
	//minsw_gray_code_patterns_img.pop_back();
	bool ret = encode.decodeMinswGrayCode(minsw_gray_code_patterns_img, threshold_list, sw_k_map);

	cv::Mat minsw_map(nr, nc, CV_32F, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		float* ptr_sw = minsw_map.ptr<float>(r);
		ushort* ptr_k2 = sw_k_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			int bin_value = -1;
			bool ret = encode.minsw10CodeToValue(ptr_k2[c], bin_value);

			if (ret)
			{
				ptr_sw[c] = bin_value;
			}
			else
			{
				ptr_sw[c] = -1;
			}
		}
	}


	cv::Mat k2_map;

	minsw_map.convertTo(k2_map, CV_16U);

	cv::Mat k1_map(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k2 = k2_map.ptr<ushort>(r);
		ushort* ptr_k1 = k1_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			ptr_k1[c] = ptr_k2[c] / 2;
		}
	}


	cv::Mat mask;

	cv::Mat wrap, confidence, average, brightness;


	ret = encode.computePhaseShift(phase_shift_patterns_img, wrap, confidence, average, brightness, mask);
	// 
	cv::Mat unwrap;
	ret = encode.unwrapBase2Kmap(wrap, k1_map, k2_map, unwrap);

	/*****************************************************************************************************/

	cv::Mat decode_color_map = k2_map.clone();
	decode_color_map.convertTo(decode_color_map, CV_8U);
	cv::applyColorMap(decode_color_map, decode_color_map, cv::COLORMAP_JET);


	std::string save_decode_dir = pointcloud_path + "/decode_0602.bmp";
	cv::imwrite(save_decode_dir, decode_color_map);
	/********************************************************************************************************/


	float confidence_val = 2;
	float ver_period = 128;

	cv::Mat mask_phase(nr, nc, CV_8UC1, cv::Scalar(255));

	unwrap /= ver_period;
	encode.selectMaskBaseConfidence(confidence, confidence_val, mask_phase);
	encode.selectMaskBaseConfidence(threshold_mask, 2, mask_phase);
	//encode_machine_.selectMaskBaseConfidence(hor_confidence_map_4, confidence_val, unwrap_mask);


	encode.maskMap(mask_phase, unwrap);

	unwrap.convertTo(unwrap, CV_64FC1);

	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	lookup_table_machine_.setImageResolution(nc, nr);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;
	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);

	//MiniLookupTableFunction minilooktable_machine;
	//minilooktable_machine.setCameraResolution(nc, nr);
	//minilooktable_machine.setProjectorVersion(camera_version_);
	//minilooktable_machine.setCalibData(calib_param);
	cv::Mat xL_rotate_x_new;
	cv::Mat xL_rotate_y_new;
	cv::Mat R1_new;
	cv::Mat pattern_mapping_new;
	cv::Mat pattern_minimapping_new;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = lookup_table_machine_.generateLookTable(xL_rotate_x_new, xL_rotate_y_new, R1_new, pattern_mapping_new);

	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	/************************************************************************************/



	cv::Mat z_map_table;
	//LookupTableFunction lookup_table_machine_;

	lookup_table_machine_.rebuildData(unwrap, 1, z_map_table, mask_phase);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, mask_phase, deep_map_table);


	startTime = clock();
	FilterModule filter_machine;
	//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12 m，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.5, 3, 3);
	//filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	endTime = clock();
	std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table_0602.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table_0602.tiff";
	std::string save_points_dir = work_path_ + "_points_table_0602.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table_0602.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table_0602.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table_0602.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table_0602.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table_0602.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, mask_phase);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, white_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, white_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
}

bool DfSolution::reconstructPatterns06(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (patterns.empty())
	{
		return false;
	}

	std::vector<cv::Mat> phase_shift_patterns_img(patterns.begin()+2, patterns.begin() + 8);
	std::vector<cv::Mat> minsw_gray_code_patterns_img(patterns.begin() + 8, patterns.begin() + 8 + 8);  

	cv::Mat map_white_map = patterns[0].clone();
	cv::Mat map_black_map = patterns[1].clone();
 

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat threshold_map(nr, nc, CV_8U, cv::Scalar(128));
	cv::Mat threshold_confidence(nr, nc, CV_8U, cv::Scalar(0));

	 
	for (int r = 0; r < nr; r++)
	{
		uchar* ptr_b = map_black_map.ptr<uchar>(r);
		uchar* ptr_w = map_white_map.ptr<uchar>(r);
		uchar* ptr_t = threshold_map.ptr<uchar>(r); 
		uchar* ptr_c = threshold_confidence.ptr<uchar>(r);

		for (int c = 0; c < nc; c++)
		{
			float d = ptr_w[c] - ptr_b[c];
			ptr_t[c] = ptr_b[c] + 0.5 + d / 2.0;
			ptr_c[c] = std::abs(d);
		}

	}


	DF_Encode encode;
	cv::Mat  sw_k_map; 
	bool ret = encode.decodeMinswGrayCode(minsw_gray_code_patterns_img, threshold_map, sw_k_map);

	cv::Mat minsw_map(nr, nc, CV_32F, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		float* ptr_sw = minsw_map.ptr<float>(r);
		ushort* ptr_k2 = sw_k_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			int bin_value = -1;
			bool ret = encode.minsw8CodeToValue(ptr_k2[c], bin_value);

			if (ret)
			{
				ptr_sw[c] = bin_value;
			}
			else
			{
				ptr_sw[c] = -1;
			}
		}
	}


	cv::Mat k2_map; 
	minsw_map.convertTo(k2_map, CV_16U);

	cv::Mat k1_map(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k2 = k2_map.ptr<ushort>(r);
		ushort* ptr_k1 = k1_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			ptr_k1[c] = ptr_k2[c] / 2;
		}
	}

	 
	cv::Mat mask; 

	cv::Mat wrap, confidence, average, brightness; 

	ret = encode.computePhaseShift(phase_shift_patterns_img, wrap, confidence, average, brightness, mask);
 
	cv::Mat unwrap;
	ret = encode.unwrapBase2Kmap(wrap, k1_map, k2_map, unwrap);


	cv::Mat decode_color_map = k2_map.clone();
	decode_color_map.convertTo(decode_color_map, CV_8U);
	cv::applyColorMap(decode_color_map, decode_color_map, cv::COLORMAP_JET);


	std::string save_decode_dir = pointcloud_path + "/decode_06.bmp";
	cv::imwrite(save_decode_dir, decode_color_map);
	/********************************************************************************************************/


	float confidence_val = 2;
	float ver_period = 128;

	cv::Mat mask_phase(nr, nc, CV_8UC1, cv::Scalar(255));

	unwrap /= ver_period;
	encode.selectMaskBaseConfidence(confidence, confidence_val, mask_phase);
	//encode_machine_.selectMaskBaseConfidence(hor_confidence_map_4, confidence_val, unwrap_mask); 

	encode.maskMap(mask_phase, unwrap);

	encode.selectMaskBaseConfidence(threshold_confidence, 5, mask_phase);
	encode.maskMap(mask_phase, unwrap);

	unwrap.convertTo(unwrap, CV_64FC1);

	/***********************************************************************************/

	clock_t startTime, endTime;
	startTime = clock();


	LookupTableFunction lookup_table_machine_;
	//LookupTableFunction lookup_table_machine;
	lookup_table_machine_.setCalibData(calib_param);
	lookup_table_machine_.setProjectorVersion(projector_version_);
	lookup_table_machine_.setImageResolution(nc, nr);

	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;
	//lookup_table_machine_.generateLookTable(xL_rotate_x, xL_rotate_y, R1, pattern_mapping);

	//lookup_table_machine_.readTable("../", 1200, 1920);

	//MiniLookupTableFunction minilooktable_machine;
	//minilooktable_machine.setCameraResolution(nc, nr);
	//minilooktable_machine.setProjectorVersion(camera_version_);
	//minilooktable_machine.setCalibData(calib_param);
	cv::Mat xL_rotate_x_new;
	cv::Mat xL_rotate_y_new;
	cv::Mat R1_new;
	cv::Mat pattern_mapping_new;
	cv::Mat pattern_minimapping_new;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = lookup_table_machine_.generateLookTable(xL_rotate_x_new, xL_rotate_y_new, R1_new, pattern_mapping_new);

	endTime = clock();
	std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


	/************************************************************************************/



	cv::Mat z_map_table;
	//LookupTableFunction lookup_table_machine_;

	lookup_table_machine_.rebuildData(unwrap, 1, z_map_table, mask_phase);

	cv::Mat deep_map_table;

	std::vector<cv::Point3f> points_cloud;
	ret = lookup_table_machine_.generate_pointcloud(z_map_table, mask_phase, deep_map_table);


	startTime = clock();
	FilterModule filter_machine;
	//相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12 m，典型值0.54mm（1200） 
	//filter_machine.RadiusOutlierRemoval(deep_map_table, unwrap_mask, 0.5, 3, 3);
	//filter_machine.statisticOutlierRemoval(deep_map_table, 6, 1);
	endTime = clock();
	std::cout << "statisticOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	/*********************************************************************************/

	std::string work_path_ = pointcloud_path + "/test";


	std::vector<cv::Mat> deep_channels;
	cv::split(deep_map_table, deep_channels);
	cv::Mat z_map;
	deep_channels[2].convertTo(z_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err_table_06.tiff";
	std::string save_depth_tiff = work_path_ + "_depth_table_06.tiff";
	std::string save_points_dir = work_path_ + "_points_table_06.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth_table_06.txt";
	std::string save_confidence_dir = work_path_ + "_confidence_table_06.bmp";
	std::string save_depth_dir = work_path_ + "_depth_table_06.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness_table_06.bmp";
	std::string save_points_z_dir = work_path_ + "point_z_table_06.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map_table, color_map, grey_map, 300, 2000);
	MaskZMap(color_map, mask_phase);


	//cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, z_map);
	cv::imwrite(save_brightness_dir, map_white_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map_table, save_points_dir, map_white_map);


	std::cout << "pointcloud: " << save_points_dir;

	return true;
	 
}


bool DfSolution::reconstructPatterns08(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	if (patterns.empty())
	{
		return false;
	}

	std::vector<cv::Mat> phase_shift_patterns_img(patterns.begin(), patterns.begin() + 6);
	std::vector<cv::Mat> minsw10_gray_code_patterns_img(patterns.begin() + 6, patterns.begin()+ 6+10);
	std::vector<cv::Mat> minsw8_gray_code_patterns_img(patterns.begin() + 16, patterns.begin() + 16 + 8);
 

	cv::Mat white_map = patterns[24];
	cv::Mat black_map = patterns[25];
	
	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	cv::Mat threshold_map(nr, nc, CV_8U, cv::Scalar(128));
	 

	cv::Mat threshold_mask(nr, nc, CV_8U, cv::Scalar(255));

	for (int p_i = 0; p_i < minsw10_gray_code_patterns_img.size(); p_i++)
	{ 
		for (int r = 0; r < nr; r++)
		{
			uchar* ptr_b = black_map.ptr<uchar>(r);
			uchar* ptr_w = white_map.ptr<uchar>(r);
			uchar* ptr_t = threshold_map.ptr<uchar>(r);
			uchar* ptr_m = threshold_mask.ptr<uchar>(r);

			for (int c = 0; c < nc; c++)
			{
				uchar diff = std::abs(ptr_w[c] - ptr_b[c]);

				if (diff < ptr_m[c])
				{
					ptr_m[c] = diff;
				}

				float d = ptr_w[c] - ptr_b[c];
				ptr_t[c] = ptr_b[c] + 0.5 + d/2.0 ;
			} 
		} 
	}
	 




	DF_Encode encode;
	cv::Mat  sw10_k_map;
	cv::Mat  sw10_mask;

	cv::Mat  sw8_k_map;
	cv::Mat  sw8_mask;

	//minsw_gray_code_patterns_img.pop_back();
	//minsw_gray_code_patterns_img.pop_back();
	bool ret = encode.decodeMinswGrayCode(minsw10_gray_code_patterns_img, threshold_map, sw10_k_map);
	ret = encode.decodeMinswGrayCode(minsw8_gray_code_patterns_img, threshold_map, sw8_k_map);

	cv::Mat minsw10_map(nr, nc, CV_32F, cv::Scalar(0));
	cv::Mat minsw8_map(nr, nc, CV_32F, cv::Scalar(0));

	for (int r = 0; r < nr; r++)
	{
		float* ptr_sw = minsw10_map.ptr<float>(r);
		ushort* ptr_k2 = sw10_k_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			int bin_value = -1;
			bool ret = encode.minsw10CodeToValue(ptr_k2[c], bin_value);

			if (ret)
			{
				ptr_sw[c] = bin_value;
			}
			else
			{
				ptr_sw[c] = -1;
			}
		}
	}
	/*******************************************************************************************************************/
	for (int r = 0; r < nr; r++)
	{
		float* ptr_sw = minsw8_map.ptr<float>(r);
		ushort* ptr_k2 = sw8_k_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			int bin_value = -1;
			bool ret = encode.minsw8CodeToValue(ptr_k2[c], bin_value);

			if (ret)
			{
				ptr_sw[c] = bin_value;
			}
			else
			{
				ptr_sw[c] = -1;
			}
		}
	}

	cv::Mat decode_color_map = minsw8_map.clone();
	decode_color_map.convertTo(decode_color_map, CV_8U);
	cv::applyColorMap(decode_color_map, decode_color_map, cv::COLORMAP_JET);


	std::string save_decode_dir = pointcloud_path + "/decode_minsw8.bmp";
	cv::imwrite(save_decode_dir, decode_color_map);


	decode_color_map = minsw10_map.clone();
	decode_color_map.convertTo(decode_color_map, CV_8U);
	cv::applyColorMap(decode_color_map, decode_color_map, cv::COLORMAP_JET);


	save_decode_dir = pointcloud_path + "/decode_minsw10.bmp";
	cv::imwrite(save_decode_dir, decode_color_map);
	/****************************************************************************************************************/


	cv::Mat minsw10_k2_map;

	minsw10_map.convertTo(minsw10_k2_map, CV_16U);

	cv::Mat minsw10_k1_map(nr, nc, CV_16U, cv::Scalar(0));


	for (int r = 0; r < nr; r++)
	{
		ushort* ptr_k2 = minsw10_k2_map.ptr<ushort>(r);
		ushort* ptr_k1 = minsw10_k1_map.ptr<ushort>(r);
		for (int c = 0; c < nc; c++)
		{
			ptr_k1[c] = ptr_k2[c] / 2;
		}
	}

	  
	cv::Mat mask; 

	cv::Mat wrap, confidence, average, brightness;


	ret = encode.computePhaseShift(phase_shift_patterns_img, wrap, confidence, average, brightness, mask);
	// 
	cv::Mat unwrap;
	ret = encode.unwrapBase2Kmap(wrap, minsw10_k1_map, minsw10_k2_map, unwrap);


	////gray_code 
	//cv::Mat gray_k1, gray_k2;
	//ret = encode.decodeGrayCode(gray_code_patterns_img, threshold_map, gray_k1, gray_k2);


	return true;
}


bool DfSolution::reconstructPatterns06BaseTable(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{
	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	int gray_code_num = 8;
	std::vector<cv::Mat> gray_code_patterns_img(patterns.begin(), patterns.begin() + gray_code_num);
	std::vector<cv::Mat> phase_shift_patterns_img(patterns.begin() + gray_code_num, patterns.end());

	cv::Mat mask;
	cv::Mat confidence;
	cv::Mat wrap;
	cv::Mat average;
	cv::Mat brightness;

	DF_Encode encode;
	bool ret = encode.computePhaseShift(phase_shift_patterns_img, wrap, confidence, average, brightness, mask);


	cv::Mat k1_map, k2_map;
	ret = encode.decodeGrayCode(gray_code_patterns_img, average, k1_map, k2_map);

	cv::Mat unwrap;
	ret = encode.unwrapBase2Kmap(wrap, k1_map, k2_map, unwrap);


	return true;
}

bool DfSolution::reconstructMixedVariableWavelengthPatternsBaseXYSR(std::vector<cv::Mat> patterns, struct CameraCalibParam calib_param, std::string pointcloud_path)
{

	if (31 != patterns.size())
	{
		return false;
	}

	bool ret = true;

	int ver_pstterns_num = 18;
	int hor_patterns_num = 12;

	int nr = patterns[0].rows;
	int nc = patterns[0].cols;

	std::vector<cv::Mat> ver_patterns_img(patterns.begin(), patterns.begin() + ver_pstterns_num);
	std::vector<cv::Mat> hor_patterns_img(patterns.begin() + ver_pstterns_num, patterns.end() - 1);

	std::vector<cv::Mat> ver_wrap_img_4;
	std::vector<cv::Mat> hor_wrap_img_4;
	cv::Mat ver_confidence_map_4;
	cv::Mat hor_confidence_map_4;

	std::vector<cv::Mat> ver_wrap_img_6;
	std::vector<cv::Mat> hor_wrap_img_6;
	cv::Mat ver_confidence_map_6;
	cv::Mat hor_confidence_map_6;

	cv::Mat org_mask_(nr, nc, CV_8U, cv::Scalar(255));
	cv::rectangle(org_mask_, cv::Point(0, 0), cv::Point(nc - 1, nr - 1), cv::Scalar(0), 3);


	cv::Mat test_mask_ = org_mask_.clone();


	std::vector<cv::Mat> ver_patterns_img_4(ver_patterns_img.begin(), ver_patterns_img.begin() + ver_pstterns_num - 6);
	std::vector<cv::Mat> hor_patterns_img_4(hor_patterns_img.begin(), hor_patterns_img.begin() + hor_patterns_num);

	std::vector<cv::Mat> ver_patterns_img_6(ver_patterns_img.begin() + ver_pstterns_num - 6, ver_patterns_img.begin() + ver_pstterns_num);
	//std::vector<cv::Mat> hor_patterns_img_6(hor_patterns_img.begin() + patterns.size() / 2 - 6, hor_patterns_img.begin() + patterns.size() / 2);



	DF_Encode encode_machine_;

	ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
	ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


	ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
	//ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

	cv::Mat wrap_0 = ver_wrap_img_4[0].clone();
	cv::Mat wrap_1 = ver_wrap_img_4[1].clone();
	cv::Mat wrap_2 = ver_wrap_img_4[2].clone();
	cv::Mat wrap_3 = ver_wrap_img_6[0].clone();


	std::vector<double> variable_wrap_rate;
	variable_wrap_rate.push_back(8);
	variable_wrap_rate.push_back(4);
	variable_wrap_rate.push_back(4);


	cv::Mat unwrap_mask = test_mask_.clone();


	int discard_num = 0;

	std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
	std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

	select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
	//select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


	cv::Mat unwrap_hor, unwrap_ver;

	float ver_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		ver_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}

	variable_wrap_rate.pop_back();

	float hor_period_num = 1;

	for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
	{
		hor_period_num *= variable_wrap_rate[r_i];
	}

	ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
	if (!ret)
	{
		std::cout << "unwrap Error!";
		return false;
	}



	float confidence_val = 10;

	float ver_period = ver_period_num;
	float hor_period = hor_period_num * 720.0 / 1280.0;


	unwrap_ver /= ver_period;
	unwrap_hor /= hor_period;

	encode_machine_.selectMaskBaseConfidence(ver_confidence_map_6, confidence_val, unwrap_mask);
	//encode_machine_.selectMaskBaseConfidence(hor_confidence_map_4, confidence_val, unwrap_mask);


	encode_machine_.maskMap(unwrap_mask, unwrap_ver);
	encode_machine_.maskMap(unwrap_mask, unwrap_hor);



	cv::Mat deep_map;


	DF_Reconstruct reconstruct_machine_;
	reconstruct_machine_.setCalibData(calib_param);
	reconstruct_machine_.setProjectorVersion(projector_version_);

	cv::Mat err_map;

	ret = reconstruct_machine_.rebuildData(unwrap_ver, unwrap_hor, 1, deep_map, err_map);
	if (!ret)
	{
		std::cout << "Rebuild Error!";

		return false;

	}


	clock_t startTime, endTime;
	startTime = clock();
	FilterModule filter_machine;
	filter_machine.RadiusOutlierRemoval(deep_map, unwrap_mask, 0.5, 2, 4);
	endTime = clock();
	std::cout << "RadiusOutlierRemoval run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	cv::Mat color_err_map;
	cv::Mat gray_err_map;
	renderErrorMap(err_map, color_err_map, gray_err_map, 0., 0.1);

	AnalyseError analyse_err_machine;

	double err_value = analyse_err_machine.computeError(err_map);
	std::cout << "calibrate err: " << err_value << std::endl;

	cv::Mat texture_map = patterns[30];

	cv::Mat undistort_img;
	reconstruct_machine_.undistortedImage(texture_map, undistort_img);
	texture_map = undistort_img.clone();

	err_map.convertTo(err_map, CV_32F);
	/*********************************************************************************/


	std::string work_path_ = pointcloud_path + "/test";



	std::vector<cv::Mat> deep_channels;

	cv::split(deep_map, deep_channels);

	cv::Mat depth_map;

	deep_channels[2].convertTo(depth_map, CV_32F);


	std::string save_err_tiff = work_path_ + "_err.tiff";
	std::string save_depth_tiff = work_path_ + "_depth.tiff";
	std::string save_points_dir = work_path_ + "_points.xyz";
	std::string save_depth_txt_dir = work_path_ + "_depth.txt";
	std::string save_confidence_dir = work_path_ + "_confidence.bmp";
	std::string save_depth_dir = work_path_ + "_depth.bmp";
	std::string save_brightness_dir = work_path_ + "_brightness.bmp";
	std::string save_points_z_dir = work_path_ + "point_z.tiff";

	cv::Mat color_map, grey_map;
	MapToColor(deep_map, color_map, grey_map, 400, 800);
	MaskZMap(color_map, unwrap_mask);



	cv::imwrite(save_err_tiff, err_map);
	cv::imwrite(save_depth_tiff, depth_map);
	cv::imwrite(save_brightness_dir, texture_map);
	cv::imwrite(save_depth_dir, color_map);
	SavePointToTxt(deep_map, save_points_dir, texture_map);
	//file_io_machine.saveDepthMapToTxt(deep_map, save_depth_txt_dir);



	std::cout << "pointcloud: " << save_points_dir;

	return true;


}



bool DfSolution::test_calib_shake()
{
	float r[9] = { 0.976078,0.0123287,-0.21707,-0.0134238,0.999904,-0.00357109,0.217005,0.00639956,0.97615 };
	cv::Mat rotate(3, 3, CV_32F, r);
	cv::Mat angles;

	rotate_matrix_to_euler_angles(rotate, angles);


	float r_1[9] = { 0.976142,0.0124168,-0.216777,-0.0133835,0.999906,-0.00299181,0.216719,0.00582166,0.976217 };
	cv::Mat rotate_1(3, 3, CV_32F, r_1);
	cv::Mat angles_1;

	rotate_matrix_to_euler_angles(rotate_1, angles_1);
	std::cout << "angles: " << angles << std::endl;
	std::cout << "angles_1: " << angles_1 << std::endl;
	std::cout << "angles_d : " << angles_1 - angles << std::endl;

	float t[3] = { 342.134,-0.632868,23.4816 };
	cv::Mat trans(3, 1, CV_32F, t);

	float t_1[3] = { 341.966,-0.60676,23.3156 };
	cv::Mat trans_1(3, 1, CV_32F, t_1);

	std::cout << "trans_d : " << trans_1 - trans;
}


