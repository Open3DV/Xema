#pragma once
#ifdef _WIN32 
#include "../sdk/open_cam3d.h" 
#include <windows.h>
#elif __linux
#include "../sdk/open_cam3d.h" 
#include <cstring>
#include <stdio.h> 
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
#endif 
#include "../firmware/system_config_settings.h"
#include "../firmware/protocol.h"
#include "../firmware/version.h"
#include "../test/solution.h"
#include "opencv2/opencv.hpp"
#include <assert.h>
#include <fstream>
#include <string.h>
#include "getopt.h" 
#include <iomanip>
#include "../test/LookupTableFunction.h"
#include "../test/triangulation.h"
#include "../sdk/xema_enums.h"

using namespace std;

const char* help_info =
"Examples:\n\
\n\
1.Get temperature:\n\
open_cam3d.exe --get-temperature --ip 192.168.x.x\n\
\n\
2.Get brightness image:\n\
open_cam3d.exe --get-brightness --ip 192.168.x.x --path ./brightness.bmp\n\
\n\
3.Get point cloud:\n\
open_cam3d.exe --get-pointcloud --ip 192.168.x.x --path ./point_cloud.xyz\n\
\n\
4.Get Frame 01:\n\
open_cam3d.exe --get-frame-01 --ip 192.168.x.x --path ./frame_01\n\
\n\
5.Get Frame 03:\n\
open_cam3d.exe --get-frame-03 --ip 192.168.x.x --path ./frame_03\n\
\n\
6.Get Frame 04:\n\
open_cam3d.exe --get-frame-04 --ip 192.168.x.x --path ./frame_03\n\
\n\
7.Get Frame 05:\n\
open_cam3d.exe --get-frame-05 --ip 192.168.x.x --path ./frame_05\n\
\n\
8.Get Frame Hdr:\n\
open_cam3d.exe --get-frame-hdr --ip 192.168.x.x --path ./frame_hdr\n\
\n\
9.Get calibration parameters: \n\
open_cam3d.exe --get-calib-param --ip 192.168.x.x --path ./param.txt\n\
\n\
10.Set calibration parameters: \n\
open_cam3d.exe --set-calib-param --ip 192.168.x.x --path ./param.txt\n\
\n\
11.Get raw images (Mode 01): \n\
open_cam3d.exe --get-raw-01 --ip 192.168.x.x --path ./raw01_image_dir\n\
\n\
12.Get raw images (Mode 02): \n\
open_cam3d.exe --get-raw-02 --ip 192.168.x.x --path ./raw02_image_dir\n\
\n\
13.Get raw images (Mode 03): \n\
open_cam3d.exe --get-raw-03 --ip 192.168.x.x --path ./raw03_image_dir\n\
\n\
14.Enable checkerboard: \n\
open_cam3d.exe --enable-checkerboard --ip 192.168.x.x\n\
\n\
15.Disable checkerboard: \n\
open_cam3d.exe --disable-checkerboard --ip 192.168.x.x\n\
\n\
16.Get Repetition Frame 03: \n\
open_cam3d.exe --get-repetition-frame-03 --count 6 --ip 192.168.x.x --path ./frame03_repetition\n\
\n\
17.Load pattern data: \n\
open_cam3d.exe --load-pattern-data --ip 192.168.x.x\n\
\n\
18.Program pattern data: \n\
open_cam3d.exe --program-pattern-data --ip 192.168.x.x\n\
\n\
19.Get network bandwidth: \n\
open_cam3d.exe --get-network-bandwidth --ip 192.168.x.x\n\
\n\
20.Get firmware version: \n\
open_cam3d.exe --get-firmware-version --ip 192.168.x.x\n\
\n\
21.Test camera calibration parameters: \n\
open_cam3d.exe --test-calib-param --use plane --ip 192.168.x.x --path ./capture\n\
\n\
22.Set calibration lookTable: \n\
open_cam3d.exe --set-calib-looktable --ip 192.168.x.x --path ./param.txt\n\
\n\
23.Set calibration minilookTable: \n\
open_cam3d.exe --set-calib-minilooktable --ip 192.168.x.x --path ./param.txt\n\
\n\
24.Set Generate Brightness Param: \n\
open_cam3d.exe --set-generate-brigntness-param --ip 192.168.x.x --model 1 --exposure 12000\n\
\n\
25.Get Generate Brightness Param: \n\
open_cam3d.exe --get-generate-brigntness-param --ip 192.168.x.x\n\
\n\
26.Set Camera Exposure Param: \n\
open_cam3d.exe --set-camera-exposure-param --ip 192.168.x.x --exposure 12000\n\
\n\
27.Get Camera Exposure Param: \n\
open_cam3d.exe --get-camera-exposure-param --ip 192.168.x.x\n\
\n\
28.Set Offset Param: \n\
open_cam3d.exe --set-offset-param --offset 12 --ip 192.168.x.x\n\
\n\
29.Get Camera Version: \n\
open_cam3d.exe --get-camera-version --ip 192.168.x.x\n\
\n\
30.Set Auto Exposure: \n\
open_cam3d.exe --set-auto-exposure-roi  --ip 192.168.x.x\n\
\n\
31.Self-test: \n\
open_cam3d.exe --self-test --ip 192.168.x.x\n\
\n\
32.Get projector temperature: \n\
open_cam3d.exe --get-projector-temperature --ip 192.168.x.x\n\
\n\
33.Get Repetition Phase 02: \n\
open_cam3d.exe --get-repetition-phase-02 --count 3 --ip 192.168.x.x --path  ./phase02_image_dir\n\
\n\
34.Enable Focusing: \n\
open_cam3d.exe --enable-focusing --ip 192.168.x.x \n\
\n\
35.Set Board Inspect: \n\
open_cam3d.exe --set-board-inspect --switch enable --ip 192.168.x.x \n\
\n\
36.Get Product Info: \n\
open_cam3d.exe --get-product-info --ip 192.168.x.x \n\
\n\
";

void help_with_version(const char* help);
bool depthTransformPointcloud(cv::Mat depth_map, cv::Mat& point_cloud_map);
int get_frame_01(const char* ip, const char* frame_path);
int get_frame_03(const char* ip, const char* frame_path);
int get_frame_04(const char* ip, const char* frame_path);
int get_frame_05(const char* ip, const char* frame_path);
int get_repetition_frame_03(const char* ip, int count, const char* frame_path);
int get_repetition_frame_04(const char* ip, int count, const char* frame_path);
int get_frame_hdr(const char* ip, const char* frame_path);
void save_frame(float* depth_buffer, unsigned char* bright_buffer, const char* frame_path);
void save_images(const char* raw_image_dir, unsigned char* buffer, int width, int height, int image_num);
void save_point_cloud(float* point_cloud_buffer, const char* pointcloud_path);
void save_color_point_cloud(float* point_cloud_buffer, unsigned char* brightness_buffer, const char* pointcloud_path);
void write_fbin(std::ofstream& out, float val);
void write_fbin(std::ofstream& out, unsigned char val);
bool SaveBinPointsToPly(cv::Mat deep_mat, string path, cv::Mat texture_map);
int on_dropped(void* param);
int get_brightness(const char* ip, const char* image_path);
int get_pointcloud(const char* ip, const char* pointcloud_path);
int get_raw_01(const char* ip, const char* raw_image_dir);
int get_raw_02(const char* ip, const char* raw_image_dir);
int get_raw_03(const char* ip, const char* raw_image_dir);
int get_raw_04(const char* ip, const char* raw_image_dir);
int get_raw_05(const char* ip, const char* raw_image_dir);
int get_raw_06(const char* ip, const char* raw_image_dir);
int get_raw_08(const char* ip, const char* raw_image_dir);
int get_calib_param(const char* ip, const char* calib_param_path);
int set_calib_param(const char* ip, const char* calib_param_path);
int set_generate_brightness_param(const char* ip, int model, float exposure);
int get_generate_brightness_param(const char* ip, int& model, float& exposure);
int set_camera_exposure_param(const char* ip, float exposure);
int get_camera_exposure_param(const char* ip, float& exposure);
int set_offset_param(const char* ip, float offset);
int set_calib_looktable(const char* ip, const char* calib_param_path);
int set_calib_minilooktable(const char* ip, const char* calib_param_path);
int test_calib_param(const char* ip, const char* result_path);
int get_temperature(const char* ip);
int enable_checkerboard(const char* ip);
int disable_checkerboard(const char* ip);
int load_pattern_data(const char* ip);
int program_pattern_data(const char* ip);
int get_network_bandwidth(const char* ip);
int get_firmware_version(const char* ip);
int get_camera_version(const char* ip);
int set_auto_exposure_base_roi(const char* ip);
int set_auto_exposure_base_board(const char* ip);
int self_test(const char* ip);
int get_projector_temperature(const char* ip);
int get_repetition_phase_02(const char* ip, int count, const char* phase_image_dir);
int configure_focusing(const char* ip);
int set_board_inspect(const char* ip,bool enable);
int get_product_info(const char* ip);

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	IP,
	PATH,
	COUNT,
	GET_TEMPERATURE,
	GET_CALIB_PARAM,
	SET_CALIB_PARAM,
	SET_CALIB_LOOKTABLE,
	SET_CALIB_MINILOOKTABLE,
	TEST_CALIB_PARAM,
	USE,
	GET_RAW_01,
	GET_RAW_02,
	GET_RAW_03,
	GET_RAW_04,
	GET_RAW_05,
	GET_RAW_06,
	GET_RAW_08,
	GET_POINTCLOUD,
	GET_FRAME_01,
	GET_FRAME_03,
	GET_FRAME_04,
	GET_FRAME_05,
	GET_REPETITION_FRAME_03,
	GET_REPETITION_FRAME_04,
	GET_FRAME_HDR,
	GET_BRIGHTNESS,
	HELP,
	ENABLE_CHECKER_BOARD,
	DISABLE_CHECKER_BOARD,
	LOAD_PATTERN_DATA,
	PROGRAM_PATTERN_DATA,
	GET_NETWORK_BANDWIDTH,
	GET_FIRMWARE_VERSION,
	SET_GENERATE_BRIGHTNESS,
	GET_GENERATE_BRIGHTNESS,
	MODEL,
	EXPOSURE,
	SET_CAMERA_EXPOSURE,
	GET_CAMERA_EXPOSURE,
	SET_OFFSET,
	OFFSET,
	GET_CAMERA_VERSION,
	SET_AUTO_EXPOSURE_BASE_ROI,
	SET_AUTO_EXPOSURE_BASE_BOARD,
	SELF_TEST,
	GET_PROJECTOR_TEMPERATURE,
	GET_REPETITION_PHASE_02,
	ENABLE_FOCUSING,
	SWITCH,
	SET_BOARD_INSPECT,
	GET_PRODUCT_INFO,
};

static struct option long_options[] =
{
	{"ip",required_argument,NULL,IP},
	{"path", required_argument, NULL, PATH},
	{"count", required_argument, NULL, COUNT},
	{"use", required_argument, NULL, USE},
	{"model", required_argument, NULL, MODEL},
	{"exposure", required_argument, NULL, EXPOSURE},
	{"offset", required_argument, NULL, OFFSET},
	{"switch", required_argument, NULL, SWITCH},
	{"get-temperature",no_argument,NULL,GET_TEMPERATURE},
	{"get-calib-param",no_argument,NULL,GET_CALIB_PARAM},
	{"set-calib-param",no_argument,NULL,SET_CALIB_PARAM},
	{"set-calib-looktable",no_argument,NULL,SET_CALIB_LOOKTABLE},
	{"set-calib-minilooktable",no_argument,NULL,SET_CALIB_MINILOOKTABLE},
	{"test-calib-param",no_argument,NULL,TEST_CALIB_PARAM},
	{"get-raw-01",no_argument,NULL,GET_RAW_01},
	{"get-raw-02",no_argument,NULL,GET_RAW_02},
	{"get-raw-03",no_argument,NULL,GET_RAW_03},
	{"get-raw-04",no_argument,NULL,GET_RAW_04},
	{"get-raw-05",no_argument,NULL,GET_RAW_05},
	{"get-raw-06",no_argument,NULL,GET_RAW_06},
	{"get-raw-08",no_argument,NULL,GET_RAW_08},
	{"get-repetition-phase-02",no_argument,NULL,GET_REPETITION_PHASE_02},
	{"get-pointcloud",no_argument,NULL,GET_POINTCLOUD},
	{"get-frame-01",no_argument,NULL,GET_FRAME_01},
	{"get-frame-03",no_argument,NULL,GET_FRAME_03},
	{"get-frame-04",no_argument,NULL,GET_FRAME_04},
	{"get-frame-05",no_argument,NULL,GET_FRAME_05},
	{"get-repetition-frame-03",no_argument,NULL,GET_REPETITION_FRAME_03},
	{"get-repetition-frame-04",no_argument,NULL,GET_REPETITION_FRAME_04},
	{"get-frame-hdr",no_argument,NULL,GET_FRAME_HDR},
	{"get-brightness",no_argument,NULL,GET_BRIGHTNESS},
	{"help",no_argument,NULL,HELP},
	{"enable-checkerboard",no_argument,NULL,ENABLE_CHECKER_BOARD},
	{"disable-checkerboard",no_argument,NULL,DISABLE_CHECKER_BOARD},
	{"load-pattern-data",no_argument,NULL,LOAD_PATTERN_DATA},
	{"program-pattern-data",no_argument,NULL,PROGRAM_PATTERN_DATA},
	{"get-network-bandwidth",no_argument,NULL,GET_NETWORK_BANDWIDTH},
	{"get-firmware-version",no_argument,NULL,GET_FIRMWARE_VERSION},
	{"set-generate-brightness-param",no_argument,NULL,SET_GENERATE_BRIGHTNESS},
	{"get-generate-brightness-param",no_argument,NULL,GET_GENERATE_BRIGHTNESS},
	{"set-camera-exposure-param",no_argument,NULL,SET_CAMERA_EXPOSURE},
	{"get-camera-exposure-param",no_argument,NULL,GET_CAMERA_EXPOSURE},
	{"set-offset-param",no_argument,NULL,SET_OFFSET},
	{"get-camera-version",no_argument,NULL,GET_CAMERA_VERSION},
	{"set-auto-exposure-roi",no_argument,NULL,SET_AUTO_EXPOSURE_BASE_ROI},
	{"set-auto-exposure-board",no_argument,NULL,SET_AUTO_EXPOSURE_BASE_BOARD},
	{"self-test",no_argument,NULL,SELF_TEST},
	{"get-projector-temperature",no_argument,NULL,GET_PROJECTOR_TEMPERATURE},
	{"enable-focusing",no_argument,NULL,ENABLE_FOCUSING},
	{"set-board-inspect",no_argument,NULL,SET_BOARD_INSPECT},
	{"get-product-info",no_argument,NULL,GET_PRODUCT_INFO},
};


const char* camera_id = NULL;
const char* path = NULL;
const char* repetition_count = NULL;
const char* use_command = NULL;
const char* c_model = NULL;
const char* c_exposure = NULL;
const char* c_offset = NULL;
const char* c_switch = NULL;
int command = HELP;

struct CameraCalibParam calibration_param_;


int main(int argc, char* argv[])
{


	int c = 0;

	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case IP:
			camera_id = optarg;
			break;
		case PATH:
			path = optarg;
			break;
		case COUNT:
			repetition_count = optarg;
			break;
		case USE:
			use_command = optarg;
			break;
		case MODEL:
			c_model = optarg;
			break;
		case EXPOSURE:
			c_exposure = optarg;
			break;
		case OFFSET:
			c_offset = optarg;
			break;
		case SWITCH:
			c_switch = optarg;
			break;
		case '?':
			printf("unknow option:%c\n", optopt);
			break;
		default:
			command = c;
			break;
		}
	}

	switch (command)
	{
	case HELP:
		help_with_version(help_info);
		break;
	case GET_TEMPERATURE:
		get_temperature(camera_id);
		break;
	case GET_CALIB_PARAM:
		get_calib_param(camera_id, path);
		break;
	case SET_CALIB_PARAM:
		set_calib_param(camera_id, path);
		break;
	case SET_CALIB_LOOKTABLE:
		set_calib_looktable(camera_id, path);
		break;
	case SET_CALIB_MINILOOKTABLE:
		set_calib_looktable(camera_id, path);
		break;
	case TEST_CALIB_PARAM:
		test_calib_param(camera_id, path);
		break;
	case GET_RAW_01:
		get_raw_01(camera_id, path);
		break;
	case GET_RAW_02:
		get_raw_02(camera_id, path);
		break;
	case GET_RAW_03:
		get_raw_03(camera_id, path);
		break;
	case GET_RAW_04:
		get_raw_04(camera_id, path);
		break;
	case GET_RAW_05:
		get_raw_05(camera_id, path);
		break;
	case GET_RAW_06:
		get_raw_06(camera_id, path);
		break;
	case GET_RAW_08:
		get_raw_08(camera_id, path);
		break;
	case GET_REPETITION_PHASE_02:
	{
		int num = std::atoi(repetition_count);
		get_repetition_phase_02(camera_id, num, path);
	}
	break;
	case GET_FRAME_01:
		get_frame_01(camera_id, path);
		break;
	case GET_FRAME_03:
		get_frame_03(camera_id, path);
		break;
	case GET_FRAME_04:
		get_frame_04(camera_id, path);
		break;
	case GET_FRAME_05:
		get_frame_05(camera_id, path);
		break;
	case GET_REPETITION_FRAME_03:
	{
		int num = std::atoi(repetition_count);
		get_repetition_frame_03(camera_id, num, path);
	}
	break;
	case GET_REPETITION_FRAME_04:
	{
		int num = std::atoi(repetition_count);
		get_repetition_frame_04(camera_id, num, path);
	}
	break;
	case GET_FRAME_HDR:
		get_frame_hdr(camera_id, path);
		break;
	case GET_POINTCLOUD:
		get_pointcloud(camera_id, path);
		break;
	case GET_BRIGHTNESS:
		get_brightness(camera_id, path);
		break;
	case ENABLE_CHECKER_BOARD:
		enable_checkerboard(camera_id);
		break;
	case DISABLE_CHECKER_BOARD:
		disable_checkerboard(camera_id);
		break;
	case LOAD_PATTERN_DATA:
		load_pattern_data(camera_id);
		break;
	case PROGRAM_PATTERN_DATA:
		program_pattern_data(camera_id);
		break;
	case GET_NETWORK_BANDWIDTH:
		get_network_bandwidth(camera_id);
		break;
	case GET_FIRMWARE_VERSION:
		get_firmware_version(camera_id);
		break;
	case SET_GENERATE_BRIGHTNESS:
	{
		int model = std::atoi(c_model);
		float exposure = std::atof(c_exposure);
		set_generate_brightness_param(camera_id, model, exposure);
	}
	break;
	case GET_GENERATE_BRIGHTNESS:
	{
		int model = 0;
		float exposure = 0;
		get_generate_brightness_param(camera_id, model, exposure);
	}
	break;
	case SET_CAMERA_EXPOSURE:
	{
		float exposure = std::atof(c_exposure);
		set_camera_exposure_param(camera_id, exposure);
	}
	break;
	case GET_CAMERA_EXPOSURE:
	{
		float exposure = 0;
		get_camera_exposure_param(camera_id, exposure);
	}
	break;
	case SET_OFFSET:
	{
		float offset = std::atof(c_offset);
		set_offset_param(camera_id, offset);
	}
	break;
	case GET_CAMERA_VERSION:
	{
		get_camera_version(camera_id);
	}
	break;
	case SET_AUTO_EXPOSURE_BASE_ROI:
	{
		set_auto_exposure_base_roi(camera_id);
	}
	break;
	case SET_AUTO_EXPOSURE_BASE_BOARD:
	{
		set_auto_exposure_base_board(camera_id);
	}
	break;
	case SELF_TEST:
		self_test(camera_id);
		break;
	case GET_PROJECTOR_TEMPERATURE:
		get_projector_temperature(camera_id);
		break;
	case ENABLE_FOCUSING:
		configure_focusing(camera_id);
		break;
	case SET_BOARD_INSPECT:
	{
		std::string switch_str(c_switch);
		if ("enable" == switch_str)
		{
			set_board_inspect(camera_id,true);
		}
		else if ("disable" == switch_str)
		{ 
			set_board_inspect(camera_id, false);
		}
	}
		break;
	case GET_PRODUCT_INFO:
		get_product_info(camera_id);
		break;
	default:
		break;
	}

	return 0;
}

void help_with_version(const char* help)
{
	char info[100 * 1024] = { '\0' };
	char version[] = _VERSION_;
	char enter[] = "\n";

#ifdef _WIN32 
	strcpy_s(info, sizeof(enter), enter);
	strcat_s(info, sizeof(info), version);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), help);


#elif __linux
	strncpy(info, enter, sizeof(enter));
	strncat(info, version, sizeof(info));
	strncat(info, enter, sizeof(info));
	strncat(info, enter, sizeof(info));
	strncat(info, help, sizeof(info));

#endif 


	printf(info);

}

int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}

bool depthTransformPointcloud(cv::Mat depth_map, cv::Mat& point_cloud_map)
{
	if (depth_map.empty())
	{
		return false;
	}

	double camera_fx = calibration_param_.camera_intrinsic[0];
	double camera_fy = calibration_param_.camera_intrinsic[4];

	double camera_cx = calibration_param_.camera_intrinsic[2];
	double camera_cy = calibration_param_.camera_intrinsic[5];

	float k1 = calibration_param_.camera_distortion[0];
	float k2 = calibration_param_.camera_distortion[1];
	float p1 = calibration_param_.camera_distortion[2];
	float p2 = calibration_param_.camera_distortion[3];
	float k3 = calibration_param_.camera_distortion[4];

	int nr = depth_map.rows;
	int nc = depth_map.cols;


	cv::Mat points_map(nr, nc, CV_32FC3, cv::Scalar(0, 0, 0));

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		float* ptr_d = depth_map.ptr<float>(r);
		cv::Vec3f* ptr_p = points_map.ptr<cv::Vec3f>(r);

		for (int c = 0; c < nc; c++)
		{
			double undistort_x = c;
			double undistort_y = r;
			if (ptr_d[c] > 0)
			{

				undistortPoint(c, r, camera_fx, camera_fy,
					camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

				cv::Point3f p;
				p.z = ptr_d[c];

				p.x = (undistort_x - camera_cx) * p.z / camera_fx;
				p.y = (undistort_y - camera_cy) * p.z / camera_fy;


				ptr_p[c][0] = p.x;
				ptr_p[c][1] = p.y;
				ptr_p[c][2] = p.z;
			}


		}


	}

	point_cloud_map = points_map.clone();

	return true;
}

void save_frame(float* depth_buffer, unsigned char* bright_buffer, const char* frame_path)
{
	std::string folderPath = frame_path;

	cv::Mat depth_map(1200, 1920, CV_32F, depth_buffer);
	cv::Mat bright_map(1200, 1920, CV_8U, bright_buffer);


	std::string depth_path = folderPath + ".tiff";
	cv::imwrite(depth_path, depth_map);
	std::cout << "save depth: " << depth_path << "\n";

	std::string bright_path = folderPath + ".bmp";
	cv::imwrite(bright_path, bright_map);
	std::cout << "save brightness: " << bright_path << "\n";

	cv::Mat point_cloud_map;
	depthTransformPointcloud(depth_map, point_cloud_map);


	std::string xyz_pointcloud_path = folderPath + ".xyz";
	std::string ply_pointcloud_path = folderPath + ".ply";

	save_color_point_cloud((float*)point_cloud_map.data, (unsigned char*)bright_map.data, xyz_pointcloud_path.c_str());
	SaveBinPointsToPly(point_cloud_map, ply_pointcloud_path, bright_map);
	std::cout << "save point cloud: " << xyz_pointcloud_path << ", " << ply_pointcloud_path << "\n";

	//struct CameraCalibParam calibration_param;
	//DfGetCalibrationParam(calibration_param);

}

void save_images(const char* raw_image_dir, unsigned char* buffer, int width, int height, int image_num)
{
	std::string folderPath = raw_image_dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	int image_size = width * height;

	for (int i = 0; i < image_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(height, width, CV_8UC1, buffer + (long)(image_size * i));
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "/phase" + ss.str() + ".bmp";
		cv::imwrite(filename, image);
	}
}

void save_color_point_cloud(float* point_cloud_buffer, unsigned char* brightness_buffer, const char* pointcloud_path)
{
	std::ofstream ofile;
	ofile.open(pointcloud_path);
	for (int i = 0; i < 1920 * 1200; i++)
	{
		if (point_cloud_buffer[i * 3 + 2] > 0.01)
			ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << " "
			<< (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << std::endl;
	}
	ofile.close();
}

inline void write_fbin(std::ofstream& out, float val) {
	out.write(reinterpret_cast<char*>(&val), sizeof(float));
}

inline void write_fbin(std::ofstream& out, unsigned char val) {
	out.write(reinterpret_cast<char*>(&val), sizeof(unsigned char));
}

//保存Bin点云到ply文件
bool SaveBinPointsToPly(cv::Mat deep_mat, string path, cv::Mat texture_map)
{

	if (deep_mat.empty())
	{
		return false;
	}

	if (path.empty())
	{
		return false;
	}

	std::ofstream file;
	file.open(path);
	if (!file.is_open())
	{
		std::cout << "Save points Error";
		return false;
	}
	else
	{

		if (texture_map.data)
		{

			std::vector<cv::Vec3f> points_list;
			std::vector<cv::Vec3b> color_list;



			if (1 == texture_map.channels())
			{

				/****************************************************************************************************/

				string str = "";

				if (CV_32FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
						uchar* ptr_color = texture_map.ptr<uchar>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{

								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c];
								color[1] = ptr_color[c];
								color[2] = ptr_color[c];

								points_list.push_back(point);
								color_list.push_back(color);
							}

						}
					}
				}
				else if (CV_64FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
						uchar* ptr_color = texture_map.ptr<uchar>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c];
								color[1] = ptr_color[c];
								color[2] = ptr_color[c];

								points_list.push_back(point);
								color_list.push_back(color);

							}

						}
					}
				}

				/*****************************************************************************************************/
			}
			else if (3 == texture_map.channels())
			{
				/****************************************************************************************************/

				string str = "";

				if (CV_32FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
						cv::Vec3b* ptr_color = texture_map.ptr<cv::Vec3b>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c][2];
								color[1] = ptr_color[c][1];
								color[2] = ptr_color[c][0];

								points_list.push_back(point);
								color_list.push_back(color);

							}

						}
					}
				}
				else if (CV_64FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
						cv::Vec3b* ptr_color = texture_map.ptr<cv::Vec3b>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{
								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c][2];
								color[1] = ptr_color[c][1];
								color[2] = ptr_color[c][0];

								points_list.push_back(point);
								color_list.push_back(color);

							}

						}
					}
				}

				/*****************************************************************************************************/
			}


			//Header 
			file << "ply" << "\n";
			file << "format binary_little_endian 1.0" << "\n";
			file << "element vertex " << points_list.size() << "\n";
			file << "property float x" << "\n";
			file << "property float y" << "\n";
			file << "property float z" << "\n";
			file << "property uchar red" << "\n";
			file << "property uchar green" << "\n";
			file << "property uchar blue" << "\n";
			file << "end_header" << "\n";

			file.close();


			/*********************************************************************************************************/
			//以二进行制保存
			std::ofstream outFile(path, std::ios::app | std::ios::binary);


			for (int i = 0; i < points_list.size(); i++)
			{
				write_fbin(outFile, static_cast<float>(points_list[i][0]));
				write_fbin(outFile, static_cast<float>(points_list[i][1]));
				write_fbin(outFile, static_cast<float>(points_list[i][2]));
				write_fbin(outFile, static_cast<unsigned char>(color_list[i][0]));
				write_fbin(outFile, static_cast<unsigned char>(color_list[i][1]));
				write_fbin(outFile, static_cast<unsigned char>(color_list[i][2]));
			}


			outFile.close();


			/**********************************************************************************************************/

		}
		else
		{

			std::vector<cv::Vec3f> points_list;

			if (CV_32FC3 == deep_mat.type())
			{
				for (int r = 0; r < deep_mat.rows; r++)
				{
					cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
					for (int c = 0; c < deep_mat.cols; c++)
					{
						if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
						{

							cv::Vec3f point;

							point[0] = ptr_dr[c][0];
							point[1] = ptr_dr[c][1];
							point[2] = ptr_dr[c][2];
							points_list.push_back(point);

						}

					}
				}
			}
			else if (CV_64FC3 == deep_mat.type())
			{
				for (int r = 0; r < deep_mat.rows; r++)
				{
					cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
					for (int c = 0; c < deep_mat.cols; c++)
					{
						if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
						{


							cv::Vec3f point;

							point[0] = ptr_dr[c][0];
							point[1] = ptr_dr[c][1];
							point[2] = ptr_dr[c][2];
							points_list.push_back(point);

						}

					}
				}
			}

			//Header 
			file << "ply" << "\n";
			file << "format binary_little_endian 1.0" << "\n";
			file << "element vertex " << points_list.size() << "\n";
			file << "property float x" << "\n";
			file << "property float y" << "\n";
			file << "property float z" << "\n";
			file << "end_header" << "\n";
			file.close();

			/*********************************************************************************************************/
			//以二进行制保存
			std::ofstream outFile(path, std::ios::app | std::ios::binary);

			for (int i = 0; i < points_list.size(); i++)
			{
				write_fbin(outFile, static_cast<float>(points_list[i][0]));
				write_fbin(outFile, static_cast<float>(points_list[i][1]));
				write_fbin(outFile, static_cast<float>(points_list[i][2]));
			}


			outFile.close();


			/**********************************************************************************************************/
		}

		std::cout << "Save points" << path;
	}

	return true;
}

void save_point_cloud(float* point_cloud_buffer, const char* pointcloud_path)
{
	std::ofstream ofile;
	ofile.open(pointcloud_path);
	for (int i = 0; i < 1920 * 1200; i++)
	{
		if (point_cloud_buffer[i * 3 + 2] > 0.01)
			ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << std::endl;
	}
	ofile.close();
}

int get_brightness(const char* ip, const char* image_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnect(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);
	int image_size = width * height;


	cv::Mat brightness;

	XemaColor color;
	int pixel_type;
	ret = DfGetCameraPixelType(pixel_type);
	if (ret == DF_SUCCESS)
	{
		if (pixel_type == (int)XemaPixelType::Mono)
		{
			color = XemaColor::Gray;
			std::cout << "gray" << std::endl;

			brightness = cv::Mat(cv::Size(width, height), CV_8U, cv::Scalar(0));
		}
		else
		{
			color = XemaColor::Rgb;
			std::cout << "rgb" << std::endl;
			brightness = cv::Mat(cv::Size(width, height), CV_8UC3, cv::Scalar(0,0,0));
		} 
	}
	else if (ret == DF_UNKNOWN)
	{
		color = XemaColor::Gray;
		std::cout << "gray" << std::endl;
		brightness = cv::Mat(cv::Size(width, height), CV_8U, cv::Scalar(0));
	}

	 

	unsigned char* brightness_buf = (unsigned char*)brightness.data;
	 
	//ret = DfGetCameraData(0, 0,
	//	brightness_buf, image_size,
	//	0, 0,
	//	0, 0);
	ret = DfCaptureBrightnessData(brightness_buf, color);
	if (ret != DF_SUCCESS)
	{
		std::cout << "capture brightness failed!"<<std::endl;
	}

	cv::imwrite(image_path, brightness);
	std::cout << "save brightness: " << image_path << std::endl;


	DfDisconnectNet();
	return 1;
}

int get_frame_hdr(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrameHdr(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);



	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

	DfDisconnectNet();



	return 1;
}


int get_repetition_frame_04(const char* ip, int count, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetRepetitionFrame04(count, depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

}
int get_repetition_frame_03(const char* ip, int count, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetRepetitionFrame03(count, depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

}

int get_frame_03(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame03(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;



	return 1;
}

int get_frame_04(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame04(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;



	return 1;
}

int get_frame_05(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame05(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;



	return 1;
}


int get_frame_01(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame01(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);



	save_frame(depth_buf, brightness_buf, frame_path);



	delete[] depth_buf;
	delete[] brightness_buf;

	DfDisconnectNet();

	return 1;
}

int get_pointcloud(const char* ip, const char* pointcloud_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int point_cloud_buf_size = image_size * 3 * 4;
	float* point_cloud_buf = (float*)(new char[point_cloud_buf_size]);

	ret = DfGetPointCloud(point_cloud_buf, point_cloud_buf_size);

	save_point_cloud(point_cloud_buf, pointcloud_path);

	delete[] point_cloud_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_01(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int capture_num = 24;

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawData01(raw_buf, image_size * capture_num);

	save_images(raw_image_dir, raw_buf, width, height, capture_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;

}


int get_raw_04(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int patterns_num = 19;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * patterns_num)];

	ret = DfGetCameraRawData04(raw_buf, image_size * patterns_num);

	save_images(raw_image_dir, raw_buf, width, height, patterns_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_05(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int patterns_num = 16;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * patterns_num)];

	ret = DfGetCameraRawData05(raw_buf, image_size * patterns_num);

	save_images(raw_image_dir, raw_buf, width, height, patterns_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_06(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int patterns_num = 16;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * patterns_num)];

	ret = DfGetCameraRawData06(raw_buf, image_size * patterns_num);

	save_images(raw_image_dir, raw_buf, width, height, patterns_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_08(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	int patterns_num = 24;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * patterns_num)];

	ret = DfGetCameraRawData08(raw_buf, image_size * patterns_num);

	save_images(raw_image_dir, raw_buf, width, height, patterns_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_03(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * 31)];

	ret = DfGetCameraRawData03(raw_buf, image_size * 31);

	save_images(raw_image_dir, raw_buf, width, height, 31);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_02(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	int capture_num = 37;

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[(long)(image_size * capture_num)];

	ret = DfGetCameraRawDataTest(raw_buf, image_size * capture_num);

	save_images(raw_image_dir, raw_buf, width, height, capture_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}


int get_repetition_phase_02(const char* ip, int count, const char* phase_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);


	int image_size = width * height;

	cv::Mat phase_x(height, width, CV_32F, cv::Scalar(0));
	cv::Mat phase_y(height, width, CV_32F, cv::Scalar(0));
	cv::Mat brightness(height, width, CV_8U, cv::Scalar(0));

	ret = DfGetRepetitionPhase02(count, (float*)phase_x.data, (float*)phase_y.data, image_size * sizeof(float), brightness.data, image_size * sizeof(char));

	DfDisconnectNet();

	DfSolution solution_machine_;
	std::string folderPath = std::string(phase_image_dir);
	folderPath = solution_machine_.replaceAll(folderPath, "/", "\\");
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());


	std::string phase_x_str = folderPath + "\\01.tiff";
	cv::imwrite(phase_x_str, phase_x);
	std::string phase_y_str = folderPath + "\\02.tiff";
	cv::imwrite(phase_y_str, phase_y);
	std::string brightness_str = folderPath + "\\03.bmp";
	cv::imwrite(brightness_str, brightness);

	return 1;
}

int test_calib_param(const char* ip, const char* result_path)
{
	std::string cmd(use_command);

	if ("plane" == cmd)
	{
		std::cout << "plane" << std::endl;

		struct CameraCalibParam calibration_param_;
		DfSolution solution_machine_;
		std::vector<cv::Mat> patterns_;

		bool ret = solution_machine_.captureMixedVariableWavelengthPatterns(std::string(ip), patterns_);

		if (!ret)
		{
			std::cout << "采集图像出错！" << std::endl;
			return false;
		}

		ret = solution_machine_.getCameraCalibData(std::string(ip), calibration_param_);

		if (!ret)
		{
			std::cout << "获取标定数据出错！" << std::endl;
		}


		solution_machine_.testCalibrationParamBasePlane(patterns_, calibration_param_, std::string(result_path));
	}
	else if ("board" == cmd)
	{
		std::cout << "board" << std::endl;

		struct CameraCalibParam calibration_param_;
		DfSolution solution_machine_;
		std::vector<cv::Mat> patterns_;

		bool ret = solution_machine_.captureMixedVariableWavelengthPatterns(std::string(ip), patterns_);

		if (!ret)
		{
			std::cout << "采集图像出错！" << std::endl;
			return false;
		}

		ret = solution_machine_.getCameraCalibData(std::string(ip), calibration_param_);

		if (!ret)
		{
			std::cout << "获取标定数据出错！" << std::endl;
		}


		solution_machine_.testCalibrationParamBaseBoard(patterns_, calibration_param_, std::string(result_path));
	}

	return 1;
}

int get_calib_param(const char* ip, const char* calib_param_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	struct CameraCalibParam calibration_param;
	DfGetCalibrationParam(calibration_param);
	std::ofstream ofile;
	ofile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ofile << ((float*)(&calibration_param))[i] << std::endl;
	}
	ofile.close();

	DfDisconnectNet();
	return 1;
}


int set_calib_looktable(const char* ip, const char* calib_param_path)
{
	/*************************************************************************************************/

	struct CameraCalibParam calibration_param;
	std::ifstream ifile;
	ifile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ifile >> ((float*)(&calibration_param))[i];
		std::cout << ((float*)(&calibration_param))[i] << std::endl;
	}
	ifile.close();
	std::cout << "Read Param" << std::endl;

	/**************************************************************************************************/

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret != DF_SUCCESS)
	{
		std::cout << "Connect failed!" << std::endl;
		return 0;
	}


	int width = 0;
	int height = 0;

	ret = DfGetCameraResolution(&width, &height);
	if (ret != DF_SUCCESS)
	{
		std::cout << "DfGetCameraResolution failed!" << std::endl;
		return 0;
	}
	std::cout << "width: " << width << std::endl;
	std::cout << "height: " << height << std::endl;

	int version = 0;
	ret = DfGetProjectorVersion(version);
	if (ret != DF_SUCCESS)
	{
		std::cout << "DfGetProjectorVersion failed!" << std::endl;
		return -1;
	}

	std::cout << "test version: " << version << std::endl;
	ret = DfDisconnectNet();

	MiniLookupTableFunction minilooktable_machine;
	minilooktable_machine.setCameraResolution(width, height);
	
	if (!minilooktable_machine.setProjectorVersion(version))
	{
		std::cout << "Set Projector Version failed!" << std::endl;
		std::cout << "version: " << version<< std::endl;
		return -1;
	}


	minilooktable_machine.setCalibData(calibration_param);
	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat rectify_R1;
	cv::Mat pattern_mapping;
	cv::Mat pattern_minimapping;


	std::cout << "Start Generate LookTable Param" << std::endl;
	//bool ok = minilooktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping);
	bool ok = minilooktable_machine.generateBigLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_mapping, pattern_minimapping);
	std::cout << "Finished Generate LookTable Param: " << ok << std::endl;

	xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
	xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
	rectify_R1.convertTo(rectify_R1, CV_32F);
	pattern_mapping.convertTo(pattern_mapping, CV_32F);
	pattern_minimapping.convertTo(pattern_minimapping, CV_32F);

	cv::Mat filling_map(pattern_mapping.size(), CV_8U, cv::Scalar(0));

	for (int i = 0; i < pattern_mapping.rows; i++)
	{
		for (int j = 0; j < pattern_mapping.cols; j++)
		{
			if (pattern_mapping.at<float>(i, j) != -2.)
			{
				filling_map.at<uchar>(i, j) = 255;
			}
		}
	}

	std::cout << "filling_map.rows: " << filling_map.rows << std::endl;
	std::cout << "filling_map.cols: " << filling_map.cols << std::endl;
	cv::imwrite("../pattern_mapping.bmp", filling_map);
	cv::imwrite("../pattern_mapping.tiff", pattern_mapping);
	DfRegisterOnDropped(on_dropped);

	ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfSetCalibrationLookTable(calibration_param, (float*)xL_rotate_x.data, (float*)xL_rotate_y.data, (float*)rectify_R1.data,
		(float*)pattern_mapping.data, (float*)pattern_minimapping.data, width, height);



	DfDisconnectNet();
	return 1;
}

int set_calib_minilooktable(const char* ip, const char* calib_param_path)
{
	/*************************************************************************************************/

	struct CameraCalibParam calibration_param;
	std::ifstream ifile;
	ifile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ifile >> ((float*)(&calibration_param))[i];
		std::cout << ((float*)(&calibration_param))[i] << std::endl;
	}
	ifile.close();
	std::cout << "Read Param" << std::endl;
	MiniLookupTableFunction looktable_machine;
	looktable_machine.setCalibData(calibration_param);
	//looktable_machine.readCalibData(calib_param_path);
	cv::Mat xL_rotate_x;
	cv::Mat xL_rotate_y;
	cv::Mat rectify_R1;
	cv::Mat pattern_minimapping;

	std::cout << "Start Generate LookTable Param" << std::endl;
	bool ok = looktable_machine.generateLookTable(xL_rotate_x, xL_rotate_y, rectify_R1, pattern_minimapping);

	std::cout << "Finished Generate LookTable Param: " << ok << std::endl;

	xL_rotate_x.convertTo(xL_rotate_x, CV_32F);
	xL_rotate_y.convertTo(xL_rotate_y, CV_32F);
	rectify_R1.convertTo(rectify_R1, CV_32F);
	pattern_minimapping.convertTo(pattern_minimapping, CV_32F);
	std::cout << "生成压缩查找表成功！！！" << pattern_minimapping.size() << std::endl;
	/**************************************************************************************************/

	DfRegisterOnDropped(on_dropped);



	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		std::cout << "相机连接失败，程序退出" << std::endl;
		return 0;
	}



	DfSetCalibrationMiniLookTable(calibration_param, (float*)xL_rotate_x.data, (float*)xL_rotate_y.data, (float*)rectify_R1.data, (float*)pattern_minimapping.data);



	DfDisconnectNet();
	return 1;
}

int get_camera_exposure_param(const char* ip, float& exposure)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfGetParamCameraExposure(exposure);


	DfDisconnectNet();


	std::cout << "camera exposure: " << exposure << std::endl;
}


int set_offset_param(const char* ip, float offset)
{
	if (offset < 0)
	{
		std::cout << "offset param out of range!" << std::endl;
		return 0;
	}


	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfSetParamOffset(offset);


	DfDisconnectNet();


	std::cout << "offset: " << offset << std::endl;

	return 1;
}

int set_camera_exposure_param(const char* ip, float exposure)
{
	if (exposure < 20 || exposure> 1000000)
	{
		std::cout << "exposure param out of range!" << std::endl;
		return 0;
	}


	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfSetParamCameraExposure(exposure);


	DfDisconnectNet();


	std::cout << "camera exposure: " << exposure << std::endl;

	return 1;
}


int get_generate_brightness_param(const char* ip, int& model, float& exposure)
{

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	DfGetParamGenerateBrightness(model, exposure);

	DfDisconnectNet();


	std::cout << "model: " << model << std::endl;
	std::cout << "exposure: " << exposure << std::endl;
	return 1;
}

int set_generate_brightness_param(const char* ip, int model, float exposure)
{
	if (exposure < 20 || exposure> 1000000)
	{
		std::cout << "exposure param out of range!" << std::endl;
		return 0;
	}

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	DfSetParamGenerateBrightness(model, exposure);


	DfDisconnectNet();

	std::cout << "model: " << model << std::endl;
	std::cout << "exposure: " << exposure << std::endl;
	return 1;
}

int set_calib_param(const char* ip, const char* calib_param_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	struct CameraCalibParam calibration_param;
	std::ifstream ifile;
	ifile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ifile >> ((float*)(&calibration_param))[i];
		std::cout << ((float*)(&calibration_param))[i] << std::endl;
	}
	ifile.close();

	DfSetCalibrationParam(calibration_param);

	DfDisconnectNet();
	return 1;
}

int get_temperature(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	float temperature = 0;
	DfGetDeviceTemperature(temperature);
	std::cout << "Device temperature: " << temperature << std::endl;

	DfDisconnectNet();
	return 1;
}

int set_board_inspect(const char* ip, bool enable)
{

	/*********************************************************************************************/
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}
 
	DfSetBoardInspect(enable);

	DfDisconnectNet();

}

int configure_focusing(const char* ip)
{

	cv::namedWindow("focusing", cv::WINDOW_NORMAL);
	cv::resizeWindow("focusing", 960, 600);
	cv::imshow("focusing", cv::Mat(1200, 1920, CV_8UC1, cv::Scalar(0)));
	cv::waitKey(5);

	enable_checkerboard(camera_id);
	clock_t startTime, endTime;
	startTime = clock();//��ʱ��ʼ




	/*********************************************************************************************/
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetCameraResolution(&width, &height);

	ret = DfSetParamLedCurrent(255);
	//cv::waitKey(10);
	if (DF_SUCCESS != ret)
	{
		std::cout << "Set Led Failed! " << std::endl;
		return 0;
	}

	int image_size = width * height;
	cv::Mat img(height, width, CV_8UC1, cv::Scalar(0));


	float f_val = 0;
	DfGetParamCameraExposure(f_val);
	int exposure = f_val / 1000;
	cv::createTrackbar("Exposure", "focusing", &exposure, 60);


	int num = 1000;
	int overheating_num = 0;
	while (num-- > 0)
	{

		int val = cv::getTrackbarPos("Exposure", "focusing");

		//if (val != exposure)
		//{

		ret = DfSetParamCameraExposure(val * 1000);
		//cv::waitKey(10);
		if (DF_SUCCESS == ret)
		{
			exposure = val;
		}
		//}

		DfGetFocusingImage(img.data, image_size * sizeof(char));

		int pixel_type = 0;
		DfGetCameraPixelType(pixel_type);

		if (pixel_type == (int)XemaPixelType::BayerRG8)
		{

			cv::Mat color_img(height, width, CV_8UC3, cv::Scalar(0));
			DfBayerToRgb(img.data, color_img.data);
			DfRgbToGray(color_img.data, img.data);
		}

		float temperature = 0;
		DfGetProjectorTemperature(temperature);

		if (temperature > 75)
		{
			std::cout << "too high temperature: " << temperature << " degree" << std::endl;
			overheating_num++;
			//break;
		}

		if (overheating_num > 50)
		{
			break;
		}

		std::string str_val = std::to_string(temperature);
		std::string title = "Focusing   T: " + str_val.substr(0, str_val.find(".") + 3) + " ℃";
		cv::setWindowTitle("focusing", title);

		cv::imshow("focusing", img);
		int key = cv::waitKey(1);
		//std::cout << "temperature: " << temperature << std::endl;
		//std::cout << "exposure: " << exposure << std::endl;
		//std::cout << "key: " << key << std::endl;
		if (32 == key || 27 == key)
		{
			break;
		}

		endTime = clock();//��ʱ����
		int count_sec = (endTime - startTime) / CLOCKS_PER_SEC;
		std::cout << "The run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

		if (count_sec > 600)
		{
			break;
		}

	}


	DfDisconnectNet();





	disable_checkerboard(camera_id);


	cv::destroyAllWindows();

	return 1;
}

// -------------------------------------------------------------------
// -- enable and disable checkerboard, by wangtong, 2022-01-27
int enable_checkerboard(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	float temperature = 0;
	DfEnableCheckerboard(temperature);
	std::cout << "Enable checkerboard: " << temperature << std::endl;

	DfDisconnectNet();
	return 1;
}

int disable_checkerboard(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	float temperature = 0;
	DfDisableCheckerboard(temperature);
	std::cout << "Disable checkerboard: " << temperature << std::endl;

	DfDisconnectNet();
	return 1;
}

// -------------------------------------------------------------------
#define PATTERN_DATA_SIZE 0xA318						// Now the pattern data build size is 0xA318 = 41752 bytes.
int load_pattern_data(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	char* LoadBuffer = new char[PATTERN_DATA_SIZE];

	DfLoadPatternData(PATTERN_DATA_SIZE, LoadBuffer);

	char string[50] = { '\0' };
	FILE* fw;
	if (fopen_s(&fw, "pattern_data.dat", "wb") == 0) {
		fwrite(LoadBuffer, 1, PATTERN_DATA_SIZE, fw);
		fclose(fw);

#ifdef _WIN32 

		sprintf_s(string, sizeof("pattern_data.dat"), "pattern_data.dat");
#elif __linux

		snprintf(string, sizeof("pattern_data.dat"), "pattern_data.dat");
#endif 





	}
	else {

#ifdef _WIN32 

		sprintf_s(string, sizeof("save pattern data fail"), "save pattern data fail");
#elif __linux

		snprintf(string, sizeof("save pattern data fail"), "save pattern data fail");
#endif 



	}

	std::cout << "Load Pattern save as:" << string << std::endl;

	delete[] LoadBuffer;

	DfDisconnectNet();
	return 1;
}

int program_pattern_data(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	// allocate the org pattern data & read back data buffer.
	char* pOrg = new char[PATTERN_DATA_SIZE];
	char* pBack = new char[PATTERN_DATA_SIZE];

	// read the pattern data from file into the front half of the buffer.
	FILE* fw;
	if (fopen_s(&fw, "pattern_data.dat", "rb") == 0) {

#ifdef _WIN32  
		fread_s(pOrg, PATTERN_DATA_SIZE, 1, PATTERN_DATA_SIZE, fw);
#elif __linux
		fread(pOrg, PATTERN_DATA_SIZE, PATTERN_DATA_SIZE, fw);
#endif 

		fclose(fw);
		std::cout << "Program Pattern:" << "load file ok!" << std::endl;
	}
	else {
		std::cout << "Program Pattern:" << "load file fail..." << std::endl;
	}

	DfProgramPatternData(pOrg, pBack, PATTERN_DATA_SIZE);

	// check the program and load data be the same.
	if (memcmp(pOrg, pBack, PATTERN_DATA_SIZE)) {
		std::cout << "Program Pattern:" << "fail..." << std::endl;
	}
	else {
		std::cout << "Program Pattern:" << "ok!" << std::endl;
	}

	delete[] pOrg;
	delete[] pBack;

	DfDisconnectNet();
	return 1;
}

int get_network_bandwidth(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int speed = 0;
	DfGetNetworkBandwidth(speed);
	std::cout << "Network bandwidth: " << speed << std::endl;

	DfDisconnectNet();
	return 1;
}

int get_firmware_version(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	char version[_VERSION_LENGTH_] = { '\0' };
	DfGetFirmwareVersion(version, _VERSION_LENGTH_);
	std::cout << "Firmware: " << version << std::endl;

	DfDisconnectNet();
	return 1;
}

int get_product_info(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED) {
		return 0;
	}

	char* info = new char[INFO_SIZE];
	DfGetProductInfo(info, INFO_SIZE);
	std::cout << "Product Info: " << info << std::endl;

	delete[] info;
	DfDisconnectNet();
	return 1;
}

int set_auto_exposure_base_roi(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int exposure = 0;
	int led = 0;

	DfSetAutoExposure(1, exposure, led);

	DfDisconnectNet();

	std::cout << "exposure: " << exposure << std::endl;
	std::cout << "led: " << led << std::endl;

	return 1;
}

int set_auto_exposure_base_board(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int exposure = 0;
	int led = 0;

	DfSetAutoExposure(2, exposure, led);

	DfDisconnectNet();

	std::cout << "exposure: " << exposure << std::endl;
	std::cout << "led: " << led << std::endl;

	return 1;
}

int get_camera_version(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int version = 0;

	DfGetCameraVersion(version);

	DfDisconnectNet();

	std::cout << "Camera Version: " << version << std::endl;
	return 1;
}

int self_test(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED) {
		return 0;
	}

	char test[500] = { '\0' };
	DfSelfTest(test, sizeof(test));
	std::cout << "Self-test: " << test << std::endl;

	DfDisconnectNet();
	return 1;
}

int get_projector_temperature(const char* ip)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	float temperature = 0;
	DfGetProjectorTemperature(temperature);
	std::cout << "Projector temperature: " << temperature << std::endl;

	DfDisconnectNet();
	return 1;
}
