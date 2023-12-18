// Dexforce_DF_Source.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#ifdef _WIN32  
#include <windows.h>
#elif __linux 
#endif
#include <iostream>
#include "solution.h"
#include "../firmware/camera_param.h"
#include "../cmd/getopt.h"
#include <string.h>  
#include <iomanip>

const char* help_info =
"Examples:\n\
\n\
1.Capture:\n\
test.exe --capture --ip 192.168.x.x --model patterns-04 --patterns ./patterns_data --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
2.Read:\n\
test.exe --read --model patterns-04 --patterns ./patterns_data  --calib ./param.txt --version DFX800  --pointcloud ./pointcloud_data\n\
\n\
3.Capture:\n\
test.exe --capture --ip 192.168.x.x --model patterns-03 --patterns ./patterns_data --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
4.Read:\n\
test.exe --read --model patterns-03 --patterns ./patterns_data  --calib ./param.txt --version DFX800  --pointcloud ./pointcloud_data\n\
\n\
5.Read:\n\
test.exe --reconstruct --use look-table --patterns ./patterns_data  --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
6.Read:\n\
test.exe --read --model phase-02 --patterns ./patterns_data  --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
7.Read:\n\
test.exe --reconstruct --use minilook-table --patterns ./patterns_data  --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
8.Read:\n\
test.exe --read --model patterns-04-c --patterns ./patterns_data  --calib ./param.txt --version DFX800  --pointcloud ./pointcloud_data\n\
\n\
9.Read(generate point cloud base confidence level:[-50, 150]):\n\
test.exe --reconstruct --use look-table --confidence 50 --patterns ./patterns_data  --calib ./param.txt --version DFX800 --pointcloud ./pointcloud_data\n\
\n\
";

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	IP,
	CAPTURE,
	READ,
	RECONSTRUCT,
	USE,
	PATTERNS,
	CALIB,
	POINTCLOUD,
	VERSION,
	HELP,
	MODEL,
	REPETITION,
	CONFIDENCE
};

static struct option long_options[] =
{
	{"ip",required_argument,NULL,IP},
	{"capture",no_argument,NULL,CAPTURE},
	{"read",no_argument,NULL,READ},
	{"reconstruct",no_argument,NULL,RECONSTRUCT},
	{"use", required_argument, NULL, USE},
	{"patterns", required_argument, NULL, PATTERNS},
	{"calib", required_argument, NULL, CALIB},
	{"pointcloud", required_argument, NULL, POINTCLOUD},
	{"version", required_argument, NULL, VERSION},
	{"help",no_argument,NULL,HELP},
	{"model", required_argument, NULL, MODEL},
	{"repetition", required_argument, NULL, REPETITION},
	{"confidence", required_argument, NULL, CONFIDENCE},
};

const char* camera_ip = "";
const char* patterns_path = "";
const char* calib_path = "";
const char* pointcloud_path = "";
const char* use_type = "look-table";
const char* char_version = "";
const char* c_model = "patterns-04";
const char* c_repetition_count = "2";
const char* confidence_level = "";
int command = HELP;


void capture_03();
void capture_04();
void read_03();
void read_04();
void read_05();
void read_04_color();
void read_06();
void read_test();
void generate_patterns_06();
void capture_04_repetition_02(int repetition);
void read_04_repetition_02();
void capture_04_repetition_01(int repetition);
void read_04_repetition_01();
void reconstruct_base_looktable();
void reconstruct_base_looktable_and_confidence();
void read_phase_02();
void reconstruct_base_minilooktable();

const char* camera_id;
const char* path;

int version_number = 0;

int main(int argc, char* argv[])
{
	int c = 0;


	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case IP:
			camera_ip = optarg;
			break;
		case PATTERNS:
			patterns_path = optarg;
			break;
		case CALIB:
			calib_path = optarg;
			break;
		case POINTCLOUD:
			pointcloud_path = optarg;
			break;
		case USE:
			use_type = optarg;
			break;
		case CONFIDENCE:
			confidence_level = optarg;
			break;
		case VERSION:
		{
			char_version = optarg;

			std::string version_str(char_version);

			if ("3010" == version_str)
			{
				version_number = 3010;
			}
			else if ("4710" == version_str )
			{
				version_number = 4710;
			}
		}
		break;
		case MODEL:
			c_model = optarg;
			break;
		case REPETITION:
			c_repetition_count = optarg;
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
		printf(help_info);
		break;
	case  CAPTURE:
	{
		std::string model(c_model);
		if ("patterns-03" == model)
		{
			capture_03();
		}
		else if ("patterns-04" == model)
		{
			capture_04();
		}
		else if ("patterns-04-repetition-02" == model)
		{
			int num = std::atoi(c_repetition_count);
			capture_04_repetition_02(num);
		}
		else if ("patterns-04-repetition-01" == model)
		{
			int num = std::atoi(c_repetition_count);
			capture_04_repetition_01(num);
		}
	}
	break;
	case READ:
	{
		std::string model(c_model);
		if ("patterns-03" == model)
		{
			read_03();
		}
		else if ("patterns-04" == model)
		{
			read_04();
		}
		else if ("patterns-05" == model)
		{
			read_05();
		}
		else if ("patterns-04-c" == model)
		{
			read_04_color();
		}
		else if ("patterns-06" == model)
		{
			read_06();
		}
		else if ("patterns-test" == model)
		{
			read_test();
		}
		else if ("gen-patterns-06" == model)
		{
			generate_patterns_06();
		}
		else if ("patterns-04-repetition-02" == model)
		{
			read_04_repetition_02();
		}
		else if ("patterns-04-repetition-01" == model)
		{
			read_04_repetition_01();
		}
		else if ("phase-02" == model)
		{
			read_phase_02();
		}

	}
	break;
	case RECONSTRUCT:
	{
		std::string cmd(use_type);
		if (confidence_level == "")
		{
			if ("look-table" == cmd)
			{
				reconstruct_base_looktable();
			}
			else if ("minilook-table" == cmd)
			{
				reconstruct_base_minilooktable();
			}
		}
		else
		{
			if ("look-table" == cmd)
			{
				reconstruct_base_looktable_and_confidence();
			}
		}


	}
	break;
	default:
		break;
	}

	/*************************************************************************************************/

}


void capture_04_repetition_01(int repetition)
{

	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	std::string folderPath = std::string(patterns_path);
	folderPath = solution_machine_.replaceAll(folderPath, "/", "\\");

	std::string del_cmd = std::string("rd /s/q ") + folderPath;
	system(del_cmd.c_str());

	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	bool ret = false;

	ret = solution_machine_.captureModel04RepetitionPatterns(camera_ip, repetition, patterns_);

	solution_machine_.savePatterns(folderPath, patterns_);



	ret = solution_machine_.getCameraCalibData(camera_ip, calibration_param_);

	if (ret)
	{
		solution_machine_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructPatterns04Repetition01BaseTable(patterns_, calibration_param_, pointcloud_path);

	//solution_machine_.reconstructPatterns04RepetitionBaseTable(patterns_list_, calibration_param_, pointcloud_path);
}

void capture_04_repetition_02(int repetition)
{

	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;
	std::vector<std::vector<cv::Mat>> patterns_list_;

	std::string folderPath = std::string(patterns_path);
	folderPath = solution_machine_.replaceAll(folderPath, "/", "\\");

	std::string del_cmd = std::string("rd /s/q ") + folderPath;
	system(del_cmd.c_str());

	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	bool ret = false;

	for (int i = 0; i < repetition; i++)
	{
		ret = solution_machine_.captureModel04Patterns(camera_ip, patterns_);

		if (ret)
		{
			patterns_list_.push_back(patterns_);
		}
	}


	for (int i = 0; i < patterns_list_.size(); i++)
	{
		std::stringstream ss;
		ss << std::setw(2) << std::setfill('0') << i;
		std::string path = folderPath + "\\" + ss.str();
		solution_machine_.savePatterns(path, patterns_list_[i]);
	}


	ret = solution_machine_.getCameraCalibData(camera_ip, calibration_param_);

	if (ret)
	{
		solution_machine_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}


	solution_machine_.reconstructPatterns04RepetitionBaseTable(patterns_list_, calibration_param_, pointcloud_path);
}


void capture_04()
{

	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	std::string folderPath = std::string(patterns_path);
	folderPath = solution_machine_.replaceAll(folderPath, "/", "\\");

	std::string del_cmd = std::string("rd /s/q ") + folderPath;
	system(del_cmd.c_str());

	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	bool ret = solution_machine_.captureModel04Patterns(camera_ip, patterns_);

	if (ret)
	{
		solution_machine_.savePatterns(patterns_path, patterns_);
	}

	ret = solution_machine_.getCameraCalibData(camera_ip, calibration_param_);

	if (ret)
	{
		solution_machine_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}


	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseTable(patterns_, calibration_param_, pointcloud_path);
}

void capture_03()
{

	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	std::string folderPath = std::string(patterns_path);
	folderPath = solution_machine_.replaceAll(folderPath, "/", "\\");

	std::string del_cmd = std::string("rd /s/q ") + folderPath;
	system(del_cmd.c_str());

	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	bool ret = solution_machine_.captureMixedVariableWavelengthPatterns(camera_ip, patterns_);

	if (ret)
	{
		solution_machine_.savePatterns(patterns_path, patterns_);
	}

	ret = solution_machine_.getCameraCalibData(camera_ip, calibration_param_);

	if (ret)
	{
		solution_machine_.saveCameraCalibData(calib_path, calibration_param_);
	}
	else
	{
		std::cout << "Get Camera Calib Data Failure!";
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_, pointcloud_path);
}

void reconstruct_base_looktable()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}


	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseTable(patterns_, calibration_param_, pointcloud_path);
}

void reconstruct_base_looktable_and_confidence()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.confidence_level_ = std::atoi(confidence_level);
	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseTableAndConfidence(patterns_, calibration_param_, pointcloud_path);
}


void read_phase_02()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;

	cv::Mat phase_x = cv::imread(std::string(patterns_path) + "\\01.tiff", -1);
	cv::Mat phase_y = cv::imread(std::string(patterns_path) + "\\02.tiff", -1);
	cv::Mat brightness = cv::imread(std::string(patterns_path) + "\\03.bmp", 0);


	if (phase_x.empty() || phase_y.empty() || brightness.empty())
	{
		std::cout << "Read Image Error!";
	}


	bool ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructBasePhase02(phase_x, phase_y, brightness, calibration_param_, pointcloud_path);
}


void read_03()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_, pointcloud_path);
}


void reconstruct_base_minilooktable()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}




	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseMiniTable(patterns_, calibration_param_, pointcloud_path);
}


void read()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructMixedVariableWavelengthPatternsBaseXYSR(patterns_, calibration_param_, pointcloud_path);
}


void read_04_repetition_01()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructPatterns04Repetition01BaseTable(patterns_, calibration_param_, pointcloud_path);
}

void read_04_repetition_02()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;

	std::vector <std::vector<cv::Mat>> patterns_list_;
	std::cout << "patterns_path: " << patterns_path << std::endl;
	bool ret = solution_machine_.readFolderImages(patterns_path, patterns_list_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}


	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	solution_machine_.reconstructPatterns04RepetitionBaseTable(patterns_list_, calibration_param_, pointcloud_path);

}


void generate_patterns_06()
{
	/**************************************************************************************/

	//DfSolution solution_machine_;
	//std::vector<cv::Mat> patterns_;
	//bool ret = solution_machine_.readImages(patterns_path, patterns_);

	//if (!ret)
	//{
	//	std::cout << "Read Image Error!";
	//}

	//std::vector<cv::Mat> patterns_list;
	//std::vector<cv::Mat> inv_patterns_list;

	//int width = patterns_[0].cols;
	//int height = patterns_[0].rows;

	//for (int p_i = 0; p_i < patterns_.size(); p_i++)
	//{
	//	cv::Mat temple = patterns_[p_i];

	//	cv::Mat gen_pattern(height, width * 5,CV_8U,cv::Scalar(0));

	//	for (int r = 0; r < height; r++)
	//	{
	//		for (int c = 0; c < width; c++)
	//		{
	//			uchar val = temple.at<uchar>(r, c);

	//			for (int i = 0; i < 5; i++)
	//			{
	//				gen_pattern.at<uchar>(r, c * 5 + i) = val;
	//			}
	//		}
	//	}

	//	cv::Mat pattern_3010(720, 1280, CV_8U, cv::Scalar(0));

	//	for (int r = 0; r < pattern_3010.rows; r++)
	//	{

	//		for (int c = 0; c < pattern_3010.cols; c++)
	//		{
	//			pattern_3010.at<uchar>(r, c) = gen_pattern.at<uchar>(r, c);
	//		}
	//	}

	//	cv::Mat inv_pattern_3010 = 255 - pattern_3010;

	//	patterns_list.push_back(pattern_3010.clone());
	//	inv_patterns_list.push_back(inv_pattern_3010.clone());

	//}


	//for (int i = 0; i < patterns_list.size(); i++)
	//{ 
	//	std::stringstream ss; 
	//	ss << std::setw(2) << std::setfill('0') << i;
	//	std::string filename = "../minsw_" + ss.str() + ".bmp";
	//	bool ret = cv::imwrite(filename, patterns_list[i]);
	//	std::cout << "save: " << filename << " " << ret << std::endl;

	//	std::string filename_inv = "../minsw_inv_" + ss.str() + ".bmp";
	//	ret = cv::imwrite(filename_inv, inv_patterns_list[i]);
	//	std::cout << "save: " << filename_inv << " " << ret << std::endl;
	//}

	/*******************************************************************************************************/
	std::vector<int> list{ 0, 1, 3, 7, 15, 31, 30, 62, 126, 254, 246, 247, 245, 213, 209, 145,
		153, 152, 136, 8, 40, 42, 43, 35, 99, 103, 71, 70, 68, 76, 204, 220, 252, 253, 189, 185,
		177, 179, 178, 146, 210, 82, 90, 91, 75, 107, 111, 109, 101, 100, 36, 164, 132, 134, 135,
		143, 159, 155, 187, 186, 250, 242, 114, 112, 80, 81, 17, 21, 29, 13, 12, 44, 46, 174, 166,
		167, 231, 199, 195, 193, 201, 200, 216, 88, 120, 56, 57, 49, 51, 55, 23, 22, 86, 94, 222,
		206, 238, 239, 237, 233, 225, 161, 160, 128, 130, 2, 10, 11, 27, 59, 63, 127, 119, 118, 116,
		244, 212, 148, 149, 157, 141, 137, 169, 168, 170, 162, 34, 98, 66, 67, 65, 69, 77, 93, 92,
		124, 60, 188, 180, 181, 183, 151, 147, 211, 219, 218, 202, 74, 106, 104, 105, 97, 33, 37,
		5, 4, 6, 14, 142, 158, 190, 191, 255, 251, 243, 241, 240, 208, 144, 16, 24, 25, 9, 41, 45,
		47, 39, 38, 102, 230, 198, 196, 197, 205, 221, 217, 249, 248, 184, 176, 48, 50, 18, 19, 83,
		87, 95, 79, 78, 110, 108, 236, 228, 229, 165, 133, 129, 131, 139, 138, 154, 26, 58, 122, 123,
		115, 113, 117, 85, 84, 20, 28, 156, 140, 172, 173, 175, 171, 163, 227, 226, 194, 192, 64, 72,
		73, 89, 121, 125, 61, 53, 52, 54, 182, 150, 214, 215, 223, 207, 203, 235, 234, 232, 224, 96, 32 };

	cv::Mat bin_map(8, 256, CV_8U, cv::Scalar(0));

	for (int c = 0; c < bin_map.cols; c++)
	{
		int bit_num = 8;
		std::vector<int> bit_list;
		int bin_code = list[c];

		for (int b_i = 0; b_i < bit_num; b_i++)
		{
			int bin_val = bin_code / ((int)std::pow(2, bit_num - b_i - 1));
			bin_code = bin_code % ((int)std::pow(2, bit_num - b_i - 1));
			//std::cout << bin_val << std::endl;
			bit_list.push_back(255*bin_val);
		}


		for (int r = 0; r < bin_map.rows; r++)
		{
			bin_map.at<uchar>(r, c) = bit_list[r];
		}
	}



	std::vector<cv::Mat> minsw_8_patterns;

	for(int r = 0;r< bin_map.rows;r++)
	{
		cv::Mat pattern(720, 1280, CV_8U, cv::Scalar(0));

		for (int c = 0; c < bin_map.cols; c++)
		{
			for (int p_r = 0; p_r < pattern.rows; p_r++)
			{
				for (int rep = 0; rep < 5; rep++)
				{
					pattern.at<uchar>(p_r, 5 * c + rep) = bin_map.at<uchar>(r,c);

				}
			}

		}

		minsw_8_patterns.push_back(pattern.clone());
	
	}


	cv::imwrite("../bin_map_minsw8.bmp", bin_map);
	for (int i = 0; i < minsw_8_patterns.size(); i++)
{ 
	std::stringstream ss; 
	ss << std::setw(2) << std::setfill('0') << i;
	std::string filename = "../minsw8_" + ss.str() + ".bmp";
	bool ret = cv::imwrite(filename, minsw_8_patterns[i]);
	std::cout << "save: " << filename << " " << ret << std::endl;

 
}

}


void read_test()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}


	solution_machine_.reconstructPatternsTest(patterns_, calibration_param_, pointcloud_path);
}

void read_06()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}


	solution_machine_.reconstructPatterns06(patterns_, calibration_param_, pointcloud_path);
}


void read_04_color()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readColorImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}


	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseTable(patterns_, calibration_param_, pointcloud_path);
}

void read_05()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}

	solution_machine_.reconstructPatterns05(patterns_, calibration_param_, pointcloud_path);
}

void read_04()
{
	struct CameraCalibParam calibration_param_;
	DfSolution solution_machine_;
	std::vector<cv::Mat> patterns_;

	bool ret = solution_machine_.readImages(patterns_path, patterns_);

	if (!ret)
	{
		std::cout << "Read Image Error!";
	}

	ret = solution_machine_.readCameraCalibData(calib_path, calibration_param_);

	if (!ret)
	{
		std::cout << "Read Calib Param Error!" << std::endl;
	}

	ret = solution_machine_.setProjectorVersion(version_number);
	if (!ret)
	{
		std::cout << "Set Camera Version Error!" << std::endl;
		return;
	}
	 
	solution_machine_.reconstructMixedVariableWavelengthXPatternsBaseTable(patterns_, calibration_param_, pointcloud_path);
}