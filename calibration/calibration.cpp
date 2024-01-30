// open_cam3d_calibration.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>  
#include <string.h>
#include "../test/encode.h"   
#include "../test/support_function.h"
#include "calibrate_function.h"
#include "../cmd/getopt.h"
#include "../firmware/version.h"

const char* help_info =
"Examples:\n\
\n\
1.Calibrate:\n\
calibration.exe --calibrate --use patterns --path ./capture/calib --version 3010 --board 20 --calib ./param.txt \n\
\n\
2.Get Version:\n\
calibration.exe --get-version \n\
\n\
3.Calibrate:\n\
calibration.exe --calibrate --use phase --path ./capture/phase --version 3010 --board 20 --calib ./param.txt \n\
\n\
4.Calibrate:\n\
calibration.exe --calibrate --use patterns-c --path ./capture/calib --version 3010 --board 20 --calib ./param.txt \n\
\n\
5.Correct:\n\
calibration.exe --correct --use patterns --path ./capture/calib --version 3010 --board 20 --param-in ./param.txt --param-out ./param.txt \n\
6.Correct:\n\
calibration.exe --correct --use patterns-c --path ./capture/calib --version 3010 --board 20 --param-in ./param.txt --param-out ./param.txt \n\
\n\
";

int dlp_width = 1920;
int dlp_height = 1080;

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	CALIBRATE,
	CORRECT,
	PATH,
	USE,
	CALIB,
	VERSION,
	GET_VERSION,
	BOARD,
	HELP,
	PARAM_IN,
	PARAM_OUT,
};

static struct option long_options[] =
{
	{"calibrate",no_argument,NULL,CALIBRATE},
	{"correct",no_argument,NULL,CORRECT},
	{"get-version", no_argument, NULL, GET_VERSION},
	{"path", required_argument, NULL, PATH},
	{"use", required_argument, NULL, USE},
	{"calib", required_argument, NULL, CALIB},
	{"version", required_argument, NULL, VERSION},
	{"board", required_argument, NULL, BOARD},
	{"help",no_argument,NULL,HELP},
	{"param-in", required_argument, NULL, PARAM_IN},
	{"param-out", required_argument, NULL, PARAM_OUT},
};


//struct BoardMessage
//{
//	int rows;
//	int cols;
//	int width;
//	int height;
//};
struct BoardMessage board_message;
const char* patterns_path = "";
const char* char_use = "";
const char* calib_path = "";
const char* char_version = "";
const char* char_board = "";
const char* param_in_path = "";
const char* param_out_path = "";
int command = HELP;

void project_version();
bool calibrate_stereo(std::string patterns_path, std::string calib_path);
bool correct_stereo(std::string patterns_path, std::string in_path, std::string out_path);
bool correct_stereo_color(std::string patterns_path, std::string in_path, std::string out_path);
bool calibrate_stereo_base_phase(std::string phase_path, std::string calib_path);
bool calibrate_stereo_color(std::string patterns_path, std::string calib_path);

int main(int argc, char* argv[])
{

	int c = 0;

	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case PATH:
			patterns_path = optarg;
			break;
		case CALIB:
			calib_path = optarg;
			break;
		case PARAM_IN:
			param_in_path = optarg;
			break;
		case PARAM_OUT:
			param_out_path = optarg;
			break;
		case VERSION:
			char_version = optarg;
			break;
		case BOARD:
			char_board = optarg;
			break;
		case USE:
			char_use = optarg;
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
	case GET_VERSION:
		project_version();
		break;
	case  CALIBRATE:
	{
		std::string patterns_str(patterns_path);
		std::string calib_str(calib_path);
		std::string version_str(char_version);
		std::string board_str(char_board);
		std::string use_str(char_use);


		if (patterns_str.empty() || calib_str.empty() || version_str.empty())
		{
			printf("calibration.exe --calibrate --path ./capture/calib --version 3010 --board 20 ./calib --calib ./param.txt \n");
		}

		if ("3010" == version_str)
		{
			dlp_width = 1280;
			dlp_height = 720;

			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 20;
			board_message.height = 10;

		}
		else if ("4710" == version_str)
		{
			dlp_width = 1920;
			dlp_height = 1080;

			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 40;
			board_message.height = 20;
		}
		else
		{
			printf("calibration.exe --calibrate --use patterns --path ./capture/calib --version 3010 --board 20 ./calib --calib ./param.txt \n");
		}

		if ("4" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 4;
			board_message.height = 2;
		}
		if ("12" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 12;
			board_message.height = 6;
		}
		else if ("20" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 20;
			board_message.height = 10;
		}
		else if ("40" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 40;
			board_message.height = 20;
		}
		else if ("80" == board_str)
		{
			board_message.rows = 13;
			board_message.cols = 9;
			board_message.width = 80;
			board_message.height = 40;
		}

		if ("patterns" == use_str)
		{
			bool ok = calibrate_stereo(patterns_str, calib_str);

			if (!ok)
			{
				printf("calibration.exe --calibrate --use patterns --path ./capture/calib --version 3010 --board 20 ./calib --calib ./param.txt \n");
			}
			else
			{
				std::cout << version_str << " Calibrate Finished! " << std::endl;

			}
		}
		else if ("patterns-c" == use_str)
		{
			bool ok = calibrate_stereo_color(patterns_str, calib_str);

			if (!ok)
			{
				printf("calibration.exe --calibrate --use patterns-c --path ./capture/calib --version 3010 --board 20 ./calib --calib ./param.txt \n");
			}
			else
			{
				std::cout << version_str << " Calibrate Finished! " << std::endl;

			}
		}
		else if ("phase" == use_str)
		{
			bool ok = calibrate_stereo_base_phase(patterns_str, calib_str);

			if (!ok)
			{
				printf("calibration.exe --calibrate --use phase --path ./capture/calib --version 3010 --board 20 ./calib --calib ./param.txt \n");
			}
			else
			{
				std::cout << version_str << " Calibrate Finished! " << std::endl;

			}
		}





	}
	break;
	case  CORRECT:
	{  
		std::string patterns_str(patterns_path);
		std::string param_in_str(param_in_path);
		std::string param_out_str(param_out_path);
		std::string version_str(char_version);
		std::string board_str(char_board);  
		std::string use_str(char_use);

		if (patterns_str.empty() || param_in_str.empty() || param_out_str.empty() || version_str.empty())
		{
			printf("calibration.exe --correct --use patterns --path ./capture/calib --version 3010 --board 20 --param-in ./param.txt --param-out ./param.txt \n");
		}


		if ("3010" == version_str)
		{
			dlp_width = 1280;
			dlp_height = 720;

			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 20;
			board_message.height = 10;

		}
		else if ("4710" == version_str)
		{
			dlp_width = 1920;
			dlp_height = 1080;

			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 40;
			board_message.height = 20;
		}
		else
		{
			printf("calibration.exe --correct --use patterns --path ./capture/calib --version 3010 --board 20 --param-in ./param.txt --param-out ./param.txt \n");
		}

		if ("4" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 4;
			board_message.height = 2;
		}
		if ("12" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 12;
			board_message.height = 6;
		}
		else if ("20" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 20;
			board_message.height = 10;
		}
		else if ("40" == board_str)
		{
			board_message.rows = 11;
			board_message.cols = 7;
			board_message.width = 40;
			board_message.height = 20;
		}
		else if ("80" == board_str)
		{
			board_message.rows = 13;
			board_message.cols = 9;
			board_message.width = 80;
			board_message.height = 40;
		}
		 
		 
		if ("patterns" == use_str)
		{
			bool ok = correct_stereo(patterns_str, param_in_str, param_out_str);

			if (!ok)
			{
				printf("calibration.exe --correct --use patterns --path ./capture/calib --version 3010 --board 20 --param-in ./param.txt --param-out ./param.txt \n");
			}
			else
			{
				std::cout << version_str << " Calibrate Finished! " << std::endl;

			}
		}
		else if ("patterns-c" == use_str)
		{ 
			bool ok = correct_stereo_color(patterns_str, param_in_str, param_out_str);

			if (!ok)
			{
				printf("calibration.exe --correct --use patterns-c --path ./capture/calib --version 3010 --board 20 --param-in ./param.txt --param-out ./param.txt \n");
			}
			else
			{
				std::cout << version_str << " Calibrate Finished! " << std::endl;

			}
		}


	}
	break;

	default:
		printf(help_info);
		break;
	}

	return 1;
}

void project_version()
{
	char info[100 * 1024] = { '\0' };
	char version[] = _VERSION_;
	char enter[] = "\n";
#ifdef _WIN32 
	strcpy_s(info, sizeof(enter), enter);
	strcat_s(info, sizeof(info), version);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), enter);
#elif __linux
	strncpy(info, enter, sizeof(enter));
	strncat(info, version, sizeof(info));
	strncat(info, enter, sizeof(info));
	strncat(info, enter, sizeof(info));
#endif 
	printf(info);
}


bool calibrate_stereo_base_phase(std::string phase_path, std::string calib_path)
{
	std::string path = phase_path;
	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message.rows, board_message.cols, board_message.width, board_message.height);

	//读取多组标定条纹图案图像

	std::vector<std::string> current_folder_list;
	getJustCurrentDir(path, current_folder_list);

	if (current_folder_list.empty())
	{
		std::cout << "Read Images error!" << std::endl;
		return false;
	}

	std::cout << "Start Read Board Images...... " << std::endl;

	//文件夹路径
	std::vector<std::string> folder_list;

	//读取标定板图像
	std::vector<cv::Mat> board_images_list;
	std::vector<cv::Mat> phase_x_list;
	std::vector<cv::Mat> phase_y_list;


	std::vector<std::vector<cv::Point2f>> board_points_list;

	for (int b_i = 0; b_i < current_folder_list.size(); b_i++)
	{

		cv::Mat board_img = cv::imread(current_folder_list[b_i] + "\\03.bmp", 0);
		if (board_img.empty())
		{
			continue;
		}

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(board_img, circle_points);


		if (found)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(board_img, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);

			std::vector<std::string> str_list = vStringSplit(current_folder_list[b_i], "/");

			std::string board_path = path + "/" + std::to_string(b_i) + "_draw.bmp";
			cv::imwrite(board_path, color_img);
			if (0 == calib_function.testOverExposure(board_img, circle_points))
			{
				std::cout << "over exposure: " << current_folder_list[b_i] << std::endl;
			}

			board_points_list.push_back(circle_points);

		}

		/*****************************************************************************************/

		cv::Mat phase_x = cv::imread(current_folder_list[b_i] + "\\01.tiff", -1);
		if (phase_x.empty())
		{
			continue;
		}


		cv::Mat phase_y = cv::imread(current_folder_list[b_i] + "\\02.tiff", -1);
		if (phase_y.empty())
		{
			continue;
		}

		board_images_list.push_back(board_img);
		phase_x_list.push_back(phase_x);
		phase_y_list.push_back(phase_y);
		folder_list.push_back(current_folder_list[b_i]);
		//std::cout << "read data: " << current_folder_list[b_i] << std::endl;
	}



	if (board_points_list.size() < 6)
	{
		std::cout << "Valid Board Number: " << board_points_list.size() << std::endl;
		return false;
	}

	std::cout << "Valid Board Number: " << board_points_list.size() << std::endl;
	std::cout << "Start Calibrate Camera...... " << std::endl;

	//标定相机并根据重投影误差筛选图片组
	std::map<int, bool> select_group;
	double err = calib_function.calibrateCamera(board_points_list, select_group);


	std::cout << "Calibrate Camera Error: " << err << std::endl;
	std::cout << "Camera Select Group Number: " << select_group.size() << std::endl;


	std::vector<std::vector<cv::Point2f>> select_board_points_list;
	std::vector<cv::Mat> select_phase_x_list;
	std::vector<cv::Mat> select_phase_y_list;

	std::vector<std::vector<cv::Point2f>> select_dlp_points_list_base_phase;

	for (int g_i = 0; g_i < select_group.size(); g_i++)
	{
		if (select_group[g_i])
		{
			cv::Mat unwrap_ver = phase_x_list[g_i];
			cv::Mat unwrap_hor = phase_y_list[g_i];

			unwrap_ver.convertTo(unwrap_ver, CV_64F);
			unwrap_hor.convertTo(unwrap_hor, CV_64F);

			//相机转换至投影仪

			std::vector<cv::Point2f> dlp_points;

			calib_function.fillThePhaseHole(unwrap_hor, true);
			calib_function.fillThePhaseHole(unwrap_ver, false);

			bool ret = calib_function.cameraPointsToDlp(board_points_list[g_i], unwrap_hor, unwrap_ver, 1, dlp_width, dlp_height, dlp_points);

			if (!ret)
			{
				std::cout << "Bad Patterns: " << folder_list[g_i] << std::endl;
			}
			else
			{
				select_board_points_list.push_back(board_points_list[g_i]);
				select_dlp_points_list_base_phase.push_back(dlp_points);

			}
		}
	}


	std::cout << "Vaild Phase Group Number: " << select_dlp_points_list_base_phase.size() << std::endl;

	std::cout << "Start Calibrate Projector...... " << std::endl;
	//投影仪标定并筛选图像组
	std::map<int, bool> select_group_dlp;
	err = calib_function.calibrateProjector(select_dlp_points_list_base_phase, select_group_dlp);


	std::cout << "Calibrate Projector Error: " << err << std::endl;


	std::vector<std::vector<cv::Point2f>> select_dlp_points_list;
	std::vector<std::vector<cv::Point2f>> select_camera_points_list;

	for (int g_i = 0; g_i < select_group_dlp.size(); g_i++)
	{
		if (select_group_dlp[g_i])
		{
			select_dlp_points_list.push_back(select_dlp_points_list_base_phase[g_i]);
			select_camera_points_list.push_back(select_board_points_list[g_i]);
		}
	}



	std::cout << "Projector Select Group Number: " << select_dlp_points_list.size() << std::endl;
	std::cout << "Start Calibrate Stereo...... " << std::endl;
	//对筛选出来的图像进行立体标定
	double stereo_err = calib_function.calibrateStereo(select_camera_points_list, select_dlp_points_list, calib_path);


	std::cout << "Reprojection Error: " << stereo_err << std::endl;
	std::cout << "Reprojection Error should be less than 0.1......" << std::endl;


}


bool calibrate_stereo_color(std::string patterns_path, std::string calib_path)
{
	std::string path = patterns_path;
	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message.rows, board_message.cols, board_message.width, board_message.height);

	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(path, files_list);

	std::vector<std::string> current_folder_list;
	getJustCurrentDir(path, current_folder_list);

	if (files_list.empty())
	{
		std::cout << "Read Images error!" << std::endl;
		return false;
	}

	std::cout << "Start Read Board Images...... " << std::endl;

	//文件夹路径
	std::vector<std::string> folder_list;

	//读取标定板图像
	std::vector<cv::Mat> board_images_list;
	for (int b_i = 0; b_i < files_list.size(); b_i++)
	{
		std::vector<std::string> g_image_list = files_list[b_i];

		cv::Mat board_img = cv::imread(g_image_list.back(), 0);
		board_images_list.push_back(board_img);
		folder_list.push_back(current_folder_list[b_i]);
	}

	std::cout << "Board Images Number: " << board_images_list.size() << std::endl;
	std::cout << "Start Find Board...... " << std::endl;

	//查找标定板初步筛选图片组
	std::vector<std::vector<std::string>> select_images_path_list_base_board;
	std::vector<cv::Mat> select_board_images_list;
	std::vector<std::vector<cv::Point2f>> board_points_list;

	for (int g_i = 0; g_i < files_list.size(); g_i++)
	{
		std::vector<std::string> g_image_list = files_list[g_i];
		cv::Mat bayer = board_images_list[g_i];

		cv::Mat img;
		cv::cvtColor(bayer, img, cv::COLOR_BayerBG2BGR);
		std::vector<cv::Mat> bgr_chanels;
		cv::split(img, bgr_chanels);
		img = bgr_chanels[0];

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(img, circle_points);

		std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
		cv::imwrite(img_path, img);

		if (found)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(img, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);

			std::vector<std::string> str_list = vStringSplit(folder_list[g_i], "/");

			std::string board_path = path + "/" + std::to_string(g_i) + "_draw.bmp";
			cv::imwrite(board_path, color_img);


			if (0 == calib_function.testOverExposure(img, circle_points))
			{
				std::cout << "over exposure: " << folder_list[g_i] << std::endl;
			}

			select_images_path_list_base_board.push_back(g_image_list);
			board_points_list.push_back(circle_points);
			select_board_images_list.push_back(img);

		}


	}

	if (select_images_path_list_base_board.size() < 6)
	{
		std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
		return false;
	}

	std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
	std::cout << "Start Calibrate Camera...... " << std::endl;

	//标定相机并根据重投影误差筛选图片组
	std::map<int, bool> select_group;
	double err = calib_function.calibrateCamera(board_points_list, select_group);


	std::cout << "Calibrate Camera Error: " << err << std::endl;
	std::cout << "Camera Select Group Number: " << select_group.size() << std::endl;

	std::vector<std::vector<std::string>> select_images_path_list_base_camera;
	std::vector<std::vector<cv::Point2f>> select_board_points_list;
	std::vector<std::vector<cv::Point2f>> dlp_points_list;
	std::vector<std::vector<cv::Point3f>> world_points_list;
	std::vector<std::string> select_folder_list;
	std::vector<cv::Mat> select_board_images_list_base_camera;

	for (int g_i = 0; g_i < select_group.size(); g_i++)
	{
		if (select_group[g_i])
		{
			select_images_path_list_base_camera.push_back(select_images_path_list_base_board[g_i]);
			select_board_points_list.push_back(board_points_list[g_i]);
			select_folder_list.push_back(folder_list[g_i]);
			select_board_images_list_base_camera.push_back(select_board_images_list[g_i]);
		}
	}

	/*****************************************************************************************************/

	/*****************************************************************************************************/

	std::cout << "Start Read Pattern Image...... " << std::endl;

	//读取筛选出来的组图像
	std::vector<std::vector<cv::Mat>> group_image_list;
	for (int g_i = 0; g_i < select_images_path_list_base_camera.size(); g_i++)
	{
		std::vector<std::string> path_list = select_images_path_list_base_camera[g_i];

		std::vector<cv::Mat> img_list;

		for (int n_i = 0; n_i < path_list.size() - 1; n_i++)
		{
			cv::Mat bayer = cv::imread(path_list[n_i], 0);

			cv::Mat img;
			convertBayer2Blue(bayer, img);

			img_list.push_back(img);
		}

		group_image_list.push_back(img_list);
	}


	std::cout << "Read Pattern Finished." << std::endl;
	std::cout << "Start Compute Phase ......" << std::endl;
	/******************************************************************************************************************/
	//依据相位值选择
	std::vector<bool> select_group_flag_base_phase;
	DF_Encode encode_machine_;


	//相位计算、相位展开、坐标转换
	for (int g_i = 0; g_i < group_image_list.size(); g_i++)
	{
		std::vector<cv::Mat> wrap_img_list;
		std::vector<cv::Mat> img_list = group_image_list[g_i];
		cv::Mat board_img = board_images_list[g_i];

		/*****************************************************************************************************************************************/
		//4+4+4+6
		int vertical_four_num = 12;
		int vertical_six_num = 6;
		int horizontal_four_num = 12;
		int horizontal_six_num = 6;

		std::vector<cv::Mat>::const_iterator list_ptr = img_list.begin();

		std::vector<cv::Mat> ver_patterns_img_4(list_ptr, list_ptr + vertical_four_num);
		std::vector<cv::Mat> ver_patterns_img_6(list_ptr + vertical_four_num, list_ptr + vertical_four_num + vertical_six_num);


		list_ptr = img_list.begin() + +img_list.size() / 2;

		std::vector<cv::Mat> hor_patterns_img_4(list_ptr, list_ptr + horizontal_four_num);
		std::vector<cv::Mat> hor_patterns_img_6(list_ptr + horizontal_four_num, list_ptr + horizontal_four_num + vertical_six_num);


		std::vector<cv::Mat> ver_wrap_img_4;
		std::vector<cv::Mat> hor_wrap_img_4;
		cv::Mat ver_confidence_map_4;
		cv::Mat hor_confidence_map_4;

		std::vector<cv::Mat> ver_wrap_img_6;
		std::vector<cv::Mat> hor_wrap_img_6;
		cv::Mat ver_confidence_map_6;
		cv::Mat hor_confidence_map_6;

		bool ret = true;

		cv::Mat test_mask_ = cv::Mat();

		DF_Encode encode_machine_;


		ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
		ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


		ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
		ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

		std::vector<double> variable_wrap_rate;
		variable_wrap_rate.push_back(8);
		variable_wrap_rate.push_back(4);
		variable_wrap_rate.push_back(4);


		cv::Mat unwrap_mask = test_mask_.clone();

		int discard_num = 0;

		std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
		std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

		select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
		select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


		cv::Mat unwrap_hor, unwrap_ver;

		float ver_period_num = 1;

		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			ver_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error." << std::endl;
			return false;
		}


		float hor_period_num = 1;
		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			hor_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error!" << std::endl;
			return false;
		}

		/*******************************************************************************************************************************************/

		float ver_period = ver_period_num;
		float hor_period = hor_period_num * dlp_height / dlp_width;


		unwrap_ver /= ver_period;
		unwrap_hor /= hor_period;
		/************************************************************************************************/

		//相机转换至投影仪

		std::vector<cv::Point2f> dlp_points;

		calib_function.fillThePhaseHole(unwrap_hor, true);
		calib_function.fillThePhaseHole(unwrap_ver, false);

		ret = calib_function.cameraPointsToDlp(select_board_points_list[g_i], unwrap_hor, unwrap_ver, 1, dlp_width, dlp_height, dlp_points);

		if (!ret)
		{
			std::cout << "Bad Patterns: " << select_folder_list[g_i] << std::endl;
			select_group_flag_base_phase.push_back(false);
		}
		else
		{
			select_group_flag_base_phase.push_back(true);
		}
		dlp_points_list.push_back(dlp_points);

		//std::string save_folder = "F:\\dexforce\\OpenCam3D\\DFX-152\\OpenCam3D\\x64\\Release\\capture_0624\\phase\\";


		//std::vector<std::string> str_list = vStringSplit(select_folder_list[g_i], "\\");
		//std::string folderPath = save_folder + str_list.back();
		//std::string mkdir_cmd = std::string("mkdir ") + folderPath;
		//system(mkdir_cmd.c_str());

		//unwrap_hor.convertTo(unwrap_hor, CV_32F);
		//unwrap_ver.convertTo(unwrap_ver, CV_32F);
		//std::string path = folderPath + "\\03.bmp";
		//cv::imwrite(path, select_board_images_list_base_camera[g_i]);

		//path = folderPath + "\\01.tiff";
		//cv::imwrite(path, unwrap_ver);

		//path = folderPath + "\\02.tiff";
		//cv::imwrite(path, unwrap_hor);
		//std::cout << "save " << folderPath << std::endl;
	}
	/**********************************************************************************/
	//选择出对应图

	if (select_group_flag_base_phase.size() != select_board_points_list.size())
	{
		std::cout << "group num error!" << std::endl;
		return -1;
	}

	std::vector<std::vector<cv::Point2f>> select_board_points_list_base_phase;
	std::vector<std::vector<cv::Point2f>> select_dlp_points_list_base_phase;

	for (int g_i = 0; g_i < select_group_flag_base_phase.size(); g_i++)
	{
		if (select_group_flag_base_phase[g_i])
		{
			select_board_points_list_base_phase.push_back(select_board_points_list[g_i]);
			select_dlp_points_list_base_phase.push_back(dlp_points_list[g_i]);
		}
	}


	std::cout << "Vaild Phase Group Number: " << select_dlp_points_list_base_phase.size() << std::endl;

	std::cout << "Start Calibrate Projector...... " << std::endl;
	//投影仪标定并筛选图像组
	std::map<int, bool> select_group_dlp;
	err = calib_function.calibrateProjector(select_dlp_points_list_base_phase, select_group_dlp);


	std::cout << "Calibrate Projector Error: " << err << std::endl;


	std::vector<std::vector<cv::Point2f>> select_dlp_points_list;
	std::vector<std::vector<cv::Point2f>> select_camera_points_list;

	for (int g_i = 0; g_i < select_group_dlp.size(); g_i++)
	{
		if (select_group_dlp[g_i])
		{
			select_dlp_points_list.push_back(select_dlp_points_list_base_phase[g_i]);
			select_camera_points_list.push_back(select_board_points_list_base_phase[g_i]);
		}
	}



	std::cout << "Projector Select Group Number: " << select_dlp_points_list.size() << std::endl;
	std::cout << "Start Calibrate Stereo...... " << std::endl;
	//对筛选出来的图像进行立体标定
	double stereo_err = calib_function.calibrateStereo(select_camera_points_list, select_dlp_points_list, calib_path);


	std::cout << "Reprojection Error: " << stereo_err << std::endl;
	std::cout << "Reprojection Error should be less than 0.1......" << std::endl;

	//cv::destroyAllWindows();


	return true;
}


bool correct_stereo_color(std::string patterns_path, std::string in_path, std::string out_path)
{
	std::string path = patterns_path;
	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message.rows, board_message.cols, board_message.width, board_message.height);

	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(path, files_list);

	std::vector<std::string> current_folder_list;
	getJustCurrentDir(path, current_folder_list);

	if (files_list.empty())
	{
		std::cout << "Read Images error!" << std::endl;
		return false;
	}

	std::cout << "Start Read Board Images...... " << std::endl;

	//文件夹路径
	std::vector<std::string> folder_list;

	//读取标定板图像
	std::vector<cv::Mat> board_images_list;

	std::vector<cv::Mat> blue_board_images_list;
	for (int b_i = 0; b_i < files_list.size(); b_i++)
	{
		std::vector<std::string> g_image_list = files_list[b_i];

		cv::Mat bayer = cv::imread(g_image_list.back(), 0);
		  
		cv::Mat board_img;
		convertBayer2Blue(bayer, board_img);

		board_images_list.push_back(board_img);
		folder_list.push_back(current_folder_list[b_i]);
	}

	std::cout << "Board Images Number: " << board_images_list.size() << std::endl;
	std::cout << "Start Find Board...... " << std::endl;

	//查找标定板初步筛选图片组
	std::vector<std::vector<std::string>> select_images_path_list_base_board;
	std::vector<cv::Mat> select_board_images_list;
	std::vector<std::vector<cv::Point2f>> board_points_list;

	for (int g_i = 0; g_i < files_list.size(); g_i++)
	{
		std::vector<std::string> g_image_list = files_list[g_i];
		cv::Mat img = board_images_list[g_i];

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(img, circle_points);

		std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
		cv::imwrite(img_path, img);
 

		if (found)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(img, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);

			std::vector<std::string> str_list = vStringSplit(folder_list[g_i], "/");

			std::string board_path = path + "/" + std::to_string(g_i) + "_draw.bmp";
			//std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
			cv::imwrite(board_path, color_img);
			//cv::imwrite(img_path, img);


			if (0 == calib_function.testOverExposure(img, circle_points))
			{
				std::cout << "over exposure: " << folder_list[g_i] << std::endl;
			}

			select_images_path_list_base_board.push_back(g_image_list);
			board_points_list.push_back(circle_points);
			select_board_images_list.push_back(img);

		}
	}

	if (select_images_path_list_base_board.size() < 2)
	{
		std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
		return false;
	}

	std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
	std::cout << "Start Calibrate Camera...... " << std::endl;

	//标定相机并根据重投影误差筛选图片组
	std::map<int, bool> select_group;
	double err = calib_function.calibrateCamera(board_points_list, select_group);


	std::cout << "Calibrate Camera Error: " << err << std::endl;
	std::cout << "Camera Select Group Number: " << select_group.size() << std::endl;

	std::vector<std::vector<std::string>> select_images_path_list_base_camera;
	std::vector<std::vector<cv::Point2f>> select_board_points_list;
	std::vector<std::vector<cv::Point2f>> dlp_points_list;
	std::vector<std::vector<cv::Point3f>> world_points_list;
	std::vector<std::string> select_folder_list;
	std::vector<cv::Mat> select_board_images_list_base_camera;

	for (int g_i = 0; g_i < select_group.size(); g_i++)
	{
		if (select_group[g_i])
		{
			select_images_path_list_base_camera.push_back(select_images_path_list_base_board[g_i]);
			select_board_points_list.push_back(board_points_list[g_i]);
			select_folder_list.push_back(folder_list[g_i]);
			select_board_images_list_base_camera.push_back(select_board_images_list[g_i]);
		}
	}

	/*****************************************************************************************************/

	std::cout << "Start Read Pattern Image...... " << std::endl;

	//读取筛选出来的组图像
	std::vector<std::vector<cv::Mat>> group_image_list;
	for (int g_i = 0; g_i < select_images_path_list_base_camera.size(); g_i++)
	{
		std::vector<std::string> path_list = select_images_path_list_base_camera[g_i];

		std::vector<cv::Mat> img_list;

		for (int n_i = 0; n_i < path_list.size() - 1; n_i++)
		{
			//cv::Mat img = cv::imread(path_list[n_i], 0);

			cv::Mat bayer = cv::imread(path_list[n_i], 0); 
			cv::Mat img;
			convertBayer2Blue(bayer, img);

			img_list.push_back(img);
		}

		group_image_list.push_back(img_list);
	}


	std::cout << "Read Pattern Finished." << std::endl;
	std::cout << "Start Compute Phase ......" << std::endl;
	/******************************************************************************************************************/
	//依据相位值选择
	std::vector<bool> select_group_flag_base_phase;
	DF_Encode encode_machine_;


	//相位计算、相位展开、坐标转换
	for (int g_i = 0; g_i < group_image_list.size(); g_i++)
	{
		std::vector<cv::Mat> wrap_img_list;
		std::vector<cv::Mat> img_list = group_image_list[g_i];
		cv::Mat board_img = board_images_list[g_i];

		/*****************************************************************************************************************************************/
		//4+4+4+6
		int vertical_four_num = 12;
		int vertical_six_num = 6;
		int horizontal_four_num = 12;
		int horizontal_six_num = 6;

		std::vector<cv::Mat>::const_iterator list_ptr = img_list.begin();

		std::vector<cv::Mat> ver_patterns_img_4(list_ptr, list_ptr + vertical_four_num);
		std::vector<cv::Mat> ver_patterns_img_6(list_ptr + vertical_four_num, list_ptr + vertical_four_num + vertical_six_num);


		list_ptr = img_list.begin() + +img_list.size() / 2;

		std::vector<cv::Mat> hor_patterns_img_4(list_ptr, list_ptr + horizontal_four_num);
		std::vector<cv::Mat> hor_patterns_img_6(list_ptr + horizontal_four_num, list_ptr + horizontal_four_num + vertical_six_num);


		std::vector<cv::Mat> ver_wrap_img_4;
		std::vector<cv::Mat> hor_wrap_img_4;
		cv::Mat ver_confidence_map_4;
		cv::Mat hor_confidence_map_4;

		std::vector<cv::Mat> ver_wrap_img_6;
		std::vector<cv::Mat> hor_wrap_img_6;
		cv::Mat ver_confidence_map_6;
		cv::Mat hor_confidence_map_6;

		bool ret = true;

		cv::Mat test_mask_ = cv::Mat();

		DF_Encode encode_machine_;


		ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
		ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


		ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
		ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

		std::vector<double> variable_wrap_rate;
		variable_wrap_rate.push_back(8);
		variable_wrap_rate.push_back(4);
		variable_wrap_rate.push_back(4);


		cv::Mat unwrap_mask = test_mask_.clone();

		int discard_num = 0;

		std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
		std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

		select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
		select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


		cv::Mat unwrap_hor, unwrap_ver;

		float ver_period_num = 1;

		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			ver_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error." << std::endl;
			return false;
		}


		float hor_period_num = 1;
		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			hor_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error!" << std::endl;
			return false;
		}

		/*******************************************************************************************************************************************/

		float ver_period = ver_period_num;
		float hor_period = hor_period_num * dlp_height / dlp_width;


		unwrap_ver /= ver_period;
		unwrap_hor /= hor_period;
		/************************************************************************************************/

		//相机转换至投影仪

		std::vector<cv::Point2f> dlp_points;

		calib_function.fillThePhaseHole(unwrap_hor, true);
		calib_function.fillThePhaseHole(unwrap_ver, false);

		ret = calib_function.cameraPointsToDlp(select_board_points_list[g_i], unwrap_hor, unwrap_ver, 1, dlp_width, dlp_height, dlp_points);

		if (!ret)
		{
			std::cout << "Bad Patterns: " << select_folder_list[g_i] << std::endl;
			select_group_flag_base_phase.push_back(false);
		}
		else
		{
			select_group_flag_base_phase.push_back(true);
		}
		dlp_points_list.push_back(dlp_points);
	}
	/**********************************************************************************/
	//选择出对应图

	if (select_group_flag_base_phase.size() != select_board_points_list.size())
	{
		std::cout << "group num error!" << std::endl;
		return -1;
	}

	std::vector<std::vector<cv::Point2f>> select_board_points_list_base_phase;
	std::vector<std::vector<cv::Point2f>> select_dlp_points_list_base_phase;

	for (int g_i = 0; g_i < select_group_flag_base_phase.size(); g_i++)
	{
		if (select_group_flag_base_phase[g_i])
		{
			select_board_points_list_base_phase.push_back(select_board_points_list[g_i]);
			select_dlp_points_list_base_phase.push_back(dlp_points_list[g_i]);
		}
	}


	std::cout << "Vaild Phase Group Number: " << select_dlp_points_list_base_phase.size() << std::endl;

	std::cout << "Start Calibrate Projector...... " << std::endl;
	//投影仪标定并筛选图像组
	std::map<int, bool> select_group_dlp;
	err = calib_function.calibrateProjector(select_dlp_points_list_base_phase, select_group_dlp);


	std::cout << "Calibrate Projector Error: " << err << std::endl;


	std::vector<std::vector<cv::Point2f>> select_dlp_points_list;
	std::vector<std::vector<cv::Point2f>> select_camera_points_list;

	for (int g_i = 0; g_i < select_group_dlp.size(); g_i++)
	{
		if (select_group_dlp[g_i])
		{
			select_dlp_points_list.push_back(select_dlp_points_list_base_phase[g_i]);
			select_camera_points_list.push_back(select_board_points_list_base_phase[g_i]);
		}
	}



	std::cout << "Projector Select Group Number: " << select_dlp_points_list.size() << std::endl;
	std::cout << "Start Calibrate Stereo...... " << std::endl;
	//对筛选出来的图像进行立体标定
	double stereo_err = calib_function.correctExtrinsics(select_camera_points_list, select_dlp_points_list, param_in_path, param_out_path);


	std::cout << "Reprojection Error: " << stereo_err << std::endl;
	std::cout << "Reprojection Error should be less than 0.1......" << std::endl;

	//cv::destroyAllWindows();


	return true;
}

bool correct_stereo(std::string patterns_path, std::string in_path, std::string out_path)
{
	std::string path = patterns_path;
	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message.rows, board_message.cols, board_message.width, board_message.height);

	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(path, files_list);

	std::vector<std::string> current_folder_list;
	getJustCurrentDir(path, current_folder_list);

	if (files_list.empty())
	{
		std::cout << "Read Images error!" << std::endl;
		return false;
	}

	std::cout << "Start Read Board Images...... " << std::endl;

	//文件夹路径
	std::vector<std::string> folder_list;

	//读取标定板图像
	std::vector<cv::Mat> board_images_list;
	for (int b_i = 0; b_i < files_list.size(); b_i++)
	{
		std::vector<std::string> g_image_list = files_list[b_i];

		cv::Mat board_img = cv::imread(g_image_list.back(), 0);
		board_images_list.push_back(board_img);
		folder_list.push_back(current_folder_list[b_i]);
	}

	std::cout << "Board Images Number: " << board_images_list.size() << std::endl;
	std::cout << "Start Find Board...... " << std::endl;

	//查找标定板初步筛选图片组
	std::vector<std::vector<std::string>> select_images_path_list_base_board;
	std::vector<cv::Mat> select_board_images_list;
	std::vector<std::vector<cv::Point2f>> board_points_list;

	for (int g_i = 0; g_i < files_list.size(); g_i++)
	{
		std::vector<std::string> g_image_list = files_list[g_i];
		cv::Mat img = board_images_list[g_i];

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(img, circle_points);

		std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
		cv::imwrite(img_path, img);

		if (found)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(img, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);

			std::vector<std::string> str_list = vStringSplit(folder_list[g_i], "/");

			std::string board_path = path + "/" + std::to_string(g_i) + "_draw.bmp";
			//std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
			cv::imwrite(board_path, color_img);
			//cv::imwrite(img_path, img);


			if (0 == calib_function.testOverExposure(img, circle_points))
			{
				std::cout << "over exposure: " << folder_list[g_i] << std::endl;
			}

			select_images_path_list_base_board.push_back(g_image_list);
			board_points_list.push_back(circle_points);
			select_board_images_list.push_back(img);

		}  
	}

	if (select_images_path_list_base_board.size() < 2)
	{
		std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
		return false;
	}

	std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
	std::cout << "Start Calibrate Camera...... " << std::endl;

	//标定相机并根据重投影误差筛选图片组
	std::map<int, bool> select_group;
	double err = calib_function.calibrateCamera(board_points_list, select_group);


	std::cout << "Calibrate Camera Error: " << err << std::endl;
	std::cout << "Camera Select Group Number: " << select_group.size() << std::endl;

	std::vector<std::vector<std::string>> select_images_path_list_base_camera;
	std::vector<std::vector<cv::Point2f>> select_board_points_list;
	std::vector<std::vector<cv::Point2f>> dlp_points_list;
	std::vector<std::vector<cv::Point3f>> world_points_list;
	std::vector<std::string> select_folder_list;
	std::vector<cv::Mat> select_board_images_list_base_camera;

	for (int g_i = 0; g_i < select_group.size(); g_i++)
	{
		if (select_group[g_i])
		{
			select_images_path_list_base_camera.push_back(select_images_path_list_base_board[g_i]);
			select_board_points_list.push_back(board_points_list[g_i]);
			select_folder_list.push_back(folder_list[g_i]);
			select_board_images_list_base_camera.push_back(select_board_images_list[g_i]);
		}
	}

	/*****************************************************************************************************/

	std::cout << "Start Read Pattern Image...... " << std::endl;

	//读取筛选出来的组图像
	std::vector<std::vector<cv::Mat>> group_image_list;
	for (int g_i = 0; g_i < select_images_path_list_base_camera.size(); g_i++)
	{
		std::vector<std::string> path_list = select_images_path_list_base_camera[g_i];

		std::vector<cv::Mat> img_list;

		for (int n_i = 0; n_i < path_list.size() - 1; n_i++)
		{
			cv::Mat img = cv::imread(path_list[n_i], 0);
			img_list.push_back(img);
		}

		group_image_list.push_back(img_list);
	}


	std::cout << "Read Pattern Finished." << std::endl;
	std::cout << "Start Compute Phase ......" << std::endl;
	/******************************************************************************************************************/
	//依据相位值选择
	std::vector<bool> select_group_flag_base_phase;
	DF_Encode encode_machine_;


	//相位计算、相位展开、坐标转换
	for (int g_i = 0; g_i < group_image_list.size(); g_i++)
	{
		std::vector<cv::Mat> wrap_img_list;
		std::vector<cv::Mat> img_list = group_image_list[g_i];
		cv::Mat board_img = board_images_list[g_i];

		/*****************************************************************************************************************************************/
		//4+4+4+6
		int vertical_four_num = 12;
		int vertical_six_num = 6;
		int horizontal_four_num = 12;
		int horizontal_six_num = 6;

		std::vector<cv::Mat>::const_iterator list_ptr = img_list.begin();

		std::vector<cv::Mat> ver_patterns_img_4(list_ptr, list_ptr + vertical_four_num);
		std::vector<cv::Mat> ver_patterns_img_6(list_ptr + vertical_four_num, list_ptr + vertical_four_num + vertical_six_num);


		list_ptr = img_list.begin() + +img_list.size() / 2;

		std::vector<cv::Mat> hor_patterns_img_4(list_ptr, list_ptr + horizontal_four_num);
		std::vector<cv::Mat> hor_patterns_img_6(list_ptr + horizontal_four_num, list_ptr + horizontal_four_num + vertical_six_num);


		std::vector<cv::Mat> ver_wrap_img_4;
		std::vector<cv::Mat> hor_wrap_img_4;
		cv::Mat ver_confidence_map_4;
		cv::Mat hor_confidence_map_4;

		std::vector<cv::Mat> ver_wrap_img_6;
		std::vector<cv::Mat> hor_wrap_img_6;
		cv::Mat ver_confidence_map_6;
		cv::Mat hor_confidence_map_6;

		bool ret = true;

		cv::Mat test_mask_ = cv::Mat();

		DF_Encode encode_machine_;


		ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
		ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


		ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
		ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

		std::vector<double> variable_wrap_rate;
		variable_wrap_rate.push_back(8);
		variable_wrap_rate.push_back(4);
		variable_wrap_rate.push_back(4);


		cv::Mat unwrap_mask = test_mask_.clone();

		int discard_num = 0;

		std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
		std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

		select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
		select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


		cv::Mat unwrap_hor, unwrap_ver;

		float ver_period_num = 1;

		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			ver_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error." << std::endl;
			return false;
		}


		float hor_period_num = 1;
		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			hor_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error!" << std::endl;
			return false;
		}

		/*******************************************************************************************************************************************/

		float ver_period = ver_period_num;
		float hor_period = hor_period_num * dlp_height / dlp_width;


		unwrap_ver /= ver_period;
		unwrap_hor /= hor_period;
		/************************************************************************************************/

		//相机转换至投影仪

		std::vector<cv::Point2f> dlp_points;

		calib_function.fillThePhaseHole(unwrap_hor, true);
		calib_function.fillThePhaseHole(unwrap_ver, false);

		ret = calib_function.cameraPointsToDlp(select_board_points_list[g_i], unwrap_hor, unwrap_ver, 1, dlp_width, dlp_height, dlp_points);

		if (!ret)
		{
			std::cout << "Bad Patterns: " << select_folder_list[g_i] << std::endl;
			select_group_flag_base_phase.push_back(false);
		}
		else
		{
			select_group_flag_base_phase.push_back(true);
		}
		dlp_points_list.push_back(dlp_points);
	}
	/**********************************************************************************/
	//选择出对应图

	if (select_group_flag_base_phase.size() != select_board_points_list.size())
	{
		std::cout << "group num error!" << std::endl;
		return -1;
	}

	std::vector<std::vector<cv::Point2f>> select_board_points_list_base_phase;
	std::vector<std::vector<cv::Point2f>> select_dlp_points_list_base_phase;

	for (int g_i = 0; g_i < select_group_flag_base_phase.size(); g_i++)
	{
		if (select_group_flag_base_phase[g_i])
		{
			select_board_points_list_base_phase.push_back(select_board_points_list[g_i]);
			select_dlp_points_list_base_phase.push_back(dlp_points_list[g_i]);
		}
	}


	std::cout << "Vaild Phase Group Number: " << select_dlp_points_list_base_phase.size() << std::endl;

	std::cout << "Start Calibrate Projector...... " << std::endl;
	//投影仪标定并筛选图像组
	std::map<int, bool> select_group_dlp;
	err = calib_function.calibrateProjector(select_dlp_points_list_base_phase, select_group_dlp);


	std::cout << "Calibrate Projector Error: " << err << std::endl;


	std::vector<std::vector<cv::Point2f>> select_dlp_points_list;
	std::vector<std::vector<cv::Point2f>> select_camera_points_list;

	for (int g_i = 0; g_i < select_group_dlp.size(); g_i++)
	{
		if (select_group_dlp[g_i])
		{
			select_dlp_points_list.push_back(select_dlp_points_list_base_phase[g_i]);
			select_camera_points_list.push_back(select_board_points_list_base_phase[g_i]);
		}
	}



	std::cout << "Projector Select Group Number: " << select_dlp_points_list.size() << std::endl;
	std::cout << "Start Calibrate Stereo...... " << std::endl;
	//对筛选出来的图像进行立体标定
	double stereo_err = calib_function.correctExtrinsics(select_camera_points_list, select_dlp_points_list, param_in_path,param_out_path);


	std::cout << "Reprojection Error: " << stereo_err << std::endl;
	std::cout << "Reprojection Error should be less than 0.1......" << std::endl;

	//cv::destroyAllWindows();


	return true;
}

bool calibrate_stereo(std::string patterns_path, std::string calib_path)
{
	std::string path = patterns_path;
	Calibrate_Function calib_function;
	calib_function.setBoardMessage(board_message.rows, board_message.cols, board_message.width, board_message.height);

	//读取多组标定条纹图案图像
	std::vector<std::vector<std::string>> files_list;
	getFilesList(path, files_list);

	std::vector<std::string> current_folder_list;
	getJustCurrentDir(path, current_folder_list);

	if (files_list.empty())
	{
		std::cout << "Read Images error!" << std::endl;
		return false;
	}

	std::cout << "Start Read Board Images...... " << std::endl;

	//文件夹路径
	std::vector<std::string> folder_list;

	//读取标定板图像
	std::vector<cv::Mat> board_images_list;
	for (int b_i = 0; b_i < files_list.size(); b_i++)
	{
		std::vector<std::string> g_image_list = files_list[b_i];

		cv::Mat board_img = cv::imread(g_image_list.back(), 0);
		board_images_list.push_back(board_img);
		folder_list.push_back(current_folder_list[b_i]);
	}

	std::cout << "Board Images Number: " << board_images_list.size() << std::endl;
	std::cout << "Start Find Board...... " << std::endl;

	//查找标定板初步筛选图片组
	std::vector<std::vector<std::string>> select_images_path_list_base_board;
	std::vector<cv::Mat> select_board_images_list;
	std::vector<std::vector<cv::Point2f>> board_points_list;

	for (int g_i = 0; g_i < files_list.size(); g_i++)
	{
		std::vector<std::string> g_image_list = files_list[g_i];
		cv::Mat img = board_images_list[g_i];

		std::vector<cv::Point2f> circle_points;
		bool found = calib_function.findCircleBoardFeature(img, circle_points);

		std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
		cv::imwrite(img_path, img);

		if (found)
		{
			cv::Mat color_img;
			cv::Size board_size = calib_function.getBoardSize();
			cv::cvtColor(img, color_img, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(color_img, board_size, circle_points, found);

			std::vector<std::string> str_list = vStringSplit(folder_list[g_i], "/");

			std::string board_path = path + "/" + std::to_string(g_i) + "_draw.bmp";
			//std::string img_path = path + "/" + std::to_string(g_i) + "_board.bmp";
			cv::imwrite(board_path, color_img);
			//cv::imwrite(img_path, img);


			if (0 == calib_function.testOverExposure(img, circle_points))
			{
				std::cout << "over exposure: " << folder_list[g_i] << std::endl;
			}

			select_images_path_list_base_board.push_back(g_image_list);
			board_points_list.push_back(circle_points);
			select_board_images_list.push_back(img);

		}


	}

	if (select_images_path_list_base_board.size() < 6)
	{
		std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
		return false;
	}

	std::cout << "Valid Board Number: " << select_images_path_list_base_board.size() << std::endl;
	std::cout << "Start Calibrate Camera...... " << std::endl;

	//标定相机并根据重投影误差筛选图片组
	std::map<int, bool> select_group;
	double err = calib_function.calibrateCamera(board_points_list, select_group);


	std::cout << "Calibrate Camera Error: " << err << std::endl;
	std::cout << "Camera Select Group Number: " << select_group.size() << std::endl;

	std::vector<std::vector<std::string>> select_images_path_list_base_camera;
	std::vector<std::vector<cv::Point2f>> select_board_points_list;
	std::vector<std::vector<cv::Point2f>> dlp_points_list;
	std::vector<std::vector<cv::Point3f>> world_points_list;
	std::vector<std::string> select_folder_list;
	std::vector<cv::Mat> select_board_images_list_base_camera;

	for (int g_i = 0; g_i < select_group.size(); g_i++)
	{
		if (select_group[g_i])
		{
			select_images_path_list_base_camera.push_back(select_images_path_list_base_board[g_i]);
			select_board_points_list.push_back(board_points_list[g_i]);
			select_folder_list.push_back(folder_list[g_i]);
			select_board_images_list_base_camera.push_back(select_board_images_list[g_i]);
		}
	}

	/*****************************************************************************************************/

	std::cout << "Start Read Pattern Image...... " << std::endl;

	//读取筛选出来的组图像
	std::vector<std::vector<cv::Mat>> group_image_list;
	for (int g_i = 0; g_i < select_images_path_list_base_camera.size(); g_i++)
	{
		std::vector<std::string> path_list = select_images_path_list_base_camera[g_i];

		std::vector<cv::Mat> img_list;

		for (int n_i = 0; n_i < path_list.size() - 1; n_i++)
		{
			cv::Mat img = cv::imread(path_list[n_i], 0);
			img_list.push_back(img);
		}

		group_image_list.push_back(img_list);
	}


	std::cout << "Read Pattern Finished." << std::endl;
	std::cout << "Start Compute Phase ......" << std::endl;
	/******************************************************************************************************************/
	//依据相位值选择
	std::vector<bool> select_group_flag_base_phase;
	DF_Encode encode_machine_;


	//相位计算、相位展开、坐标转换
	for (int g_i = 0; g_i < group_image_list.size(); g_i++)
	{
		std::vector<cv::Mat> wrap_img_list;
		std::vector<cv::Mat> img_list = group_image_list[g_i];
		cv::Mat board_img = board_images_list[g_i];

		/*****************************************************************************************************************************************/
		//4+4+4+6
		int vertical_four_num = 12;
		int vertical_six_num = 6;
		int horizontal_four_num = 12;
		int horizontal_six_num = 6;

		std::vector<cv::Mat>::const_iterator list_ptr = img_list.begin();

		std::vector<cv::Mat> ver_patterns_img_4(list_ptr, list_ptr + vertical_four_num);
		std::vector<cv::Mat> ver_patterns_img_6(list_ptr + vertical_four_num, list_ptr + vertical_four_num + vertical_six_num);


		list_ptr = img_list.begin() + +img_list.size() / 2;

		std::vector<cv::Mat> hor_patterns_img_4(list_ptr, list_ptr + horizontal_four_num);
		std::vector<cv::Mat> hor_patterns_img_6(list_ptr + horizontal_four_num, list_ptr + horizontal_four_num + vertical_six_num);


		std::vector<cv::Mat> ver_wrap_img_4;
		std::vector<cv::Mat> hor_wrap_img_4;
		cv::Mat ver_confidence_map_4;
		cv::Mat hor_confidence_map_4;

		std::vector<cv::Mat> ver_wrap_img_6;
		std::vector<cv::Mat> hor_wrap_img_6;
		cv::Mat ver_confidence_map_6;
		cv::Mat hor_confidence_map_6;

		bool ret = true;

		cv::Mat test_mask_ = cv::Mat();

		DF_Encode encode_machine_;


		ret = encode_machine_.computePhaseBaseFourStep(ver_patterns_img_4, ver_wrap_img_4, test_mask_, ver_confidence_map_4);
		ret = encode_machine_.computePhaseBaseFourStep(hor_patterns_img_4, hor_wrap_img_4, test_mask_, hor_confidence_map_4);


		ret = encode_machine_.computePhaseBaseSixStep(ver_patterns_img_6, ver_wrap_img_6, test_mask_, ver_confidence_map_6);
		ret = encode_machine_.computePhaseBaseSixStep(hor_patterns_img_6, hor_wrap_img_6, test_mask_, hor_confidence_map_6);

		std::vector<double> variable_wrap_rate;
		variable_wrap_rate.push_back(8);
		variable_wrap_rate.push_back(4);
		variable_wrap_rate.push_back(4);


		cv::Mat unwrap_mask = test_mask_.clone();

		int discard_num = 0;

		std::vector<cv::Mat> select_ver_wrap_img = ver_wrap_img_4;
		std::vector<cv::Mat> select_hor_wrap_img = hor_wrap_img_4;

		select_ver_wrap_img.push_back(ver_wrap_img_6[0]);
		select_hor_wrap_img.push_back(hor_wrap_img_6[0]);


		cv::Mat unwrap_hor, unwrap_ver;

		float ver_period_num = 1;

		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			ver_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_ver_wrap_img, variable_wrap_rate, unwrap_ver, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error." << std::endl;
			return false;
		}


		float hor_period_num = 1;
		for (int r_i = 0; r_i < variable_wrap_rate.size(); r_i++)
		{
			hor_period_num *= variable_wrap_rate[r_i];
		}

		ret = encode_machine_.unwrapVariableWavelengthPatterns(select_hor_wrap_img, variable_wrap_rate, unwrap_hor, unwrap_mask);
		if (!ret)
		{
			std::cout << "unwrap Error!" << std::endl;
			return false;
		}

		/*******************************************************************************************************************************************/

		float ver_period = ver_period_num;
		float hor_period = hor_period_num * dlp_height / dlp_width;


		unwrap_ver /= ver_period;
		unwrap_hor /= hor_period;
		/************************************************************************************************/

		//相机转换至投影仪

		std::vector<cv::Point2f> dlp_points;

		calib_function.fillThePhaseHole(unwrap_hor, true);
		calib_function.fillThePhaseHole(unwrap_ver, false);

		ret = calib_function.cameraPointsToDlp(select_board_points_list[g_i], unwrap_hor, unwrap_ver, 1, dlp_width, dlp_height, dlp_points);

		if (!ret)
		{
			std::cout << "Bad Patterns: " << select_folder_list[g_i] << std::endl;
			select_group_flag_base_phase.push_back(false);
		}
		else
		{
			select_group_flag_base_phase.push_back(true);
		}
		dlp_points_list.push_back(dlp_points);

		//std::string save_folder = "F:\\dexforce\\OpenCam3D\\DFX-152\\OpenCam3D\\x64\\Release\\capture_0624\\phase\\";


		//std::vector<std::string> str_list = vStringSplit(select_folder_list[g_i], "\\");
		//std::string folderPath = save_folder + str_list.back();
		//std::string mkdir_cmd = std::string("mkdir ") + folderPath;
		//system(mkdir_cmd.c_str());

		//unwrap_hor.convertTo(unwrap_hor, CV_32F);
		//unwrap_ver.convertTo(unwrap_ver, CV_32F);
		//std::string path = folderPath + "\\03.bmp";
		//cv::imwrite(path, select_board_images_list_base_camera[g_i]);

		//path = folderPath + "\\01.tiff";
		//cv::imwrite(path, unwrap_ver);

		//path = folderPath + "\\02.tiff";
		//cv::imwrite(path, unwrap_hor);
		//std::cout << "save " << folderPath << std::endl;
	}
	/**********************************************************************************/
	//选择出对应图

	if (select_group_flag_base_phase.size() != select_board_points_list.size())
	{
		std::cout << "group num error!" << std::endl;
		return -1;
	}

	std::vector<std::vector<cv::Point2f>> select_board_points_list_base_phase;
	std::vector<std::vector<cv::Point2f>> select_dlp_points_list_base_phase;

	for (int g_i = 0; g_i < select_group_flag_base_phase.size(); g_i++)
	{
		if (select_group_flag_base_phase[g_i])
		{
			select_board_points_list_base_phase.push_back(select_board_points_list[g_i]);
			select_dlp_points_list_base_phase.push_back(dlp_points_list[g_i]);
		}
	}


	std::cout << "Vaild Phase Group Number: " << select_dlp_points_list_base_phase.size() << std::endl;

	std::cout << "Start Calibrate Projector...... " << std::endl;
	//投影仪标定并筛选图像组
	std::map<int, bool> select_group_dlp;
	err = calib_function.calibrateProjector(select_dlp_points_list_base_phase, select_group_dlp);


	std::cout << "Calibrate Projector Error: " << err << std::endl;


	std::vector<std::vector<cv::Point2f>> select_dlp_points_list;
	std::vector<std::vector<cv::Point2f>> select_camera_points_list;

	for (int g_i = 0; g_i < select_group_dlp.size(); g_i++)
	{
		if (select_group_dlp[g_i])
		{
			select_dlp_points_list.push_back(select_dlp_points_list_base_phase[g_i]);
			select_camera_points_list.push_back(select_board_points_list_base_phase[g_i]);
		}
	}



	std::cout << "Projector Select Group Number: " << select_dlp_points_list.size() << std::endl;
	std::cout << "Start Calibrate Stereo...... " << std::endl;
	//对筛选出来的图像进行立体标定
	double stereo_err = calib_function.calibrateStereo(select_camera_points_list, select_dlp_points_list, calib_path);


	std::cout << "Reprojection Error: " << stereo_err << std::endl;
	std::cout << "Reprojection Error should be less than 0.1......" << std::endl;

	//cv::destroyAllWindows();


	return true;
}

