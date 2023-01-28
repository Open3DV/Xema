#include "xema_camera.h"
#include "../firmware/easylogging++.h"
#include "../sdk/camera_status.h"
#include "../firmware/protocol.h"
#include "../test/triangulation.h"
#include <chrono>
#include <ctime>
#include <time.h>
#include <stddef.h> 
#include <assert.h>
#include <iostream>
#include <thread>   
#include <stddef.h> 

#ifdef _WIN32

#elif __linux
#include <dirent.h>
#include <unistd.h>
#define INVALID_SOCKET (~0)
#define SOCKET_ERROR -1
#endif

INITIALIZE_EASYLOGGINGPP
using namespace std;
using namespace std::chrono;

namespace XEMA {

	XemaCamera::XemaCamera()
	{
		g_sock = INVALID_SOCKET;
		g_sock_heartbeat = INVALID_SOCKET;
	}

	XemaCamera::~XemaCamera()
	{

	}

	void rolloutHandler(const char* filename, std::size_t size)
	{
#ifdef _WIN32 
		/// ������־
		system("mkdir xemaLog");
		system("DIR .\\xemaLog\\ .log / B > LIST.TXT");
		ifstream name_in("LIST.txt", ios_base::in);//�ļ���

		int num = 0;
		std::vector<std::string> name_list;
		char buf[1024] = { 0 };
		while (name_in.getline(buf, sizeof(buf)))
		{
			//std::cout << "name: " << buf << std::endl;
			num++;
			name_list.push_back(std::string(buf));
		}

		if (num < 5)
		{
			num++;
		}
		else
		{
			num = 5;
			name_list.pop_back();
		}


		for (int i = num; i > 0 && !name_list.empty(); i--)
		{
			std::stringstream ss;
			std::string path = ".\\xemaLog\\" + name_list.back();
			name_list.pop_back();
			ss << "move " << path << " xemaLog\\log_" << i - 1 << ".log";
			std::cout << ss.str() << std::endl;
			system(ss.str().c_str());
		}

		std::stringstream ss;
		ss << "move " << filename << " xemaLog\\log_0" << ".log";
		system(ss.str().c_str());
#elif __linux 

		/// ������־
		if (access("xemaLog", F_OK) != 0)
		{
			system("mkdir xemaLog");
		}

		std::vector<std::string> name_list;
		std::string suffix = "log";
		DIR* dir;
		struct dirent* ent;
		if ((dir = opendir("xemaLog")) != NULL)
		{
			while ((ent = readdir(dir)) != NULL)
			{
				/* print all the files and directories within directory */
				// printf("%s\n", ent->d_name);

				std::string name = ent->d_name;

				if (name.size() < 3)
				{
					continue;
				}

				std::string curSuffix = name.substr(name.size() - 3);

				if (suffix == curSuffix)
				{
					name_list.push_back(name);
				}
			}
			closedir(dir);
		}

		sort(name_list.begin(), name_list.end());

		int num = name_list.size();
		if (num < 5)
		{
			num++;
		}
		else
		{
			num = 5;
			name_list.pop_back();
		}


		for (int i = num; i > 0 && !name_list.empty(); i--)
		{
			std::stringstream ss;
			std::string path = "./xemaLog/" + name_list.back();
			name_list.pop_back();
			ss << "mv " << path << " xemaLog/log_" << i - 1 << ".log";
			std::cout << ss.str() << std::endl;
			system(ss.str().c_str());
		}

		std::stringstream ss;
		ss << "mv " << filename << " xemaLog/log_0" << ".log";
		system(ss.str().c_str());

#endif 

	}


	//�������
	int on_dropped(void* param)
	{
		LOG(INFO) << "Network dropped!" << std::endl;
		return 0;
	}

	int XemaCamera::registerOnDropped(int (*p_function)(void*))
	{
		p_OnDropped_ = p_function;
		return 0;
	}


	int XemaCamera::HeartBeat()
	{
		//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		//{
		//	LOG(INFO) << "--";
		//}

		LOG(TRACE) << "heart beat: ";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock_heartbeat);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock_heartbeat);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_HEARTBEAT, g_sock_heartbeat);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to send disconnection cmd";
			close_socket(g_sock_heartbeat);
			return DF_FAILED;
		}
		ret = send_buffer((char*)&token, sizeof(token), g_sock_heartbeat);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to send token";
			close_socket(g_sock_heartbeat);
			return DF_FAILED;
		}

		int command;
		ret = recv_command(&command, g_sock_heartbeat);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock_heartbeat);
			return DF_FAILED;
		}
		if (command == DF_CMD_OK)
		{
			ret = DF_SUCCESS;
		}
		else if (command == DF_CMD_REJECT)
		{
			ret = DF_BUSY;
		}
		else
		{
			LOG(ERROR) << "Failed recv heart beat command";
		}
		close_socket(g_sock_heartbeat);
		return ret;
	}


	int XemaCamera::HeartBeat_loop()
	{
		heartbeat_error_count_ = 0;
		while (connected)
		{
			int ret = HeartBeat();
			if (ret == DF_FAILED)
			{
				heartbeat_error_count_++;
				LOG(ERROR) << "heartbeat error count: " << heartbeat_error_count_;

				if (heartbeat_error_count_ > 2)
				{
					LOG(ERROR) << "close connect";
					connected = false;
					//close_socket(g_sock); 
					p_OnDropped_(0);
				}

			}
			else if (DF_SUCCESS == ret)
			{
				heartbeat_error_count_ = 0;
			}


			for (int i = 0; i < 100; i++)
			{
				if (!connected)
				{
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		}
		return 0;
	}

	 

	int XemaCamera::connectNet(const char* ip)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfConnectNet: ";
		/*******************************************************************************************************************/
		//�ر�log���
		//el::Configurations conf;
		//conf.setToDefault();
		//conf.setGlobally(el::ConfigurationType::Format, "[%datetime{%H:%m:%s} | %level] %msg");
		//conf.setGlobally(el::ConfigurationType::Filename, "log\\log_%datetime{%Y%M%d}.log");
		//conf.setGlobally(el::ConfigurationType::Enabled, "true");
		//conf.setGlobally(el::ConfigurationType::ToFile, "true");
		//el::Loggers::reconfigureAllLoggers(conf);
		//el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");


		//DfRegisterOnDropped(on_dropped);
		/*******************************************************************************************************************/


		camera_ip_ = ip;
		LOG(INFO) << "start connection: " << ip;
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		LOG(INFO) << "sending connection cmd";
		ret = send_command(DF_CMD_CONNECT, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(INFO) << "Failed to send connection cmd";
			close_socket(g_sock);
			return DF_FAILED;
		}
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}
		if (ret == DF_SUCCESS)
		{
			if (command == DF_CMD_OK)
			{
				LOG(INFO) << "Recieved connection ok";
				ret = recv_buffer((char*)&token, sizeof(token), g_sock);
				if (ret == DF_SUCCESS)
				{
					connected = true;
					LOG(INFO) << "token: " << token;
					close_socket(g_sock);
					if (heartbeat_thread.joinable())
					{
						heartbeat_thread.join();
					}
					heartbeat_thread = std::thread(&XemaCamera::HeartBeat_loop,this);
					return DF_SUCCESS;
				}
			}
			else if (command == DF_CMD_REJECT)
			{
				LOG(INFO) << "connection rejected";
				close_socket(g_sock);
				return DF_BUSY;
			}
		}
		else
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		return DF_FAILED;
	}


	int XemaCamera::disconnectNet()
	{

		LOG(INFO) << "token " << token << " try to disconnection";
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";

		}


		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(INFO) << "Failed to setup_socket";
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_DISCONNECT, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(INFO) << "Failed to send disconnection cmd";
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		if (ret == DF_FAILED)
		{
			LOG(INFO) << "Failed to send token";
			close_socket(g_sock);
			return DF_FAILED;
		}
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}
		connected = false;
		token = 0;

		if (heartbeat_thread.joinable())
		{
			heartbeat_thread.join();
		}

		LOG(INFO) << "Camera disconnected";
		return close_socket(g_sock);
	}


	bool XemaCamera::transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation)
	{


		int point_num = camera_height_ * camera_width_;

		int nr = camera_height_;
		int nc = camera_width_;

#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{

			for (int c = 0; c < nc; c++)
			{

				int offset = r * camera_width_ + c;

				float x = org_point_cloud_map[3 * offset + 0];
				float y = org_point_cloud_map[3 * offset + 1];
				float z = org_point_cloud_map[3 * offset + 2];

				//if (z > 0)
				//{
				transform_point_cloud_map[3 * offset + 0] = rotate[0] * x + rotate[1] * y + rotate[2] * z + translation[0];
				transform_point_cloud_map[3 * offset + 1] = rotate[3] * x + rotate[4] * y + rotate[5] * z + translation[1];
				transform_point_cloud_map[3 * offset + 2] = rotate[6] * x + rotate[7] * y + rotate[8] * z + translation[2];

				//}
				//else
				//{
				   // point_cloud_map[3 * offset + 0] = 0;
				   // point_cloud_map[3 * offset + 1] = 0;
				   // point_cloud_map[3 * offset + 2] = 0;
				//}


			}

		}


		return true;
	}

	int XemaCamera::depthTransformPointcloud(float* depth_map, float* point_cloud_map)
	{

		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}

		float camera_fx = calibration_param_.camera_intrinsic[0];
		float camera_fy = calibration_param_.camera_intrinsic[4];

		float camera_cx = calibration_param_.camera_intrinsic[2];
		float camera_cy = calibration_param_.camera_intrinsic[5];


		float k1 = calibration_param_.camera_distortion[0];
		float k2 = calibration_param_.camera_distortion[1];
		float p1 = calibration_param_.camera_distortion[2];
		float p2 = calibration_param_.camera_distortion[3];
		float k3 = calibration_param_.camera_distortion[4];


		int nr = camera_height_;
		int nc = camera_width_;

#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{

			for (int c = 0; c < nc; c++)
			{



				int offset = r * camera_width_ + c;
				if (depth_map[offset] > 0)
				{
					//double undistort_x = c;
					//double undistort_y = r;
					//undistortPoint(c, r, camera_fx, camera_fy,
					//	camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

					float undistort_x = undistort_map_x_[offset];
					float undistort_y = undistort_map_y_[offset];

					point_cloud_map[3 * offset + 0] = (undistort_x - camera_cx) * depth_map[offset] / camera_fx;
					point_cloud_map[3 * offset + 1] = (undistort_y - camera_cy) * depth_map[offset] / camera_fy;
					point_cloud_map[3 * offset + 2] = depth_map[offset];


				}
				else
				{
					point_cloud_map[3 * offset + 0] = 0;
					point_cloud_map[3 * offset + 1] = 0;
					point_cloud_map[3 * offset + 2] = 0;
				}


			}

		}


		return DF_SUCCESS;
	}

	int XemaCamera::getCalibrationParam(struct CameraCalibParam& calibration_param)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = recv_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}

	int XemaCamera::connect(const char* camera_id)
	{

		LOG(INFO) << "DfConnect: ";
		/*******************************************************************************************************************/
		//�ر�log���
		el::Configurations conf;
		conf.setToDefault();
		//conf.setGlobally(el::ConfigurationType::Format, "[%datetime{%H:%m:%s} | %level] %msg"); 
#ifdef _WIN32 
	//conf.setGlobally(el::ConfigurationType::Filename, "log\\log_%datetime{%Y%M%d}.log");
		conf.setGlobally(el::ConfigurationType::Filename, "xema_log.log");
#elif __linux 
	//conf.setGlobally(el::ConfigurationType::Filename, "log/log_%datetime{%Y%M%d}.log");
		conf.setGlobally(el::ConfigurationType::Filename, "xema_log.log");
#endif 
		conf.setGlobally(el::ConfigurationType::Enabled, "true");
		conf.setGlobally(el::ConfigurationType::ToFile, "true");
		//conf.setGlobally(el::ConfigurationType::MaxLogFileSize, "204800");//1024*1024*1024=1073741824 
		el::Loggers::reconfigureAllLoggers(conf);
		el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");

		/*******************************************************************************************************************/

		registerOnDropped(on_dropped);


		int ret = connectNet(camera_id);
		if (ret != DF_SUCCESS)
		{
			return ret;
		}

		ret = getCalibrationParam(calibration_param_);

		if (ret != DF_SUCCESS)
		{
			disconnectNet();
			return ret;
		}

		int width, height;
		ret = getCameraResolution(&width, &height);
		if (ret != DF_SUCCESS)
		{
			disconnectNet();
			return ret;
		}

		if (width <= 0 || height <= 0)
		{
			disconnectNet();
			return DF_ERROR_2D_CAMERA;
		}

		camera_width_ = width;
		camera_height_ = height;

		camera_ip_ = camera_id;
		connected_flag_ = true;

		int image_size_ = camera_width_ * camera_height_;

		depth_buf_size_ = image_size_ * 1 * 4;
		depth_buf_ = (float*)(new char[depth_buf_size_]);

		pointcloud_buf_size_ = depth_buf_size_ * 3;
		point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

		trans_point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

		brightness_bug_size_ = image_size_;
		brightness_buf_ = new unsigned char[brightness_bug_size_];


		/******************************************************************************************************/
		//��������У����
		undistort_map_x_ = (float*)(new char[depth_buf_size_]);
		undistort_map_y_ = (float*)(new char[depth_buf_size_]);


		float camera_fx = calibration_param_.camera_intrinsic[0];
		float camera_fy = calibration_param_.camera_intrinsic[4];

		float camera_cx = calibration_param_.camera_intrinsic[2];
		float camera_cy = calibration_param_.camera_intrinsic[5];


		float k1 = calibration_param_.camera_distortion[0];
		float k2 = calibration_param_.camera_distortion[1];
		float p1 = calibration_param_.camera_distortion[2];
		float p2 = calibration_param_.camera_distortion[3];
		float k3 = calibration_param_.camera_distortion[4];


		int nr = camera_height_;
		int nc = camera_width_;

#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{

			for (int c = 0; c < nc; c++)
			{
				double undistort_x = c;
				double undistort_y = r;


				int offset = r * camera_width_ + c;

				undistortPoint(c, r, camera_fx, camera_fy,
					camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

				undistort_map_x_[offset] = (float)undistort_x;
				undistort_map_y_[offset] = (float)undistort_y;
			}

		}
		/*****************************************************************************************************************/

		el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
		el::Loggers::reconfigureAllLoggers(el::ConfigurationType::MaxLogFileSize, "104857600");//100MB 104857600

		/// ע��ص�����
		el::Helpers::installPreRollOutCallback(rolloutHandler);


		/********************************************************************************************************/



		return 0;
	}
	 
	int XemaCamera::disconnect(const char* camera_id)
	{ 
		LOG(INFO) << "disconnect:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}


		int ret = disconnectNet();

		if (DF_FAILED == ret)
		{
			return DF_FAILED;
		}

		delete[] depth_buf_;
		delete[] brightness_buf_;
		delete[] point_cloud_buf_;
		delete[] trans_point_cloud_buf_;
		delete[] undistort_map_x_;
		delete[] undistort_map_y_;


		connected_flag_ = false;

		/// ע���ص�����
		el::Helpers::uninstallPreRollOutCallback();

		return DF_SUCCESS;
	}
	 
	int XemaCamera::getCameraResolution(int* width, int* height)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetCameraResolution:";
		*width = camera_width_;
		*height = camera_height_;

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_CAMERA_RESOLUTION, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(width), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = recv_buffer((char*)(height), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{

			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);

		LOG(INFO) << "width: " << *width;
		LOG(INFO) << "height: " << *height;
		return DF_SUCCESS;

	}


	int XemaCamera::captureData(int exposure_num, char* timestamp)
	{

		LOG(INFO) << "captureData: " << exposure_num;
		bool ret = -1;

		if (exposure_num > 1)
		{
			switch (multiple_exposure_model_)
			{
			case 1:
			{
				ret = getFrameHdr(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}
			}
			break;
			case 2:
			{
				ret = getRepetitionFrame04(repetition_exposure_model_, depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
				if (DF_SUCCESS != ret)
				{
					return ret;
				}

			}
			default:
				break;
			}

		}
		else
		{

			ret = getFrame04(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
			if (DF_SUCCESS != ret)
			{
				return ret;
			}
		}


		std::string time = get_timestamp();
		for (int i = 0; i < time.length(); i++)
		{
			timestamp[i] = time[i];
		}

		transform_pointcloud_flag_ = false;


		return DF_SUCCESS;
	}
	 
	int XemaCamera::getDepthData(float* depth)
	{
		LOG(INFO) << "getDepthData:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}


		LOG(INFO) << "Trans Depth:";
		int point_num = camera_height_ * camera_width_;

		int nr = camera_height_;
		int nc = camera_width_;

#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{
			for (int c = 0; c < nc; c++)
			{
				int offset = r * camera_width_ + c;
				depth[offset] = depth_buf_[offset];

			}

		}

		LOG(INFO) << "Get Depth!";

		return 0;
	}


	int XemaCamera::getPointcloudData(float* point_cloud)
	{
		LOG(INFO) << "DfGetPointcloudData:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}


		if (!transform_pointcloud_flag_)
		{
			depthTransformPointcloud(depth_buf_, point_cloud_buf_);
			transform_pointcloud_flag_ = true;
		}

		memcpy(point_cloud, point_cloud_buf_, pointcloud_buf_size_);


		LOG(INFO) << "Get Pointcloud!";

		return 0;
	}


	int XemaCamera::getBrightnessData(unsigned char* brightness)
	{
		LOG(INFO) << "DfGetBrightnessData:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}


		LOG(INFO) << "Trans Brightness:";

		memcpy(brightness, brightness_buf_, brightness_bug_size_);

		//brightness = brightness_buf_;

		LOG(INFO) << "Get Brightness!";

		return 0;
	}


	int XemaCamera::getStandardPlaneParam(float* R, float* T)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetStandardPlaneParam: ";

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		int param_buf_size = 12 * 4;
		float* plane_param = new float[param_buf_size];

		ret = send_command(DF_CMD_GET_STANDARD_PLANE_PARAM, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, param_buf_size=" << param_buf_size;
			ret = recv_buffer((char*)plane_param, param_buf_size, g_sock);
			LOG(INFO) << "plane param received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get frame rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		close_socket(g_sock);


		memcpy(R, plane_param, 9 * 4);
		memcpy(T, plane_param + 9, 3 * 4);

		delete[] plane_param;

		LOG(INFO) << "Get plane param success";
		return DF_SUCCESS;

	}


	int XemaCamera::getHeightMapDataBaseParam(float* R, float* T, float* height_map)
	{
		LOG(INFO) << "DfGetHeightMapDataBaseParam:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}



		if (!transform_pointcloud_flag_)
		{
			depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
			transform_pointcloud_flag_ = true;
		}

		transformPointcloud((float*)point_cloud_buf_, (float*)trans_point_cloud_buf_, R, T);


		int nr = camera_height_;
		int nc = camera_width_;
#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{
			for (int c = 0; c < nc; c++)
			{
				int offset = r * camera_width_ + c;
				if (depth_buf_[offset] > 0)
				{
					height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
				}
				else
				{
					height_map[offset] = NULL;
				}

			}


		}


		LOG(INFO) << "Get Height Map!";

		return 0;
	}
	 
	int XemaCamera::getCalibrationParam(struct CalibrationParam* calibration_param)
	{
		LOG(INFO) << "DfGetCalibrationParam:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}

		//calibration_param = &calibration_param_;

		for (int i = 0; i < 9; i++)
		{
			calibration_param->intrinsic[i] = calibration_param_.camera_intrinsic[i];
		}

		for (int i = 0; i < 5; i++)
		{
			calibration_param->distortion[i] = calibration_param_.camera_distortion[i];
		}

		for (int i = 5; i < 12; i++)
		{
			calibration_param->distortion[i] = 0;
		}

		float extrinsic[4 * 4] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		for (int i = 0; i < 16; i++)
		{
			calibration_param->extrinsic[i] = extrinsic[i];
		}


		return 0;
	}


	int XemaCamera::getHeightMapData(float* height_map)
	{

		LOG(INFO) << "DfGetHeightMapData:";
		if (!connected_flag_)
		{
			return DF_NOT_CONNECT;
		}


		struct SystemConfigParam system_config_param;
		int ret_code = getSystemConfigParam(system_config_param);
		if (0 != ret_code)
		{
			std::cout << "Get Param Error;";
			return -1;
		}

		LOG(INFO) << "Transform Pointcloud:";

		if (!transform_pointcloud_flag_)
		{
			depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
			transform_pointcloud_flag_ = true;
		}

		//memcpy(trans_point_cloud_buf_, point_cloud_buf_, pointcloud_buf_size_);
		transformPointcloud((float*)point_cloud_buf_, (float*)trans_point_cloud_buf_, system_config_param.standard_plane_external_param, &system_config_param.standard_plane_external_param[9]);


		int nr = camera_height_;
		int nc = camera_width_;
#pragma omp parallel for
		for (int r = 0; r < nr; r++)
		{
			for (int c = 0; c < nc; c++)
			{
				int offset = r * camera_width_ + c;
				if (depth_buf_[offset] > 0)
				{
					height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
				}
				else
				{
					height_map[offset] = NULL;
				}

			}


		}


		LOG(INFO) << "Get Height Map!";

		return 0;
	}

	/************************************************************************************************/
	 

	int XemaCamera::setParamLedCurrent(int led)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamLedCurrent: " << led;
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_LED_CURRENT, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = send_buffer((char*)(&led), sizeof(led), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::getParamLedCurrent(int& led)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamLedCurrent: ";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = recv_buffer((char*)(&led), sizeof(led), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		LOG(INFO) << "led: " << led;
		return DF_SUCCESS;
	}
	

	int XemaCamera::setParamStandardPlaneExternal(float* R, float* T)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamStandardPlaneExternal: ";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			float plane_param[12];

			memcpy(plane_param, R, 9 * sizeof(float));
			memcpy(plane_param + 9, T, 3 * sizeof(float));

			ret = send_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);


			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;


	}
	 
	int XemaCamera::getParamStandardPlaneExternal(float* R, float* T)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamStandardPlaneExternal: ";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			//int param_buf_size = 12 * sizeof(float);
			//float* plane_param = new float[param_buf_size];
			float plane_param[12];

			ret = recv_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			memcpy(R, plane_param, 9 * sizeof(float));
			memcpy(T, plane_param + 9, 3 * sizeof(float));

		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamGenerateBrightness(int model, float exposure)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamGenerateBrightness: ";
		if (exposure < 20 || exposure> 1000000)
		{
			std::cout << "exposure param out of range!" << std::endl;
			return DF_FAILED;
		}

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = send_buffer((char*)(&model), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = send_buffer((char*)(&exposure), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}
	 
	int XemaCamera::getParamGenerateBrightness(int& model, float& exposure)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamGenerateBrightness: ";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&model), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = recv_buffer((char*)(&exposure), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamCameraExposure(float exposure)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamCameraExposure:";

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = send_buffer((char*)(&exposure), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}
 
	int XemaCamera::getParamCameraExposure(float& exposure)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamCameraExposure:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&exposure), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamMixedHdr(int num, int exposure_param[6], int led_param[6])
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamMixedHdr:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_MIXED_HDR, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			int param[13];
			param[0] = num;

			memcpy(param + 1, exposure_param, sizeof(int) * 6);
			memcpy(param + 7, led_param, sizeof(int) * 6);

			ret = send_buffer((char*)(param), sizeof(int) * 13, g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}

 
	int XemaCamera::getParamMixedHdr(int& num, int exposure_param[6], int led_param[6])
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamMixedHdr:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_MIXED_HDR, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			int param[13];

			ret = recv_buffer((char*)(param), sizeof(int) * 13, g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}


			memcpy(exposure_param, param + 1, sizeof(int) * 6);
			memcpy(led_param, param + 7, sizeof(int) * 6);
			num = param[0];

		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamCameraConfidence(float confidence)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamCameraConfidence:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_CAMERA_CONFIDENCE, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = send_buffer((char*)(&confidence), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}
 
	int XemaCamera::getParamCameraConfidence(float& confidence)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamCameraConfidence:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_CAMERA_CONFIDENCE, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&confidence), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamCameraGain(float gain)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamCameraGain:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_CAMERA_GAIN, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = send_buffer((char*)(&gain), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}
 
	int XemaCamera::getParamCameraGain(float& gain)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}
		LOG(INFO) << "DfGetParamCameraGain:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_CAMERA_GAIN, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&gain), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamSmoothing(int smoothing)
	{

		LOG(INFO) << "DfSetParamSmoothing:";
		int ret = -1;
		if (0 == smoothing)
		{
			ret = setParamBilateralFilter(0, 5);
		}
		else
		{
			ret = setParamBilateralFilter(1, 2 * smoothing + 1);
		}

		return ret;
	}

	int XemaCamera::getParamSmoothing(int& smoothing)
	{

		LOG(INFO) << "DfGetParamSmoothing:";
		int use = 0;
		int d = 0;

		int ret = getParamBilateralFilter(use, d);

		if (DF_FAILED == ret)
		{
			return DF_FAILED;
		}

		if (0 == use)
		{
			smoothing = 0;
		}
		else if (1 == use)
		{
			smoothing = d / 2;
		}
		return DF_SUCCESS;
	}


	int XemaCamera::setParamRadiusFilter(int use, float radius, int num)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamRadiusFilter:";
		if (use != 1 && use != 0)
		{
			std::cout << "use param should be 1 or 0:  " << use << std::endl;
			return DF_FAILED;
		}

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_RADIUS_FILTER, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = send_buffer((char*)(&use), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = send_buffer((char*)(&radius), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = send_buffer((char*)(&num), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}

	int XemaCamera::getParamRadiusFilter(int& use, float& radius, int& num)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamRadiusFilter:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_RADIUS_FILTER, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = recv_buffer((char*)(&radius), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = recv_buffer((char*)(&num), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::setParamOutlierFilter(float threshold)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfSetParamOutlierFilter:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_FISHER_FILTER, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = send_buffer((char*)(&threshold), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::getParamOutlierFilter(float& threshold)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "DfGetParamOutlierFilter:";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_FISHER_FILTER, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&threshold), sizeof(float), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}

	int XemaCamera::setParamMultipleExposureModel(int model)
	{
		if (model != 1 && model != 2)
		{
			return DF_ERROR_INVALID_PARAM;
		}
		multiple_exposure_model_ = model;

		return DF_SUCCESS;
	}
	 
	int XemaCamera::setParamRepetitionExposureNum(int num)
	{
		if (num < 2 || num >10)
		{
			return DF_ERROR_INVALID_PARAM;
		}

		repetition_exposure_model_ = num;

		return DF_SUCCESS;
	}
	/************************************************************************************************/



	/************************************************************************/
	std::tm* XemaCamera::gettm(long long timestamp)
	{
		auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //�˴�ת��Ϊ����������ʱ�䣬���������ʱ����Ҫ�������޸�
		auto mTime = std::chrono::milliseconds(milli);
		auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
		auto tt = std::chrono::system_clock::to_time_t(tp);
		std::tm* now = std::gmtime(&tt);
		//printf("%4d��%02d��%02d�� %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
		return now;
	}

	std::time_t XemaCamera::getTimeStamp(long long& msec)
	{
		std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
		auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
		seconds sec = duration_cast<seconds>(tp.time_since_epoch());


		std::time_t timestamp = tmp.count();

		msec = tmp.count() - sec.count() * 1000;
		//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
		return timestamp;
	}

	std::string XemaCamera::get_timestamp()
	{

		long long msec = 0;
		char time_str[7][16];
		auto t = getTimeStamp(msec);
		//std::cout << "Millisecond timestamp is: " << t << std::endl;
		auto time_ptr = gettm(t);
		sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //�·�Ҫ��1
		sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //�·�Ҫ��1
		sprintf(time_str[2], "%02d", time_ptr->tm_mday);//��
		sprintf(time_str[3], "%02d", time_ptr->tm_hour);//ʱ
		sprintf(time_str[4], "%02d", time_ptr->tm_min);// ��
		sprintf(time_str[5], "%02d", time_ptr->tm_sec);//ʱ
		sprintf(time_str[6], "%02lld", msec);// ��
		//for (int i = 0; i < 7; i++)
		//{
		//	std::cout << "time_str[" << i << "] is: " << time_str[i] << std::endl;
		//}

		std::string timestamp = "";

		timestamp += time_str[0];
		timestamp += "-";
		timestamp += time_str[1];
		timestamp += "-";
		timestamp += time_str[2];
		timestamp += " ";
		timestamp += time_str[3];
		timestamp += ":";
		timestamp += time_str[4];
		timestamp += ":";
		timestamp += time_str[5];
		timestamp += ",";
		timestamp += time_str[6];

		//std::cout << timestamp << std::endl;

		return timestamp;
	}



	int XemaCamera::getFrame04(float* depth, int depth_buf_size,unsigned char* brightness, int brightness_buf_size)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}
		LOG(INFO) << "GetFrame04"; 
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		ret = send_command(DF_CMD_GET_FRAME_04, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
			ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
			LOG(INFO) << "depth received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
			ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
			LOG(INFO) << "brightness received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			//brightness = (unsigned char*)depth + depth_buf_size;
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get frame rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get frame04 success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}


	int XemaCamera::getRepetitionFrame04(int count, float* depth, int depth_buf_size, unsigned char* brightness, int brightness_buf_size)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "GetRepetition01Frame04";
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		ret = send_command(DF_CMD_GET_REPETITION_FRAME_04, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = send_buffer((char*)(&count), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
			ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
			LOG(INFO) << "depth received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
			ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
			LOG(INFO) << "brightness received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}


			//brightness = (unsigned char*)depth + depth_buf_size;
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get frame rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		LOG(INFO) << "Get frame success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	 
	int XemaCamera::getFrameHdr(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		LOG(INFO) << "GetFrameHdr";
	//	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	//	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_FRAME_HDR, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
			ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
			LOG(INFO) << "depth received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
			ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
			LOG(INFO) << "brightness received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}



			//brightness = (unsigned char*)depth + depth_buf_size;
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get frame rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get frame success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}

	int XemaCamera::getSystemConfigParam(struct SystemConfigParam& config_param)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			ret = recv_buffer((char*)(&config_param), sizeof(config_param), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}

	 
	//���ܣ� ����˫���˲�����
	//��������� use�����أ�1Ϊ����0Ϊ�أ���param_d��ƽ��ϵ����3��5��7��9��11��
	//��������� ��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	int XemaCamera::setParamBilateralFilter(int use, int param_d)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		if (use != 1 && use != 0)
		{
			std::cout << "use param should be 1 or 0:  " << use << std::endl;
			return DF_FAILED;
		}

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_SET_PARAM_BILATERAL_FILTER, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = send_buffer((char*)(&use), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = send_buffer((char*)(&param_d), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}

	//�������� DfGetParamBilateralFilter
	//���ܣ� ��ȡ��϶��ع����������ع����Ϊ6�Σ�
	//��������� ��
	//��������� use�����أ�1Ϊ����0Ϊ�أ���param_d��ƽ��ϵ����3��5��7��9��11��
	//����ֵ�� ���ͣ�int��:����0��ʾ��ȡ�궨�����ɹ�;����-1��ʾ��ȡ�궨����ʧ��.
	int XemaCamera::getParamBilateralFilter(int& use, int& param_d)
	{
		std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
		while (!lck.try_lock_for(std::chrono::milliseconds(1)))
		{
			LOG(INFO) << "--";
		}

		int ret = setup_socket(camera_ip_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_PARAM_BILATERAL_FILTER, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{

			ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}

			ret = recv_buffer((char*)(&param_d), sizeof(int), g_sock);
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			close_socket(g_sock);
			return DF_BUSY;
		}
		else if (command == DF_CMD_UNKNOWN)
		{
			close_socket(g_sock);
			return DF_UNKNOWN;
		}

		close_socket(g_sock);
		return DF_SUCCESS;
	}



	/************************************************************************/
	void* createCamera() {
		return new XemaCamera;
	}

	void destroyCamera(void* pCamera) {
		delete reinterpret_cast<XemaCamera*>(pCamera);
	}

}
