#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
// #include "camera_dh.h"
#include <cassert>
#include "protocol.h"
#include <random>
#include <time.h>
#include <mutex>
#include <thread>
#include "easylogging++.h"
// #include "encode_cuda.cuh"
#include "system_config_settings.h"
#include "version.h"
#include "configure_standard_plane.h"
#include "../test/LookupTableFunction.h" 
#include "configure_auto_exposure.h"
#include <JetsonGPIO.h>
#include "scan3d.h"
#include "socket_tcp.h"
#include <dirent.h>
#include "power_controller.h"


INITIALIZE_EASYLOGGINGPP
#define INPUT_PIN           5           // BOARD pin 29, BCM pin 5
#define OUTPUT1_PIN         6           // BOARD pin 31, BCM pin 6
#define ACT_PIN             12          // BOARD pin 32, BCM pin 12
#define OUTPUT2_PIN         13          // BOARD pin 33, BCM pin 13
#define LED_CTL_PIN         19          // BOARD pin 35, BCM pin 19


Scan3D scan3d_;
PowerController pwrctrl;

std::random_device rd;
std::mt19937 rand_num(rd());
bool connected = false;
long long current_token = 0;

// heart beat
unsigned int heartbeat_timer = 0;
std::mutex mtx_heartbeat_timer;
std::thread heartbeat_thread;

// CameraDh camera;
LightCrafter3010 lc3010;
struct CameraCalibParam param;

int brightness_current = 100;
float generate_brightness_exposure_time = 12000;
int generate_brightness_model = 1;
 
float max_camera_exposure_ = 28000;
float min_camera_exposure_ = 1000;

int camera_width_ = 0;
int camera_height_ = 0;
 
int frame_status_ = DF_SUCCESS;

SystemConfigDataStruct system_config_settings_machine_;

bool readSystemConfig()
{
    return system_config_settings_machine_.loadFromSettings("../system_config.ini");
}

bool saveSystemConfig()
{
    return system_config_settings_machine_.saveToSettings("../system_config.ini");
}

int reboot_lightcraft()
{
    LOG(ERROR)<<"reboot lightcraft"; 
    pwrctrl.off_projector();
    LOG(ERROR)<<"wait..."; 
    sleep(6);

    int operate_num = 3;

    while (operate_num-- > 0)
    {
        int ret = lc3010.init();
        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "lc3010 init FAILED; CODE : " << ret;
            sleep(1);
        }
        else
        {
            LOG(INFO)<<"init lightcraft!";
            break;
        }
    }

    operate_num = 3;
    while (operate_num-- > 0)
    {
        int ret = lc3010.SetLedCurrent(system_config_settings_machine_.Instance().config_param_.led_current,
                                       system_config_settings_machine_.Instance().config_param_.led_current,
                                       system_config_settings_machine_.Instance().config_param_.led_current);

        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "lc3010 SetLedCurrent FAILED; CODE : " << ret;
            usleep(100000);
        }
        else
        {
            
            LOG(INFO)<<"set led current!";
            break;
        }
    }

    return 0;
}

int reboot_system()
{
    LOG(ERROR)<<"reboot board"; 
    pwrctrl.off_board();
    return 0;
}


void handle_error(int code)
{
    if (DF_SUCCESS != code)
    {
        LOG(ERROR) << "handle error: " << code;

        frame_status_ = code;

        switch (code)
        {
        case DF_ERROR_LOST_TRIGGER:
        {
            reboot_lightcraft();
        }
        break;

        case DF_ERROR_LIGHTCRAFTER_SET_PATTERN_ORDER:
        {
            reboot_lightcraft();
        }
        break;
        case DF_ERROR_CAMERA_STREAM:
        {
            if (DF_ERROR_2D_CAMERA == scan3d_.reopenCamera())
            {
                reboot_system();
            }
        }
        break;
        default:
            break;
        }
    }
}


bool findMaskBaseConfidence(cv::Mat confidence_map, int threshold, cv::Mat& mask)
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


bool set_projector_version(int version)
{
    switch (version)
    {
    case DF_PROJECTOR_3010:
    {
        // cuda_set_camera_version(DFX_800);
        max_camera_exposure_ = 100000;
        min_camera_exposure_ = 1000;
        return true;
    }
    break;

    case DF_PROJECTOR_4710:
    {

        // cuda_set_camera_version(DFX_1800);
        max_camera_exposure_ = 28000; 
        min_camera_exposure_ = 1000;
        return true;
    }
    break;

    default:
        break;
    }

    return false;
}

int heartbeat_check()
{
    while (connected)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        mtx_heartbeat_timer.lock();
        heartbeat_timer ++;
        if (heartbeat_timer > 30)
        {
            LOG(INFO) << "HeartBeat stopped!";
            connected = false;
            current_token = 0;
        }
        mtx_heartbeat_timer.unlock();        
    }
    return 0;
}

long long generate_token()
{
    long long token = rand_num();
    return token;
}


float read_temperature(int flag)
{
    float val = -1.0;

    switch(flag)
    {
        case 0:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone0/temp");
            infile >> data;
            // std::cout << "first read data from file1.dat == " << data << std::endl;
 
            val = (float)std::atoi(data) / 1000.0; 

        }
        break;

        case 1:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone1/temp");
            infile >> data;
            
            val = (float)std::atoi(data) / 1000.0; 
        }
        break;

        case 2:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone2/temp");
            infile >> data;
            
            val =(float)std::atoi(data) / 1000.0; 
        }
        break;

        default:
        break;
    }
   
 

    return val;
}



int handle_cmd_connect(int client_sock)
{
    int ret;
    if (connected)
    {
        LOG(INFO) << "new connection rejected" << std::endl;
        return send_command(client_sock, DF_CMD_REJECT);
    }
    else
    {
        ret = send_command(client_sock, DF_CMD_OK);
        if (ret == DF_FAILED)
        {
          
            LOG(INFO) << "send_command FAILED" ;
            return DF_FAILED;
        }
        long long token = generate_token();
        ret = send_buffer(client_sock, (char *)&token, sizeof(token));
        if (ret == DF_FAILED)
        {
            return DF_FAILED;
        }
        connected = true;
        current_token = token;

        mtx_heartbeat_timer.lock();
        heartbeat_timer = 0;
        mtx_heartbeat_timer.unlock();

        if (heartbeat_thread.joinable())
        {
            heartbeat_thread.join();
        }
        heartbeat_thread = std::thread(heartbeat_check);

        LOG(INFO)<<"connection established, current token is: "<<current_token;
        return DF_SUCCESS;
    }
}

int handle_cmd_unknown(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    //std::cout<<"token ret = "<<ret<<std::endl;
    //std::cout<<"checking token:"<<token<<std::endl;
    if(ret == DF_FAILED)
    {
    	return DF_FAILED;
    }

    if(token == current_token)
    {
        ret = send_command(client_sock, DF_CMD_UNKNOWN);

        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        else
        {
            return DF_SUCCESS;
        }
    }
    else
    {
        LOG(INFO)<<"reject"<<std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED" ;
            return DF_FAILED;
        }
        else
        {
            return DF_SUCCESS;
        }
    }
}

int check_token(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char *)&token, sizeof(token));
    // std::cout<<"token ret = "<<ret<<std::endl;
    // std::cout<<"checking token:"<<token<<std::endl;
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "recv_buffer token FAILED";
        return DF_FAILED;
    }

    if (token == current_token)
    {
        ret = send_command(client_sock, DF_CMD_OK);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        return DF_SUCCESS;
    }
    else
    {
        LOG(INFO) << "reject" << std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        return DF_FAILED;
    }
}

int handle_cmd_disconnect(int client_sock)
{
    LOG(INFO) << "handle_cmd_disconnect";
    long long token = 0;
    int ret = recv_buffer(client_sock, (char *)&token, sizeof(token));
    LOG(INFO) << "token " << token << " trying to disconnect" ;
    if (ret == DF_FAILED)
    {
        return DF_FAILED;
    }
    if (token == current_token)
    {
        connected = false;
        current_token = 0;
        LOG(INFO) << "client token=" << token << " disconnected";
        ret = send_command(client_sock, DF_CMD_OK);
        if (ret == DF_FAILED)
        {
            LOG(INFO)<<"send_command FAILED";
            return DF_FAILED;
        }
    }
    else
    {
        LOG(INFO)<< "disconnect rejected" << std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
        }

        return DF_FAILED;
    }

    if (heartbeat_thread.joinable())
    {
        heartbeat_thread.join();
    }
    return DF_SUCCESS;
}

bool inspect_board()
{
     
    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 


    scan3d_.captureFrame04BaseConfidence(); 
    scan3d_.copyBrightnessData(brightness); 
 
    std::vector<cv::Point2f> points;
    cv::Mat img(camera_height_,camera_width_,CV_8U,brightness);

    ConfigureStandardPlane plane_machine; 
    bool found = plane_machine.findCircleBoardFeature(img,points);

    delete []brightness;

    GPIO::output(OUTPUT1_PIN, GPIO::HIGH);

    if (found) {
	    GPIO::output(OUTPUT2_PIN, GPIO::HIGH);
    } else {
	    GPIO::output(OUTPUT2_PIN, GPIO::LOW);
    }

    return found;
}

int handle_cmd_set_board_inspect(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }
 
    int use =1;

    int ret = recv_buffer(client_sock, (char*)(&use), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
        LOG(INFO)<<"set board inspect: "<<use;
    
    if(1 == use)
    {
        //注册板检测中断函数
        GPIO::add_event_detect(INPUT_PIN, GPIO::Edge::RISING, inspect_board, 1);
        LOG(INFO)<<"Regist Interrupt Callback\n";
    }
    else
    {
        //注消板检测中断函数
        GPIO::add_event_detect(INPUT_PIN, GPIO::Edge::RISING);
        LOG(INFO)<<"Cancle Interrupt Callback\n";
    }

    return DF_SUCCESS;
}

int handle_cmd_set_auto_exposure_base_board(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = camera_width_ * camera_height_;
    char *buffer = new char[buffer_size];

    ConfigureAutoExposure auto_exposure;
    float average_pixel = 0;
    float over_exposure_rate = 0;


    cv::Mat brightness_mat(camera_height_,camera_width_,CV_8U,cv::Scalar(0));

    float current_exposure = system_config_settings_machine_.Instance().config_param_.camera_exposure_time;

    bool capture_one_ret = scan3d_.captureTextureImage(2, current_exposure, (unsigned char *)brightness_mat.data);

    auto_exposure.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

    return DF_SUCCESS;
}

int handle_cmd_set_auto_exposure_base_roi_half(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = camera_width_ * camera_height_;
    char *buffer = new char[buffer_size];

    ConfigureAutoExposure auto_exposure_machine;
    float average_pixel = 0;
    float over_exposure_rate = 0;

    float high_max_over_rate = 1.0;
    float high_min_over_rate = 0.7;

    float low_max_over_rate = 0.3;
    float low_min_over_rate = 0.2;
 
    cv::Mat brightness_mat(camera_height_,camera_width_,CV_8U,cv::Scalar(0));


    int current_exposure = (min_camera_exposure_ + max_camera_exposure_)/2;

    int adjust_exposure_val = current_exposure;

    int low_limit_exposure = min_camera_exposure_;
    int high_limit_exposure = max_camera_exposure_;

    //电流值设置到最大
    if (brightness_current < 1023 && current_exposure != min_camera_exposure_)
    {
        brightness_current = 1023;
        if (DF_SUCCESS != lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current))
        {
            LOG(ERROR) << "Set Led Current";
             
        }
        system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
    }

    //发光，自定义曝光时间
    // lc3010.enable_solid_field();
    // bool capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
    
    bool capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
    auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);


    int iterations_num = 0;
    while(iterations_num< 16)
    {
        if(average_pixel> 127.5 && average_pixel < 128.5)
        {
            break;
        }

        if(average_pixel<= 127.5)
        {
            //偏暗
            low_limit_exposure = current_exposure;
            adjust_exposure_val /= 2;
            current_exposure = current_exposure + adjust_exposure_val;
        }
        else if(average_pixel >= 128.5)
        {
            //偏亮
            high_limit_exposure = current_exposure;
            adjust_exposure_val /= 2;
            current_exposure = current_exposure - adjust_exposure_val;

        }

        // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
        capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);

        iterations_num++;

        LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
        LOG(INFO) << "current_exposure: " << current_exposure;
        LOG(INFO) << "low_limit_exposure: " << low_limit_exposure;
        LOG(INFO) << "high_limit_exposure: " << high_limit_exposure;
        LOG(INFO) << "iterations_num: " << iterations_num;
        LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
        LOG(INFO) << "average_pixel: " << average_pixel;
        LOG(INFO) << "";
    }
    
    /**************************************************************************************************/

    if(average_pixel> 127.5 && average_pixel < 128.5)
    {

        //根据过曝光情况调整
        if (over_exposure_rate < low_min_over_rate)
        {
            //需要加亮
            low_limit_exposure = current_exposure;
            high_limit_exposure = max_camera_exposure_;
            current_exposure = (high_limit_exposure - low_limit_exposure) / 2;
            adjust_exposure_val = current_exposure;
        }
        else if (over_exposure_rate > high_max_over_rate)
        {
            //需要变暗
            low_limit_exposure = min_camera_exposure_;
            high_limit_exposure = current_exposure;
            current_exposure = (high_limit_exposure - low_limit_exposure) / 2;
            adjust_exposure_val = current_exposure;
        }

        // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

        
        iterations_num = 0;
        while (iterations_num < 16)
        {
            if (over_exposure_rate >= low_min_over_rate && over_exposure_rate < high_max_over_rate)
            {
                break;
            }

            if (over_exposure_rate < low_min_over_rate)
            {
                //偏暗
                low_limit_exposure = current_exposure;
                adjust_exposure_val /= 2;
                current_exposure = current_exposure + adjust_exposure_val;
            }
            else if (over_exposure_rate >= high_max_over_rate)
            {
                //偏亮
                high_limit_exposure = current_exposure;
                adjust_exposure_val /= 2;
                current_exposure = current_exposure - adjust_exposure_val;
            }

            // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            iterations_num++;

            LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "low_limit_exposure: " << low_limit_exposure;
            LOG(INFO) << "high_limit_exposure: " << high_limit_exposure;
            LOG(INFO) << "iterations_num: " << iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) << "";
        }
    }
    else if(average_pixel>= 128.5)
    {
        //太亮、曝光时间设置成最小值、调整led值
        current_exposure = min_camera_exposure_;

        int low_limit_led = 0;
        int high_limit_led = 1023;

        int current_led = (high_limit_led - low_limit_led) / 2;
        int adjust_led_val = current_led;

        brightness_current = current_led;

        if (DF_SUCCESS != lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current))
        {
            LOG(ERROR) << "Set Led Current";
        }

        // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

        iterations_num = 0;
        while (iterations_num < 10)
        {
            if (average_pixel > 127.5 && average_pixel < 128.5)
            {
                break;
            }

        if(average_pixel<= 127.5)
        {
            //偏暗
            low_limit_led = current_led;
            adjust_led_val /= 2;
            current_led = current_led + adjust_led_val;
        }
        else if(average_pixel >= 128.5)
        {
            //偏亮
            high_limit_led = current_led;
            adjust_led_val /= 2;
            current_led = current_led - adjust_led_val;

        }

        brightness_current = current_led;
        if (DF_SUCCESS != lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current))
        {
            LOG(ERROR) << "Set Led Current";
        }
        // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
        capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);

        iterations_num++;

        LOG(INFO) << "adjust_led_val: " << adjust_led_val;
        LOG(INFO) << "current_exposure: " << current_exposure;
        LOG(INFO) << "low_limit_led: " << low_limit_exposure;
        LOG(INFO) << "high_limit_led: " << high_limit_led;
        LOG(INFO) << "iterations_num: " << iterations_num;
        LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
        LOG(INFO) << "average_pixel: " << average_pixel;
        LOG(INFO) << "";
        }

    }
    else if(average_pixel< 127.5)
    {
        //太暗，曝光设置成最大值
        current_exposure = max_camera_exposure_;
        // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        capture_one_ret = scan3d_.captureTextureImage(1, current_exposure, (unsigned char *)brightness_mat.data);
    }

    /***************************************************************************************************/

    
    // lc3010.disable_solid_field();

    system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;

    int auto_exposure = current_exposure;
    int auto_led = brightness_current;

    int ret = send_buffer(client_sock, (char*)(&auto_exposure), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&auto_led), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }


    return DF_SUCCESS;

}

int handle_cmd_set_auto_exposure_base_roi_pid(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int buffer_size = camera_width_ * camera_height_;
    char *buffer = new char[buffer_size];

    ConfigureAutoExposure auto_exposure_machine;
    float average_pixel = 0;
    float over_exposure_rate = 0;


    cv::Mat brightness_mat(camera_height_,camera_width_,CV_8U,cv::Scalar(0));

    float current_exposure = system_config_settings_machine_.Instance().config_param_.camera_exposure_time;

    //发光，自定义曝光时间
    // lc3010.enable_solid_field();
    // bool capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char*)brightness_mat.data); 
    bool capture_one_ret = scan3d_.captureTextureImage(2, current_exposure, (unsigned char *)brightness_mat.data);
    auto_exposure_machine.evaluateBrightnessParam(brightness_mat,cv::Mat(),average_pixel,over_exposure_rate);
 
    int adjust_exposure_val = 1000;
    int adjust_led_= 128;

    int adjust_led_val = 1000; 
    int current_led = 0;
    
    float high_max_over = 1.0;
    float high_min_over = 0.7;

    float low_max_over = 0.3;
    float low_min_over = 0.2;
 /*******************************************************************************************************************/
   
    /*******************************************************************************************************************/
    //pid 调节到128
    float Kp = 200;
    float Ki = 0.005;
    float Kd = 0.1;

    float error_p =0;
    float error_i =0;
    float error_d =0;
    float error_dp =0; 
    int iterations_num = 0;

	while (iterations_num< 30) {
        error_p = 128 - average_pixel;
        error_i += error_p;
        error_d = error_p - error_dp;
        error_dp = error_p;

        adjust_exposure_val = Kp * error_p + Ki * error_i + Kd * error_d;
        current_exposure += Kp * error_p + Ki * error_i + Kd * error_d;

        if (brightness_current < 1023 && current_exposure != min_camera_exposure_)
        {
            brightness_current = 1023;

            if (DF_SUCCESS != lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current))
            {
            LOG(ERROR) << "Set Led Current";
            }
            system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
        }

        if (current_exposure > max_camera_exposure_)
        {
            current_exposure = max_camera_exposure_;
        }

        if(current_exposure< min_camera_exposure_)
        {
            current_exposure = min_camera_exposure_;
        }


        iterations_num++;
        // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
        capture_one_ret = scan3d_.captureTextureImage(2, current_exposure, (unsigned char *)brightness_mat.data);
        auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

        LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
        LOG(INFO) << "current_exposure: " << current_exposure;
        LOG(INFO) << "current_led: " << brightness_current;
        LOG(INFO) << "iterations_num: " << iterations_num;
        LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
        LOG(INFO) << "average_pixel: " << average_pixel;
        LOG(INFO) << "";

        if(average_pixel< 128.5 && average_pixel> 127.5)
        {
            break;
        }

        //最大亮度还不够
        if(average_pixel< 127.5 && current_exposure == max_camera_exposure_)
        {
            break;
        }

        //最小还是过曝光
        if(average_pixel> 127.5 && current_exposure == min_camera_exposure_ && over_exposure_rate > high_max_over)
        {
            break;
        } 
    } 

    /********************************************************************************************************************/
    //过调led
    //调LED
    if (current_exposure == min_camera_exposure_ && over_exposure_rate > high_max_over)
    {
        //pid 调节到128
        float led_Kp = 6;
        float led_Ki = 0.5;
        float led_Kd = 1;

        float led_error_p = 0;
        float led_error_i = 0;
        float led_error_d = 0;
        float led_error_dp = 0;
        int led_iterations_num = 0;

        while (led_iterations_num < 30)
        {
            led_error_p = 128 - average_pixel;
            led_error_i += led_error_p;
            led_error_d = led_error_p - led_error_dp;
            led_error_dp = led_error_p;

            adjust_led_val = led_Kp * led_error_p + led_Ki * led_error_i + led_Kd * led_error_d;
            current_led += led_Kp * led_error_p + led_Ki * led_error_i + led_Kd * led_error_d;
 
            if (current_led > 1023)
            {
                current_led = 1023;
            }

            if(current_led< 0)
            {
                current_led = 0;
            }

            led_iterations_num++;

            if (DF_SUCCESS != lc3010.SetLedCurrent(current_led, current_led, current_led))
            {
                LOG(ERROR) << "Set Led Current";
            }
            // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            capture_one_ret = scan3d_.captureTextureImage(2, current_exposure, (unsigned char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            LOG(INFO) <<""; 
            LOG(INFO) << "adjust_led_val: " << adjust_led_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "current_led: " << current_led;
            LOG(INFO) << "iterations_num: " << led_iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) <<""; 

            if (average_pixel < 128.5 && average_pixel > 127.5)
            {
                break;
            }

            //最大亮度还不够
            if (average_pixel < 127.5 && current_led == 1023)
            {
                break;
            }

            //最小还是过曝光
            if (average_pixel > 127.5 && current_led == 0 && over_exposure_rate > high_max_over)
            {
                break;
            }
        }
    }

   
    /********************************************************************************************************************/
    //微调
    if (average_pixel > 128 && over_exposure_rate < low_min_over)
    {
        //调节到low_min_over

        float Kp = 200;
        float Ki = 0.5;
        float Kd = 1;

        float error_p = 0;
        float error_i = 0;
        float error_d = 0;
        float error_dp = 0;
        int iterations_num = 0;

        while (iterations_num < 30)
        {
            error_p = over_exposure_rate - (low_min_over + low_max_over) / 2.0;
            error_i += error_p;
            error_d = error_p - error_dp;
            error_dp = error_p;

            adjust_exposure_val = Kp * error_p + Ki * error_i + Kd * error_d;
            current_exposure += Kp * error_p + Ki * error_i + Kd * error_d;

            if (current_exposure > max_camera_exposure_)
            {
                current_exposure = max_camera_exposure_;
            }

            if(current_exposure< min_camera_exposure_)
            {
                current_exposure = min_camera_exposure_;
            }
            iterations_num++;
            // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            capture_one_ret = scan3d_.captureTextureImage(2, current_exposure, (unsigned char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "current_led: " << brightness_current;
            LOG(INFO) << "iterations_num: " << iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) << "";

            //最小还是过曝光
            if (over_exposure_rate > low_min_over && over_exposure_rate < low_max_over)
            {
                break;
            }
        }
    }
    else
    {
        //调节到high_min_over

        float Kp = 200;
        float Ki = 0.5;
        float Kd = 1;

        float error_p = 0;
        float error_i = 0;
        float error_d = 0;
        float error_dp = 0;
        int iterations_num = 0;

        while (iterations_num < 30)
        {
            error_p = over_exposure_rate - (high_min_over + high_max_over) / 2.0;
            error_i += error_p;
            error_d = error_p - error_dp;
            error_dp = error_p;

            adjust_exposure_val = Kp * error_p + Ki * error_i + Kd * error_d;
            current_exposure += Kp * error_p + Ki * error_i + Kd * error_d;

            if (current_exposure > max_camera_exposure_)
            {
                current_exposure = max_camera_exposure_;
            }

            if(current_exposure< min_camera_exposure_)
            {
                current_exposure = min_camera_exposure_;
            }

            iterations_num++;
            // capture_one_ret = camera.captureSingleExposureImage(current_exposure, (char *)brightness_mat.data);
            capture_one_ret = scan3d_.captureTextureImage(2, current_exposure, (unsigned char *)brightness_mat.data);
            auto_exposure_machine.evaluateBrightnessParam(brightness_mat, cv::Mat(), average_pixel, over_exposure_rate);

            LOG(INFO) << "adjust_exposure_val: " << adjust_exposure_val;
            LOG(INFO) << "current_exposure: " << current_exposure;
            LOG(INFO) << "current_led: " << brightness_current;
            LOG(INFO) << "iterations_num: " << iterations_num;
            LOG(INFO) << "over_exposure_rate: " << over_exposure_rate;
            LOG(INFO) << "average_pixel: " << average_pixel;
            LOG(INFO) << "";

            //最小还是过曝光
            if (over_exposure_rate > high_min_over && over_exposure_rate < high_max_over)
            {
                break;
            }
        }

    }

    // lc3010.disable_solid_field();

    int auto_exposure = current_exposure;
    int auto_led = brightness_current;

    int ret = send_buffer(client_sock, (char*)(&auto_exposure), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&auto_led), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }


    return DF_SUCCESS;
}

int handle_cmd_get_focusing_image(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
    LOG(INFO) << "capture focusing image";

    int buffer_size = camera_width_*camera_height_;
    unsigned char* buffer = new unsigned char[buffer_size];

    //不发光，自定义曝光时间
    
    bool capture_one_ret = scan3d_.captureTextureImage(3,system_config_settings_machine_.Instance().config_param_.camera_exposure_time,buffer); 

    LOG(TRACE) << "start send image, image_size=" << buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);
    delete[] buffer;
    if (ret == DF_FAILED)
    {
        LOG(ERROR) << "send error, close this connection!";
        return DF_FAILED;
    }
    LOG(TRACE) << "image sent!";
    return DF_SUCCESS;
}


int handle_cmd_get_brightness(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }
    LOG(INFO)<<"capture single image";

    int buffer_size = camera_width_*camera_height_;
    unsigned char* buffer = new unsigned char[buffer_size];

    scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,buffer);
  
    LOG(INFO)<<"start send image, image_size="<<buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);
    delete [] buffer;
    if(ret == DF_FAILED)
    {
        LOG(ERROR)<<"send error, close this connection!";
	    return DF_FAILED;
    }
    LOG(INFO)<<"image sent!";
    return DF_SUCCESS;
}



int handle_cmd_get_raw_04_repetition(int client_sock)
{

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int repetition_count = 1;
    //接收重复数
    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }



    // lc3010.pattern_mode04_repetition(repetition_count); 


    int image_num= 19 + 6*(repetition_count-1);  
    int buffer_size = camera_width_*camera_height_*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    // camera.captureRawTest(image_num,buffer);

    
    scan3d_.captureRaw04Repetition01(repetition_count,buffer);

    LOG(INFO)<<"start send image, buffer_size= "<<buffer_size;

    ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
}


int handle_cmd_get_raw_04(int client_sock)
{
  
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 19;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw04(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
  
}


int handle_cmd_get_raw_05(int client_sock)
{
  
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 16;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw05(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
  
}


int handle_cmd_get_raw_06(int client_sock)
{
  
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 16;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw06(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
  
}

int handle_cmd_get_raw_08(int client_sock)
{
  
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 24;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw08(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
  
}


int handle_cmd_get_raw_03(int client_sock)
{
   
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 31;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw03(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
}


int handle_cmd_get_raw_02(int client_sock)
{ 

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 37;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw02(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
}


int handle_cmd_get_raw_01(int client_sock)
{
   
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    } 

    int image_num= 24;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];
    
    scan3d_.captureRaw01(buffer); 

    LOG(INFO)<<"start send image, buffer_size= "<< buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO)<<"ret= "<<ret;

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!"; 
        delete [] buffer;
        return DF_FAILED;
    }

    LOG(INFO)<<"image sent!";

    delete [] buffer;
    return DF_SUCCESS;
   
}  



int handle_cmd_get_frame_06_repetition_black(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	
    frame_status_ = DF_FRAME_CAPTURING;
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
        frame_status_ = DF_ERROR_NETWORK;
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    ret = scan3d_.captureFrame06RepetitionMono12(repetition_count);
    if (DF_SUCCESS != ret)
    {
    //   LOG(ERROR) << "captureFrame04BaseConfidence code: " << ret;
    //   frame_status_ = ret;
      
        handle_error(ret);
    }
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

        
    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
         scan3d_.copyBrightnessData(brightness);
    }

    // scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map); 
 
    LOG(INFO)<<"capture Frame04 Repetition02 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }


   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06 repetition";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

    

}



int handle_cmd_get_frame_06_repetition_color(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	
    frame_status_ = DF_FRAME_CAPTURING;
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
        frame_status_ = DF_ERROR_NETWORK;
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    ret = scan3d_.captureFrame06RepetitionColor(repetition_count);
    if (DF_SUCCESS != ret)
    {
    //   LOG(ERROR) << "captureFrame04BaseConfidence code: " << ret;
    //   frame_status_ = ret;
      
        handle_error(ret);
    }
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

        
    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
         scan3d_.copyBrightnessData(brightness);
    }

    // scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map); 
 
    LOG(INFO)<<"capture Frame04 Repetition02 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }


   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06 repetition";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

    

}

 
int handle_cmd_get_frame_06_hdr_color(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"Frame06 HDR Exposure:"; 
    frame_status_ = DF_FRAME_CAPTURING;


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret = scan3d_.captureFrame06HdrColor(); 

    if(DF_SUCCESS != ret)
    { 
        //  LOG(ERROR)<<"captureFrame06Hdr code: "<<ret;
        //  frame_status_ = ret;
         
        handle_error(ret);
    }

    // std::thread  t_merge_brightness(&Scan3D::mergeBrightness, &scan3d_);
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();
         
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  
    LOG(INFO)<<"Reconstruct Frame04 Finished!";
   

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        
        // t_merge_brightness.join();
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    // t_merge_brightness.join();
    // scan3d_.copyBrightnessData(brightness);

    if (1 != generate_brightness_model)
    {
        // t_merge_brightness.detach();
        scan3d_.captureTextureImage(generate_brightness_model, generate_brightness_exposure_time, brightness);
    }
    else
    {
        // t_merge_brightness.join();
        scan3d_.copyBrightnessData(brightness);
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;  

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

}


int handle_cmd_get_frame_06_black(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"Frame06 HDR Exposure:"; 
    frame_status_ = DF_FRAME_CAPTURING;


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret = scan3d_.captureFrame06Mono12(); 

    if(DF_SUCCESS != ret)
    { 
        //  LOG(ERROR)<<"captureFrame06Hdr code: "<<ret;
        //  frame_status_ = ret;
         
        handle_error(ret);
    }

    // std::thread  t_merge_brightness(&Scan3D::mergeBrightness, &scan3d_);
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();
         
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  
    LOG(INFO)<<"Reconstruct Frame04 Finished!";
   

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        
        // t_merge_brightness.join();
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    // t_merge_brightness.join();
    // scan3d_.copyBrightnessData(brightness);

    if (1 != generate_brightness_model)
    {
        // t_merge_brightness.detach();
        scan3d_.captureTextureImage(generate_brightness_model, generate_brightness_exposure_time, brightness);
    }
    else
    {
        // t_merge_brightness.join();
        scan3d_.copyBrightnessData(brightness);
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;  

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

}


int handle_cmd_get_frame_06_hdr_black(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"Frame06 HDR Exposure:"; 
    frame_status_ = DF_FRAME_CAPTURING;


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret = scan3d_.captureFrame06HdrMono12(); 

    if(DF_SUCCESS != ret)
    { 
        //  LOG(ERROR)<<"captureFrame06Hdr code: "<<ret;
        //  frame_status_ = ret;
         
        handle_error(ret);
    }

    // std::thread  t_merge_brightness(&Scan3D::mergeBrightness, &scan3d_);
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();
         
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  
    LOG(INFO)<<"Reconstruct Frame04 Finished!";
   

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        
        // t_merge_brightness.join();
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    // t_merge_brightness.join();
    // scan3d_.copyBrightnessData(brightness);

    if (1 != generate_brightness_model)
    {
        // t_merge_brightness.detach();
        scan3d_.captureTextureImage(generate_brightness_model, generate_brightness_exposure_time, brightness);
    }
    else
    {
        // t_merge_brightness.join();
        scan3d_.copyBrightnessData(brightness);
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;  

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

}


int handle_cmd_get_frame_06_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"Frame06 HDR Exposure:"; 
    frame_status_ = DF_FRAME_CAPTURING;


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret = scan3d_.captureFrame06Hdr(); 

    if(DF_SUCCESS != ret)
    { 
        //  LOG(ERROR)<<"captureFrame06Hdr code: "<<ret;
        //  frame_status_ = ret;
         
        handle_error(ret);
    }

    // std::thread  t_merge_brightness(&Scan3D::mergeBrightness, &scan3d_);
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();
         
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  
    LOG(INFO)<<"Reconstruct Frame04 Finished!";
   

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        
        // t_merge_brightness.join();
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    // t_merge_brightness.join();
    // scan3d_.copyBrightnessData(brightness);

    if (1 != generate_brightness_model)
    {
        // t_merge_brightness.detach();
        scan3d_.captureTextureImage(generate_brightness_model, generate_brightness_exposure_time, brightness);
    }
    else
    {
        // t_merge_brightness.join();
        scan3d_.copyBrightnessData(brightness);
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;  

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

}


int handle_cmd_get_frame_04_hdr_parallel_mixed_led_and_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    LOG(INFO)<<"Mixed HDR Exposure:"; 
    frame_status_ = DF_FRAME_CAPTURING;


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    int ret = scan3d_.captureFrame04HdrBaseConfidence(); 

    if(DF_SUCCESS != ret)
    { 
        handle_error(ret);
        //  LOG(ERROR)<<"captureFrame04BaseConfidence code: "<<ret;
        //  frame_status_ = ret;

 
        //  switch (ret)
        //  {
        //  case DF_ERROR_LOST_TRIGGER:
        //  {
        //     reboot_lightcraft();
        //  }
        //         break;
        //  case DF_ERROR_CAMERA_STREAM:
        //  {
        //     if(DF_ERROR_2D_CAMERA == scan3d_.reopenCamera())
        //     {
        //         reboot_system();
        //     }
        //  }
        //         break;
        //  default:
        //         break;
        //  }

    }

    // std::thread  t_merge_brightness(&Scan3D::mergeBrightness, &scan3d_);
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();
         
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  
    LOG(INFO)<<"Reconstruct Frame04 Finished!";
   

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";
    }

   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        
        // t_merge_brightness.join();
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }


    if(1!=generate_brightness_model)
    {
        // t_merge_brightness.detach();
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    {
        // t_merge_brightness.join();
        scan3d_.copyBrightnessData(brightness);
    }
  

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame04";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;  

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
     
    return DF_SUCCESS;

}


/********************************************************************************************************************/
 

int handle_cmd_get_phase_02_repetition_02_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
	
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/

    int phase_buf_size = camera_width_ * camera_height_ * 4;
    float *phase_map_x = new float[phase_buf_size];
    float *phase_map_y = new float[phase_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char *brightness = new unsigned char[brightness_buf_size];

    if (repetition_count < 1)
    {
        repetition_count = 1;
    }

    if (repetition_count > 10)
    {
        repetition_count = 10;
    }

    scan3d_.capturePhase02Repetition02(repetition_count,phase_map_x, phase_map_y,brightness);

 

    LOG(INFO) << "start send depth, buffer_size= " << phase_buf_size;
    ret = send_buffer(client_sock, (const char *)phase_map_x, phase_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] phase_map_x;
        delete[] phase_map_y;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send depth, buffer_size=" << phase_buf_size;
    ret = send_buffer(client_sock, (const char *)phase_map_y, phase_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] phase_map_x;
        delete[] phase_map_y;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    LOG(INFO) << "Send Phase 02";

    float temperature = lc3010.get_projector_temperature();
    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";

        delete[] phase_map_x;
        delete[] phase_map_y;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    delete[] phase_map_x;
    delete[] phase_map_y;
    delete[] brightness;
    return DF_SUCCESS;
}


int handle_cmd_get_frame_06_repetition(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    frame_status_ = DF_FRAME_CAPTURING;
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
        frame_status_ = DF_ERROR_NETWORK;
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    ret = scan3d_.captureFrame06Repetition(repetition_count);
    // ret = scan3d_.captureFrame06Black();
    if (DF_SUCCESS != ret)
    {
    //   LOG(ERROR) << "captureFrame04BaseConfidence code: " << ret;
    //   frame_status_ = ret;
      
        handle_error(ret);
    }
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

        
    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
         scan3d_.copyBrightnessData(brightness);
    }

    // scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map); 
 
    LOG(INFO)<<"capture Frame04 Repetition02 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }


   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06 repetition";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

    

}



int handle_cmd_get_frame_04_repetition_02_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    frame_status_ = DF_FRAME_CAPTURING;
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
        frame_status_ = DF_ERROR_NETWORK;
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    ret = scan3d_.captureFrame04Repetition02BaseConfidence(repetition_count);
    if (DF_SUCCESS != ret)
    {

        
        handle_error(ret);
    //   LOG(ERROR) << "captureFrame04BaseConfidence code: " << ret;
    //   frame_status_ = ret;

    //      switch (ret)
    //      {
    //      case DF_ERROR_LOST_TRIGGER:
    //      {
    //         reboot_lightcraft();
    //      }
    //             break;
    //      case DF_ERROR_CAMERA_STREAM:
    //      {
    //         if(DF_ERROR_2D_CAMERA == scan3d_.reopenCamera())
    //         {
    //             reboot_system();
    //         }
    //      }
    //             break;
    //      default:
    //             break;
    //      }


    }
 
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
         scan3d_.copyBrightnessData(brightness);
    }
              
    scan3d_.copyDepthData(depth_map); 
 
    LOG(INFO)<<"capture Frame04 Repetition02 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }


   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame04";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;
        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }
    return DF_SUCCESS;

    

}



int handle_cmd_get_frame_04_repetition_01_parallel(int client_sock)
{
    /**************************************************************************************/

    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    
    int repetition_count = 1;

    int ret = recv_buffer(client_sock, (char*)(&repetition_count), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO)<<"repetition_count: "<<repetition_count<<"\n";
    /***************************************************************************************/


    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    if(repetition_count< 1)
    {
      repetition_count = 1;
    }
    
    if(repetition_count> 10)
    {
      repetition_count = 10;
    }

    scan3d_.captureFrame04Repetition01(repetition_count);
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();


    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);

    LOG(INFO) << "capture Frame04 Repetition01 Finished!";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral";


    }

   

   /***************************************************************************************************/
    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame04";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        LOG(INFO) <<"send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;  

    return DF_SUCCESS;
 
}

 

int handle_cmd_get_standard_plane_param_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int pointcloud_buf_size = camera_width_*camera_height_*4*3;
    float* pointcloud_map = new float[pointcloud_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 


    scan3d_.captureFrame04BaseConfidence();

    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyPointcloudData(pointcloud_map);
 
    

    int plane_buf_size = 12*4;
    float* plane_param = new float[plane_buf_size];

    memset(plane_param, 0, sizeof(float) * 12);

    float* R = new float[9];
    float* T = new float[3];
    
    memset(R, 0, sizeof(float) * 9);
    memset(T, 0, sizeof(float) * 3);

 

    ConfigureStandardPlane plane_machine;
    plane_machine.setCalibrateParam(param);
    bool found = plane_machine.getStandardPlaneParam(pointcloud_map,brightness,R,T);

    if(found)
    {
        memcpy(plane_param, R, sizeof(float) * 9);
        memcpy(plane_param+9, T, sizeof(float) * 3);
    }
 

    LOG(INFO)<<("start send plane param, buffer_size=%d\n", plane_buf_size);
    int ret = send_buffer(client_sock, (const char*)plane_param, plane_buf_size);
    LOG(INFO)<<("depth ret=%d\n", ret);

    if(ret == DF_FAILED)
    {
        LOG(INFO)<<("send error, close this connection!\n");
	// delete [] buffer;
	delete [] pointcloud_map;
	delete [] brightness;
	
	return DF_FAILED;
    }
     
    LOG(INFO)<<("plane param sent!\n");
    // delete [] buffer;
    delete [] pointcloud_map;
    delete [] brightness;
    return DF_SUCCESS;
     
}



int handle_cmd_get_frame_06_color(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    frame_status_ = DF_FRAME_CAPTURING;
    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 
 

    LOG(INFO)<<"captureFrame06"; 
    ret = scan3d_.captureFrame06Color();
    if(DF_SUCCESS != ret)
    { 
         LOG(ERROR)<<"captureFrame06 code: "<<ret;
        //  frame_status_ = ret;
        
        handle_error(ret);
    }
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

     
    LOG(INFO)<<"Reconstruct Frame06 Finished!";
    // scan3d_.copyBrightnessData(brightness);

        
    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
        scan3d_.copyBrightnessData(brightness);
    }

    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }

    return DF_SUCCESS;
}


int handle_cmd_get_frame_06(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    frame_status_ = DF_FRAME_CAPTURING;
    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 
 

    LOG(INFO)<<"captureFrame06"; 
    ret = scan3d_.captureFrame06();
    if(DF_SUCCESS != ret)
    { 
         LOG(ERROR)<<"captureFrame06 code: "<<ret;
        //  frame_status_ = ret;
        
        handle_error(ret);
    }
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

     
    LOG(INFO)<<"Reconstruct Frame06 Finished!";
    // scan3d_.copyBrightnessData(brightness);

        
    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
        scan3d_.copyBrightnessData(brightness);
    }

    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame06";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }

    return DF_SUCCESS;
}


int handle_cmd_get_frame_04_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    frame_status_ = DF_FRAME_CAPTURING;
    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 
 

    LOG(INFO)<<"captureFrame04"; 
    ret = scan3d_.captureFrame04BaseConfidence();
    if(DF_SUCCESS != ret)
    { 
        
        handle_error(ret);
        //  LOG(ERROR)<<"captureFrame04 code: "<<ret;
        //  frame_status_ = ret;

        //  switch (ret)
        //  {
        //  case DF_ERROR_LOST_TRIGGER:
        //  {
        //     reboot_lightcraft();
        //  }
        //         break;
        //  case DF_ERROR_CAMERA_STREAM:
        //  {
        //     if(DF_ERROR_2D_CAMERA == scan3d_.reopenCamera())
        //     {
        //         reboot_system();
        //     }
        //  }
        //         break;
        //  default:
        //         break;
        //  }
    }
    scan3d_.removeOutlierBaseDepthFilter();
    scan3d_.removeOutlierBaseRadiusFilter();

     
    if(1!=generate_brightness_model)
    {
        scan3d_.captureTextureImage(generate_brightness_model,generate_brightness_exposure_time,brightness);
    }
    else
    { 
         scan3d_.copyBrightnessData(brightness);
    }

    LOG(INFO)<<"Reconstruct Frame04 Finished!";
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame04";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        frame_status_ = DF_ERROR_NETWORK;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    if (DF_FRAME_CAPTURING == frame_status_)
    {
        frame_status_ = DF_SUCCESS;
    }

    return DF_SUCCESS;
}


int handle_cmd_get_frame_05_parallel(int client_sock)
{
   
     if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 
 

    LOG(INFO)<<"captureFrame05";
    scan3d_.captureFrame05();
     
    LOG(INFO)<<"Reconstruct Frame05 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    int ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame04";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;


}


int handle_cmd_get_frame_03_parallel(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 
 

    LOG(INFO)<<"captureFrame03";
    scan3d_.captureFrame03();
     
    LOG(INFO)<<"Reconstruct Frame03 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    int ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame03";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;
    

}
    


int handle_cmd_test_get_frame_01(int client_sock)
{
    
  if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int image_num= 24; 
    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);
 
    int buffer_size = height*width*image_num;
    unsigned char* buffer = new unsigned char[buffer_size];

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 

    LOG(INFO) << "recv raw 01 data:";
    int ret= recv_buffer(client_sock,(char*)buffer,buffer_size);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
 
    LOG(INFO) << "recv raw 01 finished!";
 

    LOG(INFO)<<"testCaptureFrame01";
    scan3d_.testCaptureFrame01(buffer);
     
    LOG(INFO)<<"Reconstruct Frame01 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    // if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    // { 
    //     cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
    //     cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
    //     cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
    //     memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
    //     LOG(INFO) << "Bilateral"; 
    // }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame01";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;
  
}


int handle_cmd_get_frame_01(int client_sock)
{
    
  if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    int depth_buf_size = camera_width_*camera_height_*4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_*camera_height_*1;
    unsigned char* brightness = new unsigned char[brightness_buf_size]; 
 

    LOG(INFO)<<"captureFrame01";
    scan3d_.captureFrame01();
     
    LOG(INFO)<<"Reconstruct Frame01 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);

 
    LOG(INFO)<<"copy depth";  

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }
  

    LOG(INFO) << "start send depth, buffer_size= "<< depth_buf_size;
    int ret = send_buffer(client_sock, (const char *)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= "<<ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= "<<brightness_buf_size;
    ret = send_buffer(client_sock, (const char *)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= "<<ret;

    LOG(INFO) << "Send Frame01";

    float temperature = lc3010.get_projector_temperature();

    LOG(INFO) << "projector temperature: " << temperature << " deg";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    return DF_SUCCESS;
  
}

 
int handle_cmd_get_point_cloud(int client_sock)
{
    lc3010.pattern_mode01();

    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    LOG(INFO) << "captureFrame01";
    scan3d_.captureFrame01();

    int point_cloud_buf_size = camera_width_ * camera_height_ * 3 * 4;
    float *point_cloud_map = new float[point_cloud_buf_size];

    LOG(INFO) << "Reconstruct Frame01 Finished!";
    scan3d_.copyPointcloudData(point_cloud_map);

    LOG(INFO) << ("start send point cloud, buffer_size=%d\n", point_cloud_buf_size);
    int ret = send_buffer(client_sock, (const char *)point_cloud_map, point_cloud_buf_size);
    LOG(INFO) << ("ret=%d\n", ret);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << ("send error, close this connection!\n");
        delete[] point_cloud_map;
        return DF_FAILED;
    }
    LOG(INFO) << ("image sent!\n");
    delete[] point_cloud_map;
    return DF_SUCCESS;
}

int handle_heartbeat(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;	
    }

    mtx_heartbeat_timer.lock();
    heartbeat_timer = 0;
    mtx_heartbeat_timer.unlock();

    return DF_SUCCESS;
}
    
int handle_get_temperature(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
    float temperature = read_temperature(0);
    LOG(INFO) << "CPU temperature:" << temperature;
    int ret = send_buffer(client_sock, (char *)(&temperature), sizeof(temperature));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}
    
int read_calib_param()
{
    std::ifstream ifile;

    ifile.open("calib_param.txt");

    if(!ifile.is_open())
    {
        return DF_FAILED;
    }

    int n_params = sizeof(param)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	ifile>>(((float*)(&param))[i]);
    }
    ifile.close();
    return DF_SUCCESS;
}

int handle_get_camera_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }

    read_calib_param();
	
    int ret = send_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;

}

/*****************************************************************************************/
//system config param 
int handle_get_system_config_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }

    read_calib_param();
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_), sizeof(system_config_settings_machine_.Instance().config_param_));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;

}

bool set_system_config(SystemConfigParam &rect_config_param)
{

    //set led current
    if(rect_config_param.led_current != system_config_settings_machine_.Instance().config_param_.led_current)
    { 
        if(0<= rect_config_param.led_current && rect_config_param.led_current< 1024)
        {
            brightness_current = rect_config_param.led_current;
            if(DF_SUCCESS != lc3010.SetLedCurrent(brightness_current,brightness_current,brightness_current))
            {
                LOG(ERROR)<<"Set Led Current";
            }

            system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;
        }
 
    }

    //set many exposure param
    system_config_settings_machine_.Instance().config_param_.exposure_num = rect_config_param.exposure_num; 
    std::memcpy(system_config_settings_machine_.Instance().config_param_.exposure_param , rect_config_param.exposure_param,sizeof(rect_config_param.exposure_param));
 
 
    //set external param
    
    std::memcpy(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param , rect_config_param.standard_plane_external_param,sizeof(rect_config_param.standard_plane_external_param));

    return true;
}

int handle_set_system_config_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	
     

    SystemConfigParam rect_config_param;


    int ret = recv_buffer(client_sock, (char*)(&rect_config_param), sizeof(rect_config_param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    bool ok = set_system_config(rect_config_param);    

    if(!ok)
    {
        return DF_FAILED;
    }

    return DF_SUCCESS;

}

/**********************************************************************************************************************/
//设置基准平面外参
int handle_cmd_set_param_standard_param_external(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    float plane_param[12]; 

    int ret = recv_buffer(client_sock, (char*)(plane_param), sizeof(float)*12);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


	memcpy(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param, plane_param, sizeof(float)*12);
 
 
    return DF_SUCCESS;
 
}


//获取基准平面外参
int handle_cmd_get_param_standard_param_external(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
   
	
    int ret = send_buffer(client_sock, (char*)(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param), sizeof(float)*12);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

//获取相机增益参数
int handle_cmd_get_param_camera_gain(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    } 
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.camera_gain), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    return DF_SUCCESS;
  
}

//获取相机曝光参数
int handle_cmd_get_param_camera_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    } 
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.camera_exposure_time), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    return DF_SUCCESS;
  
}
 
 
//获取相机像素类型
int handle_cmd_get_camera_pixel_type(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    } 

    XemaPixelType type;
    scan3d_.getCameraPixelType(type);

    int val = (int)type;
	
    int ret = send_buffer(client_sock, (char*)(&val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 

    return DF_SUCCESS;
}

//获取生成亮度参数
int handle_cmd_get_param_brightness_exposure_model(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    } 
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.generate_brightness_exposure_model), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 

    return DF_SUCCESS;
}


//设置亮度图增益参数
int handle_cmd_set_param_brightness_exposure_model(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	   
    int model = 0;

    int ret = recv_buffer(client_sock, (char*)(&model), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    if(1 != model && 2 != model)
    {
        LOG(ERROR)<<"model error: "<<model;
        return DF_FAILED;
    }
 
 
    system_config_settings_machine_.Instance().firwmare_param_.generate_brightness_exposure_model = model;
 
  
    return DF_SUCCESS;
}


//设置亮度图增益参数
int handle_cmd_get_param_brightness_gain(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	   
    float gain = system_config_settings_machine_.Instance().firwmare_param_.brightness_gain;

    int ret = send_buffer(client_sock, (char*)(&gain), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

  
    return DF_SUCCESS;
}

//设置亮度图增益参数
int handle_cmd_set_param_brightness_gain(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	   
    float gain = 0;

    int ret = recv_buffer(client_sock, (char*)(&gain), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    if(gain< 0)
    {
        gain = 0;
    }
    else if(gain > 24)
    {
        gain = 24;
    }
 
 
    system_config_settings_machine_.Instance().firwmare_param_.brightness_gain = gain;
 
  
    return DF_SUCCESS;
}

//设置相机增益参数
int handle_cmd_set_param_camera_gain(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    float gain = 0;

    int ret = recv_buffer(client_sock, (char*)(&gain), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    if(gain< 0)
    {
        gain = 0;
    }
    else if(gain > 24)
    {
        gain = 24;
    }


    if(scan3d_.setParamGain(gain))
    { 
        LOG(INFO) << "Set Camera Gain: " << gain; 
        system_config_settings_machine_.Instance().config_param_.camera_gain = gain;
    }
    else
    {
         LOG(INFO) << "Set Camera Gain Error!"; 
    }

 
  
    return DF_SUCCESS;
}

//设置相机曝光参数
int handle_cmd_set_param_camera_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    float exposure = 0;

    int ret = recv_buffer(client_sock, (char*)(&exposure), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }



    if(scan3d_.setParamExposure(exposure))
    { 
        LOG(INFO) << "Set Camera Exposure Time: " << exposure;

        system_config_settings_machine_.Instance().config_param_.camera_exposure_time = exposure;

    }
    else
    {
         LOG(INFO) << "Set Camera Exposure Time Error!"; 
    }

 
  
    return DF_SUCCESS;
}

//获取生成亮度参数
int handle_cmd_get_param_generate_brightness(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
   
	
    int ret = send_buffer(client_sock, (char*)(&generate_brightness_model), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&generate_brightness_exposure_time), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    return DF_SUCCESS;
}

//设置生成亮度参数
int handle_cmd_set_param_generate_brightness(int client_sock)
{
 if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    int flag = 1 ; 

    int ret = recv_buffer(client_sock, (char*)(&flag), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    float exposure = 0;

    ret = recv_buffer(client_sock, (char*)(&exposure), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

  
    if (scan3d_.setParamGenerateBrightness(flag, exposure))
    {
        generate_brightness_model = flag;
        generate_brightness_exposure_time = exposure;

        LOG(INFO) << "generate_brightness_model: " << generate_brightness_model << "\n";
        LOG(INFO) << "generate_brightness_exposure_time: " << generate_brightness_exposure_time << "\n";
    }

    // camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);

  
    return DF_SUCCESS;
}


//设置反射光滤波参数
int handle_cmd_set_param_reflect_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int switch_val = 0;
 

    int ret = recv_buffer(client_sock, (char*)(&switch_val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 
 

    system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter = switch_val; 

 
    LOG(INFO)<<"use_reflect_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter; 
         

    return DF_SUCCESS;
}


int handle_cmd_get_param_reflect_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
  
    LOG(INFO)<<"use_reflect_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter; 
          
    return DF_SUCCESS;
}


//设置半径滤波参数
int handle_cmd_set_param_radius_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int switch_val = 0;
    float radius = 2;
    int num = 3;

    int ret = recv_buffer(client_sock, (char*)(&switch_val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&radius), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
     
    ret = recv_buffer(client_sock, (char*)(&num), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter = switch_val;
    system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r = radius;
    system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num = num;

 
    LOG(INFO)<<"use_radius_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter;
    LOG(INFO)<<"radius_filter_r: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
    LOG(INFO)<<"radius_filter_threshold_num: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
         

    return DF_SUCCESS;
}

int handle_cmd_get_param_radius_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    
    LOG(INFO)<<"use_radius_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter;
    LOG(INFO)<<"radius_filter_r: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
    LOG(INFO)<<"radius_filter_threshold_num: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
         

    return DF_SUCCESS;
}

int handle_cmd_set_param_depth_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int switch_val = 0;
    float depth_throshold = 2;

    int ret = recv_buffer(client_sock, (char*)(&switch_val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&depth_throshold), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


    system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold = depth_throshold;
    system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter = switch_val;

 
    LOG(INFO)<<"use_depth_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter;
    LOG(INFO)<<"depth_filter_threshold: "<<system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold;
         

    return DF_SUCCESS;
}

int handle_cmd_get_param_depth_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    LOG(INFO)<<"use_depth_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter;
    LOG(INFO)<<"depth_filter_threshold: "<<system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold;
         

    return DF_SUCCESS;
}

int handle_cmd_set_param_gray_rectify(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int switch_val = 0;
    int radius = 9;
    float sigma = 40;

    int ret = recv_buffer(client_sock, (char*)(&switch_val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&radius), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
     
    ret = recv_buffer(client_sock, (char*)(&sigma), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify = switch_val;
    system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r = radius;
    system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma = sigma;

 
    LOG(INFO)<<"use_gray_rectify: "<<system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify;
    LOG(INFO)<<"gray_rectify_r: "<<system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r;
    LOG(INFO)<<"gray_rectify_sigma: "<<system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma;
         

    return DF_SUCCESS;
}

int handle_cmd_get_param_gray_rectify(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma), sizeof(float) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    
    LOG(INFO)<<"use_gray_rectify: "<<system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify;
    LOG(INFO)<<"gray_rectify_r: "<<system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r;
    LOG(INFO)<<"gray_rectify_sigma: "<<system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma;
         

    return DF_SUCCESS;
}

//获取置信度参数
int handle_cmd_get_param_fisher_filter(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
 
    float confidence = system_config_settings_machine_.Instance().firwmare_param_.fisher_confidence;
 
    float offset_val = (confidence + 50)/2;

    int ret = send_buffer(client_sock, (char *)(&offset_val), sizeof(float) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

//噪点过滤参数
int handle_cmd_set_param_fisher_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    float confidence = 0;

    int ret = recv_buffer(client_sock, (char*)&confidence, sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    if(0> confidence || confidence > 100)
    {
        return DF_FAILED;
    }
 
    float offset_val = (confidence*2)-50;
 
    system_config_settings_machine_.Instance().firwmare_param_.fisher_confidence = offset_val;
  
    LOG(INFO)<<"fisher_confidence: "<<system_config_settings_machine_.Instance().firwmare_param_.fisher_confidence;

    scan3d_.setParamFisherConfidence(offset_val);
         

    return DF_SUCCESS;
}


//设置双边滤波参数
int handle_cmd_set_param_bilateral_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int param[2];

    int ret = recv_buffer(client_sock, (char*)(&param[0]), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&param[1]), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter = param[0];


    if (1 == param[0])
    {   
        if (3 == param[1] || 5 == param[1] || 7 == param[1] || 9 == param[1] || 11 == param[1])
        { 
        system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d = param[1];
        }
    }

    
    LOG(INFO)<<"Use Bilateral Filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter;
    LOG(INFO)<<"Bilateral Filter param: "<<system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d;
         

    return DF_SUCCESS;
}

//获取双边滤波参数
int handle_cmd_get_param_bilateral_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 

    return DF_SUCCESS;
}

//获取混合多曝光参数
int handle_cmd_get_param_brightness_hdr_exposure(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int param[11];
    param[0] = system_config_settings_machine_.Instance().firwmare_param_.brightness_hdr_exposure_num;

    memcpy(param + 1, system_config_settings_machine_.Instance().firwmare_param_.brightness_hdr_exposure_param_list, sizeof(int) * 10); 

    int ret = send_buffer(client_sock, (char *)(&param), sizeof(int) * 11);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}


//设置亮度图hdr曝光参数
int handle_cmd_set_param_brightness_hdr_exposure(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	LOG(INFO)<<"check_token finished!";
    int param[11]; 

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(int)*11);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 
    
	LOG(INFO)<<"recv_buffer param finished!";

    int num = param[0];  
      //set led current

    int max_exposure = 1000000;
    int min_exposure = 20;

    for (int i = 0; i < num; i++)
    {
        int exposure = param[1 + i];

        if (exposure > max_exposure)
        {
            exposure = max_exposure;
        }
        else if (exposure < min_exposure)
        {
            exposure = min_exposure;
        }
        param[1 + i] = exposure;
    }

        if(0< num && num<= 10)
        {
 
            system_config_settings_machine_.Instance().firwmare_param_.brightness_hdr_exposure_num = num;
            memcpy(system_config_settings_machine_.Instance().firwmare_param_.brightness_hdr_exposure_param_list, param + 1, sizeof(int) * num); 
            
            for(int i= 0;i< system_config_settings_machine_.Instance().firwmare_param_.brightness_hdr_exposure_num;i++)
            {
	            LOG(INFO)<<"brightness hdr exposure param: "<<i<<" "<<system_config_settings_machine_.Instance().firwmare_param_.brightness_hdr_exposure_param_list[i]; 
            }

            return DF_SUCCESS;
        }
  
        return DF_FAILED;
}


//设置混合多曝光参数
int handle_cmd_set_param_mixed_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	LOG(INFO)<<"check_token finished!";
    int param[13]; 

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(int)*13);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    
	LOG(INFO)<<"recv_buffer param finished!";

    int num = param[0];  
      //set led current

    for (int i = 0; i < num; i++)
    {
        int exposure = param[1 + i];

        if (exposure > max_camera_exposure_)
        {
            exposure = max_camera_exposure_;
        }
        else if (exposure < min_camera_exposure_)
        {
            exposure = min_camera_exposure_;
        }
        param[1 + i] = exposure;
    }

        if(0< num && num<= 6)
        {
 
            system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num = num;
            memcpy(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list, param + 1, sizeof(int) * 6);
            memcpy(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list, param + 7, sizeof(int) * 6);
            system_config_settings_machine_.Instance().firwmare_param_.hdr_model = 2;

            std::vector<int> led_current_list;
            std::vector<int> camera_exposure_list;

            for (int i = 0; i < 6; i++)
            {
                led_current_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list[i]);
                camera_exposure_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list[i]);
            }

            scan3d_.setParamHdr(num, led_current_list, camera_exposure_list);

            return DF_SUCCESS;
        }
  
        return DF_FAILED;
}

//获取相机分辨率
int handle_cmd_get_param_camera_resolution(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width,height);

    // lc3010.read_dmd_device_id(version); 

    int ret = send_buffer(client_sock, (char *)(&width), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char *)(&height), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO)<<"camera width: "<<width;
    LOG(INFO)<<"camera height: "<<height;

    return DF_SUCCESS;

}

//获取相机版本参数
int handle_cmd_get_param_projector_version(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int version = 0;

    scan3d_.getProjectorVersion(version);

    // lc3010.read_dmd_device_id(version); 

    int ret = send_buffer(client_sock, (char *)(&version), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO)<<"camera version: "<<version << "\n";

    return DF_SUCCESS;

}

//获取相机版本参数
int handle_cmd_get_param_camera_version(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int version = 0;

    scan3d_.getProjectorVersion(version);

    lc3010.read_dmd_device_id(version); 

    if(3010 == version)
    {
        version = 800;
    }
    else if(4710 == version)
    { 
        version = 1800;
    }

    int ret = send_buffer(client_sock, (char *)(&version), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO)<<"camera version: "<<version << "\n";

    return DF_SUCCESS;

}


//设置置信度参数
int handle_cmd_set_param_confidence(int client_sock)
{

    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    float val = 0; 
    int ret = recv_buffer(client_sock, (char*)(&val), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO) << "Set Confidence: "<<val;
    system_config_settings_machine_.Instance().firwmare_param_.confidence = val;
    // cuda_set_config(system_config_settings_machine_);

    int rate = 1;
    int bits = 8;

    if(DF_SUCCESS == scan3d_.getPixelFormat(bits))
    {
        switch (bits)
        {
        case 8:
            rate = 1;
            break;

        case 10:
            rate = 4;
            break;

        case 12:
            rate = 16;
            break;

        default:
            break;
        }
    }


    if(!scan3d_.setParamConfidence(val*rate))
    { 
        LOG(INFO)<<"Set Param Confidence Failed!";
    }

    return DF_SUCCESS;
}

//获取置信度参数
int handle_cmd_get_param_confidence(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
 
    float confidence = system_config_settings_machine_.Instance().firwmare_param_.confidence;
 

    int ret = send_buffer(client_sock, (char *)(&confidence), sizeof(float) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

//获取混合多曝光参数
int handle_cmd_get_param_mixed_hdr(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int param[13];
    param[0] = system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num;

    memcpy(param + 1, system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list, sizeof(int) * 6);
    memcpy(param + 7, system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list, sizeof(int) * 6);

    int ret = send_buffer(client_sock, (char *)(&param), sizeof(int) * 13);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

//设置多曝光参数
int handle_cmd_set_param_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    int param[7]; 

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(int)*7);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    int num = param[0];  
      //set led current
    
        if(0< num && num<= 6)
        {
 
            system_config_settings_machine_.Instance().config_param_.exposure_num = num;
            memcpy(system_config_settings_machine_.Instance().config_param_.exposure_param, param+1, sizeof(int) * 6);
            system_config_settings_machine_.Instance().firwmare_param_.hdr_model = 1;
            return DF_SUCCESS;
        }
  
        return DF_FAILED; 
}

//获取多曝光参数
int handle_cmd_get_param_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
  
  		int param[7];
		param[0] = system_config_settings_machine_.Instance().config_param_.exposure_num;

		memcpy(param+1, system_config_settings_machine_.Instance().config_param_.exposure_param, sizeof(int)*6);
	
    int ret = send_buffer(client_sock, (char*)(&param), sizeof(int) * 7);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

//设置采集引擎
int handle_cmd_set_param_capture_engine(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    int engine= -1;

    int ret = recv_buffer(client_sock, (char*)(&engine), sizeof(engine));
    if(ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    // set led current

    if (0 <= engine && engine < 3)
    {
        system_config_settings_machine_.Instance().firwmare_param_.engine = engine;

        XemaPixelType type;
        scan3d_.getCameraPixelType(type);

        if (type == XemaPixelType::Mono)
        {
            if (2 == engine)
            {
                scan3d_.setPixelFormat(12);
            }
            else
            {
                scan3d_.setPixelFormat(8);
            }
        }
        else if (type == XemaPixelType::BayerRG8)
        {
                /*****************************************************************/

                /******************************************************************/
        }

        return DF_SUCCESS;
    }
    else
    {
        LOG(ERROR)<<"engine param error!";
    }

        return DF_FAILED; 
}

//设置光机投影亮度
int handle_cmd_set_param_led_current(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    int led= -1;

    int ret = recv_buffer(client_sock, (char*)(&led), sizeof(led));
    if(ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    // set led current

    if (0 <= led && led < 1024)
    {
        brightness_current = led;

        if (DF_SUCCESS != lc3010.SetLedCurrent(brightness_current, brightness_current, brightness_current))
        {
                LOG(ERROR) << "Set Led Current";

                return DF_FAILED;
        }
        system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;

        scan3d_.setParamLedCurrent(led);
        return DF_SUCCESS;
    }

        return DF_FAILED; 
}


//获取光机投影亮度
int handle_cmd_get_param_led_current(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
  
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.led_current), sizeof(system_config_settings_machine_.Instance().config_param_.led_current));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

//获取相机帧状态
int handle_cmd_get_frame_status(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    LOG(INFO) << "Frame Status: "<<frame_status_;

    int ret = send_buffer(client_sock, (char*)(&frame_status_), sizeof(frame_status_));
    if(ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    return DF_SUCCESS;
}

/*************************************************************************************************************************/

int write_calib_param()
{
    std::ofstream ofile;
    ofile.open("calib_param.txt");
    int n_params = sizeof(param)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	    ofile<<(((float*)(&param))[i])<<std::endl;
    }
    ofile.close();
    return DF_SUCCESS;
}
    
    
int handle_set_camera_looktable(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = -1;

    ret = recv_buffer(client_sock, (char *)(&param), sizeof(param));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO) << "recv param\n";

    int width = 0;
    int height = 0;
    scan3d_.getCameraResolution(width,height);

    /**************************************************************************************/
    cv::Mat xL_rotate_x(height, width, CV_32FC1, cv::Scalar(-2));
    cv::Mat xL_rotate_y(height, width, CV_32FC1, cv::Scalar(-2));
    cv::Mat rectify_R1(3, 3, CV_32FC1, cv::Scalar(-2));
    cv::Mat pattern_mapping(4000, 2000, CV_32FC1, cv::Scalar(-2));
    cv::Mat pattern_minimapping(128, 128, CV_32FC1, cv::Scalar(-2));

    ret = recv_buffer(client_sock, (char *)(xL_rotate_x.data), height * width * sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    LOG(INFO) << "recv xL_rotate_x\n";

    ret = recv_buffer(client_sock, (char *)(xL_rotate_y.data), height * width * sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    LOG(INFO) << "recv xL_rotate_y\n";

    ret = recv_buffer(client_sock, (char *)(rectify_R1.data), 3 * 3 * sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    LOG(INFO) << "recv rectify_R1\n";

    ret = recv_buffer(client_sock, (char *)(pattern_mapping.data), 4000 * 2000 * sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    LOG(INFO) << "recv pattern_mapping\n";

    ret = recv_buffer(client_sock, (char *)(pattern_minimapping.data), 128 * 128 * sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    LOG(INFO) << "recv pattern_minimapping\n";

 

    write_calib_param();

    LookupTableFunction lookup_table_machine_;
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x);
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y);
    lookup_table_machine_.saveBinMappingFloat("./R1.bin", rectify_R1);
    lookup_table_machine_.saveBinMappingFloat("./single_pattern_mapping.bin", pattern_mapping);
    lookup_table_machine_.saveBinMappingFloat("./single_pattern_minimapping.bin",pattern_minimapping);

    LOG(INFO) << "save looktable ";

    if(!scan3d_.loadCalibData())
    {
        LOG(INFO) << "load looktable error!";
    }

    LOG(INFO) << "load looktable";
    /***************************************************************************************/

    return DF_SUCCESS;

}


int handle_set_camera_minilooktable(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    int ret = -1;

    ret = recv_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

     LOG(INFO)<<"recv param\n";
    /**************************************************************************************/
    cv::Mat xL_rotate_x(camera_height_,camera_width_,CV_32FC1,cv::Scalar(-2));
    cv::Mat xL_rotate_y(camera_height_,camera_width_,CV_32FC1,cv::Scalar(-2));
    cv::Mat rectify_R1(3,3,CV_32FC1,cv::Scalar(-2));
    cv::Mat pattern_minimapping(128,128,CV_32FC1,cv::Scalar(-2));

    ret = recv_buffer(client_sock, (char*)(xL_rotate_x.data), camera_height_*camera_width_ *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv xL_rotate_x\n";

     ret = recv_buffer(client_sock, (char*)(xL_rotate_y.data), camera_height_*camera_width_ *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv xL_rotate_y\n";

     ret = recv_buffer(client_sock, (char*)(rectify_R1.data), 3*3 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv rectify_R1\n";

     ret = recv_buffer(client_sock, (char*)(pattern_minimapping.data), 128*128 *sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send minimap error, close this connection!\n";
	    return DF_FAILED;
    }
    LOG(INFO)<<"recv pattern_mini_mapping\n";

  
	    cv::Mat R1_t = rectify_R1.t(); 

    //     LOG(INFO)<<"start copy table:";
    //     reconstruct_copy_minitalbe_to_cuda_memory((float*)pattern_minimapping.data,(float*)xL_rotate_x.data,(float*)xL_rotate_y.data,(float*)R1_t.data);
    //     LOG(INFO)<<"copy finished!";

    //     float b = sqrt(pow(param.translation_matrix[0], 2) + pow(param.translation_matrix[1], 2) + pow(param.translation_matrix[2], 2));
    //     reconstruct_set_baseline(b);
 
    // LOG(INFO)<<"copy looktable\n"; 
    
    write_calib_param();
 
	LookupTableFunction lookup_table_machine_; 
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_x_cam1_iter.bin",xL_rotate_x);
    lookup_table_machine_.saveBinMappingFloat("./combine_xL_rotate_y_cam1_iter.bin",xL_rotate_y);
    lookup_table_machine_.saveBinMappingFloat("./R1.bin",rectify_R1);
    lookup_table_machine_.saveBinMappingFloat("./single_pattern_minimapping.bin",pattern_minimapping);

    LOG(INFO)<<"save minilooktable\n";

    if(!scan3d_.loadCalibData())
    {
        LOG(INFO) << "load looktable error!";
    }

    LOG(INFO) << "load looktable";
    /***************************************************************************************/

    return DF_SUCCESS;

}

int handle_set_camera_parameters(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	return DF_FAILED;
    }
	
    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(param));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    write_calib_param();

    return DF_SUCCESS;

}

/*****************************************************************************************/
bool config_checkerboard(bool enable)
{
    if (enable) {
        lc3010.enable_checkerboard();
    } else {
        lc3010.disable_checkerboard();
        lc3010.init();
    }

    return true;
}
//*****************************************************************************************/
int handle_enable_checkerboard(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"enable checkerboard!";
    config_checkerboard(true);

    float temperature = lc3010.get_projector_temperature();
    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_disable_checkerboard(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"disable checkerboard!";
    config_checkerboard(false);

    float temperature = lc3010.get_projector_temperature();
    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}
/*****************************************************************************************/
bool set_internal_pattern_stop()
{
    bool ack = true;

    lc3010.set_internal_pattern_stop();

    return ack;
}

bool set_flash_data_type()
{
    bool ack = true;

    lc3010.set_flash_data_type();

    return ack;
}

bool set_flash_build_data_size(unsigned int data_size)
{
    bool ack = true;

    ack = lc3010.set_flash_build_data_size(data_size);

    return ack;
}

bool set_erase_flash()
{
    bool ack = true;

    lc3010.set_erase_flash();

    return ack;
}

bool check_erase_flash_status()
{
    bool ack = true;

    ack = lc3010.check_erase_flash_status();

    return ack;
}

bool set_flash_data_length(unsigned short dataLen)
{
    bool ack = true;

    lc3010.set_flash_data_length(dataLen);

    return ack;
}

bool write_internal_pattern_data_into_the_flash(char *WriteData, unsigned int data_size)
{
    bool ack = true;
    int i = 0;
    unsigned int send_package = data_size / WRITE_PACKAGE_SIZE;
    unsigned int send_separately = data_size % WRITE_PACKAGE_SIZE;

 //   char string[50] = {'\0'};
    int wtCnt = 0;
    wtCnt = lc3010.write_data_into_the_flash(Write_Flash_Start, WriteData, WRITE_PACKAGE_SIZE);
 //   sprintf(string, "Write_Flash_Start size: %d", wtCnt);
 //   LOG(INFO)<<string;

    for (i = 1; i < send_package; i++) {
        lc3010.write_data_into_the_flash(Write_Flash_Continue, &WriteData[i*WRITE_PACKAGE_SIZE], WRITE_PACKAGE_SIZE);
    }

    lc3010.set_flash_data_length(send_separately);
    wtCnt = lc3010.write_data_into_the_flash(Write_Flash_Continue, &WriteData[i*WRITE_PACKAGE_SIZE], send_separately);
//    sprintf(string, "Write_Flash_Continue size: %d", wtCnt);
//    LOG(INFO)<<string;
/*
	FILE* fw;
	fw = fopen("pattern_data_1.dat", "wb");
    if (fw != NULL) {
		fwrite(WriteData, 1, data_size, fw);
		fclose(fw);
	}
	else {
        LOG(INFO)<< "save pattern data fail";
	}

    LOG(INFO)<< "data size--" << data_size;
    LOG(INFO)<< "send_package--" << send_package;
    LOG(INFO)<< "send_separately--" << send_separately;
*/
    return ack;
}

bool read_internal_pattern_data_from_the_flash(char *ReadData, unsigned int data_size)
{
    bool ack = true;
    int i = 0;
    unsigned int read_package = data_size / READ_PACKAGE_SIZE;
    unsigned int read_separately = data_size % READ_PACKAGE_SIZE;

    lc3010.read_data_from_the_flash(Read_Flash_Start, ReadData, READ_PACKAGE_SIZE);

    for (i = 1; i < read_package; i++) {
        lc3010.read_data_from_the_flash(Read_Flash_Continue, &ReadData[i*READ_PACKAGE_SIZE], READ_PACKAGE_SIZE);
    }

    lc3010.set_flash_data_length(read_separately);
    lc3010.read_data_from_the_flash(Read_Flash_Continue, &ReadData[i*READ_PACKAGE_SIZE], read_separately);
/*
	FILE* fw;
	fw = fopen("read_pattern_data.dat", "wb");
    if (fw != NULL) {
		fwrite(ReadData, 1, data_size, fw);
		fclose(fw);
	}
	else {
        LOG(INFO)<< "save pattern data fail";
	}
*/
    return ack;
}

bool reload_pattern_order_table_from_flash()
{
    bool ack = true;

    lc3010.reload_pattern_order_table_from_flash();

    return ack;
}
/*****************************************************************************************/
bool load_pattern_process(char *ReadData, unsigned int data_size, char *string)
{
    bool ack = true;

    // step1 -- Set Internal Pattern Stop, Do not repeat (run once)
    if ( !set_internal_pattern_stop() )
    {
        strcpy(string, "step1 -- set_internal_pattern_stop error.");
        return false;
    }

    // step2 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step2 -- set_flash_data_type error.");
        return false;
    }

    // step3 -- set read internal pattern data length, 256 bytes once opation.
    if ( !set_flash_data_length(0x0100) )
    {
        strcpy(string, "step5 -- set_flash_data_length error.");
        return false;
    }

    // step4 -- start to read flash data, 256 bytes once opation. 
    if ( !read_internal_pattern_data_from_the_flash(ReadData, data_size) )
    {
        strcpy(string, "step6 -- read_internal_pattern_data_from_the_flash error.");
        return false;
    }

    return ack;
}

bool program_pattern_process(char *WriteData, char *ReadData, unsigned int data_size, char *string)
{
    bool ack = true;
 
    // step1 -- Set Internal Pattern Stop, Do not repeat (run once)
    if ( !set_internal_pattern_stop() )
    {
        strcpy(string, "step1 -- set_internal_pattern_stop error.");
        return false;
    }

    // step2 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step2 -- set_flash_data_type error.");
        return false;
    }

    // step3 -- set Flash Build Data Size (LSB ~ MSB), return err or not. 
    if ( !set_flash_build_data_size(data_size) )
    {
        strcpy(string, "step3 -- set_flash_build_data_size error.");
        return false;
    }

    // step4 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step4 -- set_flash_data_type error.");
        return false;
    }

    // step5 -- Signature: Value = AAh, BBh, CCh, DDh.
    if ( !set_erase_flash() )
    {
        strcpy(string, "step5 -- set_erase_flash error.");
        return false;
    }

    // step6 -- erase flash status check.
    if ( !check_erase_flash_status() )
    {
        strcpy(string, "step6 -- check_erase_flash_status error.");
        return false;
    }

    // step7 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step7 -- set_flash_data_type error.");
        return false;
    }

    // step8 -- Set  Flash Data Length 256 bytes.
    if ( !set_flash_data_length(0x0100) )
    {
        strcpy(string, "step8 -- set_flash_data_length error.");
        return false;
    }

    // step9 -- write internal pattern data into the flash. Write 256 bytes once operation.
    if ( !write_internal_pattern_data_into_the_flash(WriteData, data_size) )
    {
        strcpy(string, "step9 -- write_internal_pattern_data_into_the_flash error.");
        return false;
    }

    // step10 -- Flash Data Type (D0h for pattern data)
    if ( !set_flash_data_type() )
    {
        strcpy(string, "step2 -- set_flash_data_type error.");
        return false;
    }

    // step11 -- set read internal pattern data length, 256 bytes once opation.
    if ( !set_flash_data_length(0x0100) )
    {
        strcpy(string, "step5 -- set_flash_data_length error.");
        return false;
    }

    // step12 -- start to read flash data, 256 bytes once opation. 
    if ( !read_internal_pattern_data_from_the_flash(ReadData, data_size) )
    {
        strcpy(string, "step6 -- read_internal_pattern_data_from_the_flash error.");
        return false;
    }

    // step13 --Reload from flash
    if ( !reload_pattern_order_table_from_flash() )
    {
        strcpy(string, "step13 -- reload_pattern_order_table_from_flash error.");
        return false;
    }

    return ack;
}
/*****************************************************************************************/
int handle_load_pattern_data(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    unsigned int data_size;
    int ret = recv_buffer(client_sock, (char*)(&data_size), sizeof(data_size));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    char string[100] = {'\0'};
    sprintf(string, "load pattern data size: 0x%X", data_size);
    char *ReadData = new char[data_size];
    memset(ReadData, 0, data_size);
    load_pattern_process(ReadData, data_size, string);

    LOG(INFO)<<string;

    ret = send_buffer(client_sock, ReadData, data_size);
    delete [] ReadData;
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_program_pattern_data(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    unsigned int pattern_size;
    int ret = recv_buffer(client_sock, (char*)(&pattern_size), sizeof(pattern_size));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"recv error, close this connection!\n";
	    return DF_FAILED;
    }

    char string[100] = {'\0'};
    sprintf(string, "program pattern data size: 0x%X", pattern_size);

    char *org_buffer = new char[pattern_size];
    char *back_buffer = new char[pattern_size];
    memset(back_buffer, 0, pattern_size);
/*
	FILE* fr;
    fr = fopen("pattern_data.dat", "rb");
	if (fr != NULL) {
		fread(org_buffer, 1, pattern_size, fr);
		fclose(fr);
	}
	else {
		sprintf(string, "read pattern data fail");
	}
*/
    ret = recv_buffer(client_sock, org_buffer, pattern_size);
    if (ret == DF_FAILED)
    {
        delete [] org_buffer;
        delete [] back_buffer;
        LOG(INFO)<<"recv error, close this connection!\n";
	    return DF_FAILED;
    }

    program_pattern_process(org_buffer, back_buffer, pattern_size, string);
    LOG(INFO)<<string;

    ret = send_buffer(client_sock, back_buffer, pattern_size);
    delete [] org_buffer;
    delete [] back_buffer;
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}
/*****************************************************************************************/
int read_bandwidth()
{
    int val = 0;
    char data[100];

    std::ifstream infile;
    infile.open("/sys/class/net/eth0/speed");
    infile >> data;
    val = (int)std::atoi(data);
    infile.close();

    return val;
}

int handle_get_network_bandwidth(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get network bandwidth!";

    int speed = read_bandwidth();
    int ret = send_buffer(client_sock, (char*)(&speed), sizeof(speed));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}
/*****************************************************************************************/
int handle_get_firmware_version(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get firmware version!";

    char version[_VERSION_LENGTH_] = _VERSION_;
    int ret = send_buffer(client_sock, version, _VERSION_LENGTH_);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

void load_txt(std::string filename, char* info, int length)
{
	FILE* fr = fopen(filename.c_str(), "rb");

	if (fr != NULL) {
		fread(info, 1, length, fr);
		fclose(fr);
	}
	else {
		std::cout << "open file error" << std::endl;
	}
}

int handle_cmd_get_product_info(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED) {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get product info!";

    char *info = new char[INFO_SIZE];
    memset(info, 0, INFO_SIZE);
    load_txt("../product_info.txt", info, INFO_SIZE);
	std::cout << "INFO:\n" << info << std::endl;

    int ret = send_buffer(client_sock, info, INFO_SIZE);    
    delete [] info;
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

bool check_trigger_line()
{
    bool ret = false;
    // char* buffer = new char[camera_width_*camera_height_];

    // lc3010.pattern_mode_brightness();
    // ret = camera.CaptureSelfTest();

    // delete [] buffer;

    return ret;
}

void self_test(char *test_out)
{
    // check the network
    if( read_bandwidth() < 1000) {
        sprintf(test_out, "The network failure -- bandwidth less than 1000Mb.");
        return;
    }

    // check usb camera
    if (!scan3d_.cameraIsValid())
    {
        sprintf(test_out, "The camera failure -- driver installed not porperly.");
        return;
    }

    uint32_t nDeviceNum = 0;
    GX_STATUS status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        sprintf(test_out, "The camera failure -- device not connected.");
        return;
    }

    // check projector i2c
    int version= 0;
    lc3010.read_dmd_device_id(version);
    if ((version != 800) && (version != 1800))
    {
        sprintf(test_out, "The projector failure -- communication error.");
        return;
    }

    // check trigger-line
    if (scan3d_.triggerLineIsValid() == false) {
        sprintf(test_out, "The camera failure -- trigger-line not connected.");
        return;
    }

    sprintf(test_out, "Self-test OK.");
}

int handle_cmd_self_test(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"self test!";

    char test[500] = {'\0'};
    self_test(test);
    int ret = send_buffer(client_sock, test, sizeof(test));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_get_projector_temperature(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get projector temperature!";

    float temperature = lc3010.get_projector_temperature();

    int ret = send_buffer(client_sock, (char*)(&temperature), sizeof(temperature));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

/*****************************************************************************************/
int handle_commands(int client_sock)
{
    int command;
    int ret = recv_command(client_sock, &command); 
    LOG(INFO)<<"command:"<<command;
    
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"connection command not received";
	    close(client_sock);
        return DF_FAILED;
    }

    // set led indicator
	GPIO::output(ACT_PIN, GPIO::HIGH); 

    switch(command)
    {
	case DF_CMD_CONNECT:
	    LOG(INFO)<<"DF_CMD_CONNECT";
	    ret = handle_cmd_connect(client_sock);
	    break;
	case DF_CMD_DISCONNECT:
	    LOG(INFO)<<"DF_CMD_DISCONNECT";
	    ret = handle_cmd_disconnect(client_sock);
	    break;
	case DF_CMD_GET_BRIGHTNESS:
	    LOG(INFO)<<"DF_CMD_GET_BRIGHTNESS";
	    ret = handle_cmd_get_brightness(client_sock);
	    break;
	case DF_CMD_GET_RAW:
	    LOG(INFO)<<"DF_CMD_GET_RAW_01"; 
	    ret = handle_cmd_get_raw_01(client_sock);
	    break;
	case DF_CMD_GET_RAW_TEST:
	    LOG(INFO)<<"DF_CMD_GET_RAW_02"; 
	    ret = handle_cmd_get_raw_02(client_sock);
	    break;
	case DF_CMD_GET_RAW_03:
	    LOG(INFO)<<"DF_CMD_GET_RAW_03"; 
	    ret = handle_cmd_get_raw_03(client_sock);
	    break;
    case DF_CMD_GET_RAW_04:
        LOG(INFO) << "DF_CMD_GET_RAW_04";
        ret = handle_cmd_get_raw_04(client_sock);
        break;
    case DF_CMD_GET_RAW_05:
        LOG(INFO) << "DF_CMD_GET_RAW_05";
        ret = handle_cmd_get_raw_05(client_sock);
        break;
    case DF_CMD_GET_RAW_06:
        LOG(INFO) << "DF_CMD_GET_RAW_06";
        ret = handle_cmd_get_raw_06(client_sock);
        break;
    case DF_CMD_GET_RAW_08:
        LOG(INFO) << "DF_CMD_GET_RAW_08";
        ret = handle_cmd_get_raw_08(client_sock);
	    break;
    case DF_CMD_GET_RAW_04_REPETITION:
	    LOG(INFO)<<"DF_CMD_GET_RAW_04_REPETITION"; 
	    ret = handle_cmd_get_raw_04_repetition(client_sock);
	    break; 
	case DF_CMD_GET_FRAME_01:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_01"; 
	    ret = handle_cmd_get_frame_01(client_sock); 
	    break;
	case DF_CMD_TEST_GET_FRAME_01:
	    LOG(INFO)<<"DF_CMD_TEST_GET_FRAME_01"; 
	    ret = handle_cmd_test_get_frame_01(client_sock); 
	    break;
    case DF_CMD_GET_FRAME_HDR:
        LOG(INFO) << "DF_CMD_GET_FRAME_HDR"; 
        { 
                XemaPixelType type;
                scan3d_.getCameraPixelType(type);

                if (type == XemaPixelType::Mono)
                {
                if (1 == system_config_settings_machine_.Instance().firwmare_param_.hdr_model)
                {

                    std::vector<int> led_list;
                    std::vector<int> exposure_list;

                    for (int i = 0; i < 6; i++)
                    {
                        led_list.push_back(system_config_settings_machine_.Instance().config_param_.exposure_param[i]);
                        exposure_list.push_back(system_config_settings_machine_.Instance().config_param_.camera_exposure_time);
                    }

                    scan3d_.setParamHdr(system_config_settings_machine_.Instance().config_param_.exposure_num, led_list, exposure_list);

                    ret = handle_cmd_get_frame_04_hdr_parallel_mixed_led_and_exposure(client_sock);
                }
                else if (2 == system_config_settings_machine_.Instance().firwmare_param_.hdr_model)
                {
                    std::vector<int> led_current_list;
                    std::vector<int> camera_exposure_list;
                    for (int i = 0; i < 6; i++)
                    {
                        led_current_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list[i]);
                        camera_exposure_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list[i]);
                    }

                    scan3d_.setParamHdr(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num, led_current_list, camera_exposure_list);

                    ret = handle_cmd_get_frame_04_hdr_parallel_mixed_led_and_exposure(client_sock);
                }
                }
                else if (type == XemaPixelType::BayerRG8)
                {
                    ret = handle_cmd_get_frame_06_hdr_color(client_sock);
                }
        }

        break;
 
	case DF_CMD_GET_FRAME_03:
	    LOG(INFO)<<"DF_CMD_GET_FRAME_03";   
    	ret = handle_cmd_get_frame_03_parallel(client_sock); 
	    break;
    case DF_CMD_GET_REPETITION_FRAME_04:
    {

        LOG(INFO) << "DF_CMD_GET_REPETITION_FRAME_04";
        XemaPixelType type;
        scan3d_.getCameraPixelType(type);

        if (type == XemaPixelType::Mono)
        {

                ret = handle_cmd_get_frame_04_repetition_02_parallel(client_sock);
        }
        else if (type == XemaPixelType::BayerRG8)
        {
                ret = handle_cmd_get_frame_06_repetition_color(client_sock);
        }
    }
        break;
    case DF_CMD_GET_FRAME_04:
    {
        LOG(INFO) << "DF_CMD_GET_FRAME_04";
        XemaPixelType type;
        scan3d_.getCameraPixelType(type);

        if (type == XemaPixelType::Mono)
        {
            ret = handle_cmd_get_frame_04_parallel(client_sock);
        }
        else if (type == XemaPixelType::BayerRG8)
        {
            ret = handle_cmd_get_frame_06_color(client_sock);
        }
    }

    break;
    case DF_CMD_GET_FRAME_05:
        LOG(INFO) << "DF_CMD_GET_FRAME_05";
        ret = handle_cmd_get_frame_05_parallel(client_sock);
        break;
    case DF_CMD_GET_FRAME_06_MONO12:
        {
            LOG(INFO) << "DF_CMD_GET_FRAME_06_MONO12";
            XemaPixelType type;
            scan3d_.getCameraPixelType(type);

            if (type == XemaPixelType::Mono)
            {
                ret = handle_cmd_get_frame_06_black(client_sock);
            }
            else if (type == XemaPixelType::BayerRG8)
            {
                ret = handle_cmd_get_frame_06_color(client_sock);
            }
        } 
    break;
        case DF_CMD_GET_FRAME_06_HDR_MONO12:
    { 
        LOG(INFO) << "DF_CMD_GET_FRAME_06_MONO12";

            XemaPixelType type;
            scan3d_.getCameraPixelType(type);

            if (type == XemaPixelType::Mono)
            {
                ret = handle_cmd_get_frame_06_hdr_black(client_sock);
            }
            else if (type == XemaPixelType::BayerRG8)
            {
                ret = handle_cmd_get_frame_06_hdr_color(client_sock);
            }
    }

    break;
    case DF_CMD_GET_REPETITION_FRAME_06_MONO12:
    {
            LOG(INFO) << "DF_CMD_GET_REPETITION_FRAME_06_MONO12";

            XemaPixelType type;
            scan3d_.getCameraPixelType(type);

            if (type == XemaPixelType::Mono)
            { 
                ret = handle_cmd_get_frame_06_repetition_black(client_sock); 
                
            }
            else if (type == XemaPixelType::BayerRG8)
            {
                ret = handle_cmd_get_frame_06_repetition_color(client_sock);
            }
    }

        break;  
    case DF_CMD_GET_FRAME_06:
        {
            LOG(INFO) << "DF_CMD_GET_FRAME_06";
            XemaPixelType type;
            scan3d_.getCameraPixelType(type);

            if (type == XemaPixelType::Mono)
            {
                ret = handle_cmd_get_frame_06(client_sock);
            }
            else if (type == XemaPixelType::BayerRG8)
            {
                ret = handle_cmd_get_frame_06_color(client_sock);
            }
        } 
    break;
    case DF_CMD_GET_FRAME_06_HDR:
    { 
        LOG(INFO) << "DF_CMD_GET_FRAME_06";

            XemaPixelType type;
            scan3d_.getCameraPixelType(type);

            if (type == XemaPixelType::Mono)
            {
                ret = handle_cmd_get_frame_06_hdr(client_sock);
            }
            else if (type == XemaPixelType::BayerRG8)
            {
                ret = handle_cmd_get_frame_06_hdr_color(client_sock);
            }
    }

    break;
    case DF_CMD_GET_REPETITION_FRAME_06:
    {
            LOG(INFO) << "DF_CMD_GET_REPETITION_FRAME_06";

            XemaPixelType type;
            scan3d_.getCameraPixelType(type);

            if (type == XemaPixelType::Mono)
            {
                ret = handle_cmd_get_frame_06_repetition(client_sock);  
            }
            else if (type == XemaPixelType::BayerRG8)
            {
                ret = handle_cmd_get_frame_06_repetition_color(client_sock);
            }
    }

        break;  
	case DF_CMD_GET_POINTCLOUD:
	    LOG(INFO)<<"DF_CMD_GET_POINTCLOUD"; 
	    ret = handle_cmd_get_point_cloud(client_sock);
	    break;
	case DF_CMD_HEARTBEAT:
	    LOG(INFO)<<"DF_CMD_HEARTBEAT";
	    ret = handle_heartbeat(client_sock);
	    break;
	case DF_CMD_GET_TEMPERATURE:
	    LOG(INFO)<<"DF_CMD_GET_TEMPERATURE";
	    ret = handle_get_temperature(client_sock);
	    break;
	case DF_CMD_GET_CAMERA_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_GET_CAMERA_PARAMETERS";
	    ret = handle_get_camera_parameters(client_sock);
	    break;
	case DF_CMD_SET_CAMERA_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_SET_CAMERA_PARAMETERS";
	    ret = handle_set_camera_parameters(client_sock);
        read_calib_param();
	    break;
	case DF_CMD_SET_CAMERA_LOOKTABLE:
	    LOG(INFO)<<"DF_CMD_SET_CAMERA_LOOKTABLE";
	    ret = handle_set_camera_looktable(client_sock);
        read_calib_param();
	    break;
	case DF_CMD_SET_CAMERA_MINILOOKTABLE:
	    LOG(INFO)<<"DF_CMD_SET_CAMERA_MINILOOKTABLE";
	    ret = handle_set_camera_minilooktable(client_sock);
        read_calib_param();
	    break;
	case DF_CMD_ENABLE_CHECKER_BOARD:
	    LOG(INFO)<<"DF_CMD_ENABLE_CHECKER_BOARD";
	    ret = handle_enable_checkerboard(client_sock);
	    break;

	case DF_CMD_DISABLE_CHECKER_BOARD:
	    LOG(INFO)<<"DF_CMD_DISABLE_CHECKER_BOARD";
	    ret = handle_disable_checkerboard(client_sock);
	    break;

    case DF_CMD_LOAD_PATTERN_DATA:
	    LOG(INFO)<<"DF_CMD_LOAD_PATTERN_DATA";
	    ret = handle_load_pattern_data(client_sock);
        break;

    case DF_CMD_PROGRAM_PATTERN_DATA:
	    LOG(INFO)<<"DF_CMD_PROGRAM_PATTERN_DATA";
	    ret = handle_program_pattern_data(client_sock);
        break;

	case DF_CMD_GET_NETWORK_BANDWIDTH:
	    LOG(INFO)<<"DF_CMD_GET_NETWORK_BANDWIDTH";
	    ret = handle_get_network_bandwidth(client_sock);
	    break;

	case DF_CMD_GET_FIRMWARE_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_FIRMWARE_VERSION";
	    ret = handle_get_firmware_version(client_sock);
	    break;

	case DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS";
	    ret = handle_get_system_config_parameters(client_sock);
	    break;
	case DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS";
	    ret = handle_set_system_config_parameters(client_sock);
        saveSystemConfig();
	    break;
	case DF_CMD_GET_STANDARD_PLANE_PARAM:
	    LOG(INFO)<<"DF_CMD_GET_STANDARD_PLANE_PARAM";   
    	ret = handle_cmd_get_standard_plane_param_parallel(client_sock); 
	    break;
	case DF_CMD_GET_PARAM_LED_CURRENT:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_LED_CURRENT";   
    	ret = handle_cmd_get_param_led_current(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_LED_CURRENT:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_LED_CURRENT";   
    	ret = handle_cmd_set_param_led_current(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_HDR:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_HDR";   
    	ret = handle_cmd_get_param_hdr(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_HDR:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_HDR";   
    	ret = handle_cmd_set_param_hdr(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM";   
    	ret = handle_cmd_get_param_standard_param_external(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM";   
    	ret = handle_cmd_set_param_standard_param_external(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS";   
    	ret = handle_cmd_set_param_generate_brightness(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS";   
    	ret = handle_cmd_get_param_generate_brightness(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME";   
    	ret = handle_cmd_set_param_camera_exposure(client_sock);
	    break;
	case DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME";   
    	ret = handle_cmd_get_param_camera_exposure(client_sock);
	    break;
    case DF_CMD_SET_PARAM_CAMERA_GAIN:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_GAIN";   
    	ret = handle_cmd_set_param_camera_gain(client_sock);
	    break; 
	case DF_CMD_GET_PARAM_CAMERA_GAIN:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_GAIN";   
    	ret = handle_cmd_get_param_camera_gain(client_sock);
	    break;
	// case DF_CMD_SET_PARAM_OFFSET:
	//     LOG(INFO)<<"DF_CMD_SET_PARAM_OFFSET";   
    // 	handle_cmd_set_param_offset(client_sock);
	//     break;
	// case DF_CMD_GET_PARAM_OFFSET:
	//     LOG(INFO)<<"DF_CMD_GET_PARAM_OFFSET";   
    // 	handle_cmd_get_param_offset(client_sock);
	//     break;
	case DF_CMD_SET_PARAM_MIXED_HDR:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_MIXED_HDR";   
    	ret = handle_cmd_set_param_mixed_hdr(client_sock);
	    break;
	case DF_CMD_GET_PARAM_MIXED_HDR:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_MIXED_HDR";   
    	ret = handle_cmd_get_param_mixed_hdr(client_sock);
	    break;
    case DF_CMD_GET_PARAM_CAMERA_CONFIDENCE:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_CONFIDENCE";   
    	ret = handle_cmd_get_param_confidence(client_sock); 
	    break;
    case DF_CMD_SET_PARAM_CAMERA_CONFIDENCE:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_CONFIDENCE";   
    	ret = handle_cmd_set_param_confidence(client_sock); 
	    break;
    case DF_CMD_GET_PARAM_FISHER_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_CONFIDENCE";    
        ret = handle_cmd_get_param_fisher_filter(client_sock);
	    break;
    case DF_CMD_SET_PARAM_FISHER_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_FISHER_FILTER";  
        ret = handle_cmd_set_param_fisher_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_CAMERA_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_VERSION";   
    	ret = handle_cmd_get_param_camera_version(client_sock);
	    break;
	case DF_CMD_GET_PARAM_PROJECTOR_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_PROJECTOR_VERSION";   
    	ret = handle_cmd_get_param_projector_version(client_sock);
	    break;
	case DF_CMD_SET_PARAM_REFLECT_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_REFLECT_FILTER";   
    	ret = handle_cmd_set_param_reflect_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_REFLECT_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_REFLECT_FILTER";   
    	ret = handle_cmd_get_param_reflect_filter(client_sock);
	    break;
	case DF_CMD_SET_PARAM_RADIUS_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_RADIUS_FILTER";   
    	ret = handle_cmd_set_param_radius_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_RADIUS_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_RADIUS_FILTER";   
    	ret = handle_cmd_get_param_radius_filter(client_sock);
	    break;
    case DF_CMD_SET_PARAM_DEPTH_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_DEPTH_FILTER";   
    	ret = handle_cmd_set_param_depth_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_DEPTH_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_DEPTH_FILTER";   
    	ret = handle_cmd_get_param_depth_filter(client_sock);
	    break;
    case DF_CMD_SET_PARAM_GRAY_RECTIFY:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_GRAY_RECTIFY";   
    	ret = handle_cmd_set_param_gray_rectify(client_sock);
	    break;
	case DF_CMD_GET_PARAM_GRAY_RECTIFY:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_GRAY_RECTIFY";   
    	ret = handle_cmd_get_param_gray_rectify(client_sock);
	    break;
	case DF_CMD_SET_PARAM_BILATERAL_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_BILATERAL_FILTER";   
    	handle_cmd_set_param_bilateral_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_BILATERAL_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_BILATERAL_FILTER";   
    	ret = handle_cmd_get_param_bilateral_filter(client_sock);
	    break;
	case DF_CMD_SET_AUTO_EXPOSURE_BASE_ROI:
	    LOG(INFO)<<"DF_CMD_SET_AUTO_EXPOSURE_BASE_ROI";   
    	ret = handle_cmd_set_auto_exposure_base_roi_half(client_sock);
	    break;
	case DF_CMD_SET_AUTO_EXPOSURE_BASE_BOARD:
	    LOG(INFO)<<"DF_CMD_SET_AUTO_EXPOSURE_BASE_BOARD";   
    	ret = handle_cmd_set_auto_exposure_base_board(client_sock);
	case DF_CMD_SELF_TEST:
	    LOG(INFO)<<"DF_CMD_SELF_TEST";   
    	ret = handle_cmd_self_test(client_sock);
	    break;
	case DF_CMD_GET_PROJECTOR_TEMPERATURE:
	    LOG(INFO)<<"DF_CMD_GET_PROJECTOR_TEMPERATURE";
	    ret = handle_get_projector_temperature(client_sock);
	    break;
    case DF_CMD_GET_PHASE_02_REPETITION:
    	LOG(INFO)<<"DF_CMD_GET_PHASE_02_REPETITION";
	    ret = handle_cmd_get_phase_02_repetition_02_parallel(client_sock);
	    break;
    case DF_CMD_GET_FOCUSING_IMAGE:
        LOG(INFO)<<"DF_CMD_CONFIGURE_FOCUSING"; 
        ret = handle_cmd_get_focusing_image(client_sock);
        break;
    case DF_CMD_GET_CAMERA_RESOLUTION:
        LOG(INFO)<<"DF_CMD_GET_CAMERA_RESOLUTION"; 
        ret = handle_cmd_get_param_camera_resolution(client_sock);
        break;
    case DF_CMD_SET_INSPECT_MODEL_FIND_BOARD:
        LOG(INFO)<<"DF_CMD_SET_INSPECT_MODEL_FIND_BOARD"; 
        ret = handle_cmd_set_board_inspect(client_sock);
        break;
    case DF_CMD_GET_PRODUCT_INFO:
        LOG(INFO)<<"DF_CMD_GET_PRODUCT_INFO"; 
        ret = handle_cmd_get_product_info(client_sock);
        break;

    case DF_CMD_GET_FRAME_STATUS:
        LOG(INFO)<<"DF_CMD_GET_FRAME_STATUS"; 
        ret = handle_cmd_get_frame_status(client_sock);
        break;
     case DF_CMD_GET_PARAM_BRIGHTNESS_HDR_EXPOSURE:
        LOG(INFO)<<"DF_CMD_GET_PARAM_BRIGHTNESS_HDR_EXPOSURE"; 
        ret = handle_cmd_get_param_brightness_hdr_exposure(client_sock);
        break;
    case DF_CMD_SET_PARAM_BRIGHTNESS_HDR_EXPOSURE:
        LOG(INFO)<<"DF_CMD_SET_PARAM_BRIGHTNESS_HDR_EXPOSURE"; 
        ret = handle_cmd_set_param_brightness_hdr_exposure(client_sock);
        break;
    case DF_CMD_GET_PARAM_BRIGHTNESS_GAIN:
        LOG(INFO)<<"DF_CMD_GET_PARAM_BRIGHTNESS_GAIN"; 
        ret = handle_cmd_get_param_brightness_gain(client_sock);
        break;
    case DF_CMD_SET_PARAM_BRIGHTNESS_GAIN:
        LOG(INFO)<<"DF_CMD_SET_PARAM_BRIGHTNESS_GAIN"; 
        ret = handle_cmd_set_param_brightness_gain(client_sock);
        break;
    case DF_CMD_SET_PARAM_BRIGHTNESS_EXPOSURE_MODEL:
        LOG(INFO)<<"DF_CMD_SET_PARAM_BRIGHTNESS_EXPOSURE_MODEL"; 
        ret = handle_cmd_set_param_brightness_exposure_model(client_sock);
        break;
    case DF_CMD_GET_PARAM_BRIGHTNESS_EXPOSURE_MODEL:
        LOG(INFO)<<"DF_CMD_GET_PARAM_BRIGHTNESS_EXPOSURE_MODEL"; 
        ret = handle_cmd_get_param_brightness_exposure_model(client_sock);
        break;
    case DF_CMD_GET_CAMERA_PIXEL_TYPE:
        LOG(INFO)<<"DF_CMD_GET_CAMERA_PIXEL_TYPE"; 
        ret = handle_cmd_get_camera_pixel_type(client_sock);
        break;
    case DF_CMD_SET_PARAM_CAPTURE_ENGINE:
        LOG(INFO)<<"DF_CMD_SET_PARAM_CAPTURE_ENGINE"; 
        ret = handle_cmd_set_param_capture_engine(client_sock);
        break;
	default:
	    LOG(INFO)<<"DF_CMD_UNKNOWN";
        ret = handle_cmd_unknown(client_sock);
	    break;
    }

    // close led indicator
	GPIO::output(ACT_PIN, GPIO::LOW); 

    close(client_sock);
    
	LOG(INFO)<<"handle_commands ret: "<<ret;
    return DF_SUCCESS;
}

int init()
{  
    // init led indicator
	GPIO::setmode(GPIO::BCM);                           // BCM mode
	GPIO::setup(INPUT_PIN, GPIO::IN);                   // inutput pin, set to HIGH level
	GPIO::setup(OUTPUT1_PIN, GPIO::OUT, GPIO::LOW);     // output pin, set to LOW level
 	GPIO::setup(ACT_PIN, GPIO::OUT, GPIO::LOW);         // output pin, set to LOW level
	GPIO::setup(OUTPUT2_PIN, GPIO::OUT, GPIO::LOW);     // output pin, set to LOW level
	GPIO::setup(LED_CTL_PIN, GPIO::OUT, GPIO::LOW);     // output pin, set to LOW level

    int ret = DF_SUCCESS;

    ret = scan3d_.init();

    if(DF_SUCCESS != ret)
    { 
        LOG(INFO)<<"init Failed!";
    }

  
    scan3d_.getCameraResolution(camera_width_,camera_height_);
    //set default param 
    if(!scan3d_.setParamConfidence(system_config_settings_machine_.Instance().firwmare_param_.confidence))
    { 
        LOG(INFO)<<"Set Param Confidence Failed!";
    }

    if(!scan3d_.setParamExposure(system_config_settings_machine_.Instance().config_param_.camera_exposure_time))
    { 
        LOG(INFO)<<"Set Param Exposure Failed!";
    }

    if(!scan3d_.setParamGain(system_config_settings_machine_.Instance().config_param_.camera_gain))
    { 
        LOG(INFO)<<"Set Param Gain Failed!";
    }

    scan3d_.setParamSystemConfig(system_config_settings_machine_);

    int version= 0;
    lc3010.read_dmd_device_id(version);
    LOG(INFO)<<"read camera version: "<<version;

    // cuda_set_config(system_config_settings_machine_);

    set_projector_version(version);
    // LOG(INFO)<<"camera version: "<<DFX_800;

    return ret;
}
  
void rolloutHandler(const char* filename, std::size_t size)
{ 
	/// 备份日志
    if (access("xemaLog", F_OK) != 0)
    {
        system("mkdir xemaLog");
    }

    std::vector<std::string> name_list;
    std::string suffix = "log";
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir("xemaLog")) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            /* print all the files and directories within directory */
            // printf("%s\n", ent->d_name);

            std::string name = ent->d_name;

            if(name.size()< 3)
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
	if (num < 10)
	{
		num++;
	}
	else
	{
		num = 10;
		name_list.pop_back();
	}
 

	for (int i = num; i > 0 && !name_list.empty(); i--)
	{
		std::stringstream ss;
		std::string path = "./xemaLog/" + name_list.back();
		name_list.pop_back();
		ss << "mv " << path << " xemaLog/log_" << i-1 << ".log";
		std::cout << ss.str() << std::endl;
		system(ss.str().c_str());
	}

	std::stringstream ss;
	ss << "mv " << filename << " xemaLog/log_0" <<".log";
	system(ss.str().c_str());
 

}
 


int main()
{

    	//关闭log输出
	el::Configurations conf;
	conf.setToDefault();  
	conf.setGlobally(el::ConfigurationType::Filename, "xema_log.log"); 
	conf.setGlobally(el::ConfigurationType::Enabled, "true");
	conf.setGlobally(el::ConfigurationType::ToFile, "true"); 
	el::Loggers::reconfigureAllLoggers(conf); 
    //log configure
	el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::MaxLogFileSize, "10485760");//10MB 10485760 
	/// 注册回调函数
	el::Helpers::installPreRollOutCallback(rolloutHandler);

    /***************************************************************************/

    LOG(INFO)<<"server started";
    int ret = init();
    if(DF_SUCCESS!= ret)
    {
        
        lc3010.disable_solid_field();
        LOG(INFO)<<"init FAILED";
        // return -1;
    }


    if(DF_ERROR_2D_CAMERA == ret)
    {
 
        LOG(ERROR) << "Open Camera Error!"; 
    /************************************************************************************/
    //相机打开失败，闪烁6秒

        int light_num = 6;

        while (light_num-- > 0)
        {
           lc3010.enable_solid_field();

           usleep(500000);
           lc3010.disable_solid_field(); 
           usleep(500000);
        }

        LOG(ERROR) << "reboot!"; 
        pwrctrl.off_board();
    /*************************************************************************************/
    }

    LOG(INFO)<<"inited";
    int sec = 0;



    int server_sock;
    do
    {
        server_sock = setup_socket(DF_PORT);
        sleep(1);
        sec++;
        if (sec > 30)
        { 
            lc3010.disable_solid_field();
        }
    } while (server_sock == DF_FAILED);

    LOG(INFO)<<"listening"; 
    
    lc3010.disable_solid_field();
 

    while(true)
    {
        int client_sock = accept_new_connection(server_sock);
        if(client_sock!=-1)
	    {
            handle_commands(client_sock);
        }
    }

    close(server_sock);
	GPIO::cleanup();

    return 0;
}
