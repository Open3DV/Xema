#include "system_config_settings.h" 
#include <string>
#include "istream"
#include <fstream> 
#include <vector>
#include <sstream>

SystemConfigDataStruct SystemConfigDataStruct::instance_;
SystemConfigDataStruct::SystemConfigDataStruct()
{
	instance_.config_param_.led_current = 1023;
	instance_.config_param_.exposure_num = 2;

	for (int i = 0; i < 6; i++)
	{
		instance_.config_param_.exposure_param[i] = (i + 1) * 100;
	}


	instance_.config_param_.camera_exposure_time = 24000;
	instance_.config_param_.camera_gain = 0;
	instance_.config_param_.external_param_flag = 0;

	instance_.config_param_.standard_plane_external_param[0] = 1;
	instance_.config_param_.standard_plane_external_param[1] = 0;
	instance_.config_param_.standard_plane_external_param[2] = 0;

	instance_.config_param_.standard_plane_external_param[3] = 0;
	instance_.config_param_.standard_plane_external_param[4] = 1;
	instance_.config_param_.standard_plane_external_param[5] = 0;

	instance_.config_param_.standard_plane_external_param[6] = 0;
	instance_.config_param_.standard_plane_external_param[7] = 0;
	instance_.config_param_.standard_plane_external_param[8] = 1;

	instance_.config_param_.standard_plane_external_param[9] = 0;
	instance_.config_param_.standard_plane_external_param[10] = 0;
	instance_.config_param_.standard_plane_external_param[11] = 0;

	instance_.config_param_.standard_plane[0] = 0;
	instance_.config_param_.standard_plane[1] = 0;
	instance_.config_param_.standard_plane[2] = 1;
	instance_.config_param_.standard_plane[3] = 0;

	instance_.config_param_.camera_exposure_time = 24000;
	instance_.config_param_.camera_gain = 0;


	instance_.firwmare_param_.generate_brightness_exposure = 12000;
	instance_.firwmare_param_.generate_brightness_model = 1;
	instance_.firwmare_param_.generate_brightness_exposure_model = 1;
	instance_.firwmare_param_.mixed_exposure_num = 2;
	instance_.firwmare_param_.hdr_model = 1;

	instance_.firwmare_param_.mixed_led_param_list[0] = 1023;
	instance_.firwmare_param_.mixed_led_param_list[1] = 1023;
	instance_.firwmare_param_.mixed_led_param_list[2] = 1023;
	instance_.firwmare_param_.mixed_led_param_list[3] = 1023;
	instance_.firwmare_param_.mixed_led_param_list[4] = 1023;
	instance_.firwmare_param_.mixed_led_param_list[5] = 1023;

	instance_.firwmare_param_.mixed_exposure_param_list[0] = 12000;
	instance_.firwmare_param_.mixed_exposure_param_list[1] = 36000;
	instance_.firwmare_param_.mixed_exposure_param_list[2] = 24000;
	instance_.firwmare_param_.mixed_exposure_param_list[3] = 36000;
	instance_.firwmare_param_.mixed_exposure_param_list[4] = 48000;
	instance_.firwmare_param_.mixed_exposure_param_list[5] = 60000;

	instance_.firwmare_param_.use_bilateral_filter = 1;
	instance_.firwmare_param_.bilateral_filter_param_d = 7;

	instance_.firwmare_param_.confidence = 5;

	instance_.firwmare_param_.use_radius_filter = 1; 
	instance_.firwmare_param_.radius_filter_r = 2.5; 
	instance_.firwmare_param_.radius_filter_threshold_num = 8; 
	instance_.firwmare_param_.use_depth_filter = 0; 
	instance_.firwmare_param_.depth_filter_threshold = 33; 
	instance_.firwmare_param_.use_reflect_filter = 0;
	instance_.firwmare_param_.fisher_confidence = -50;
	instance_.firwmare_param_.use_gray_rectify = 0; 
	instance_.firwmare_param_.gray_rectify_r = 5; 
	instance_.firwmare_param_.gray_rectify_sigma = 40; 

	for (int i = 0; i < 10; i++)
	{
		instance_.firwmare_param_.brightness_hdr_exposure_param_list[i] = 10000*(i+1);
		instance_.firwmare_param_.brightness_hdr_gain_param_list[i] = 0;
	}
	instance_.firwmare_param_.brightness_hdr_exposure_num = 2;

	instance_.firwmare_param_.brightness_gain = 0;

	instance_.firwmare_param_.engine = 1;

	instance_.firwmare_param_.use_global_light_filter = 0;
	instance_.firwmare_param_.global_light_filter_b =75;
	instance_.firwmare_param_.global_light_filter_threshold = 25;
}

bool SystemConfigDataStruct::loadFromSettings(const std::string& f)
{

	std::ifstream ifile;
	ifile.open(f, std::ios::in);
	if (!ifile)
	{
		std::cerr << "Open Settings File Fail." << std::endl;
		return false;
	}

	std::vector<std::string> str_list;
	std::string line;

	while (getline(ifile, line)) // line中不包括每行的换行符
	{
		std::cout << line << std::endl;
		str_list.push_back(line);
	}
	ifile.close();

	for (int i = 0; i < str_list.size(); i++)
	{
		// std::cout << str_list[i] << std::endl;

		std::vector<std::string> param_list = vStringSplit(str_list[i], ":");

		if (2 == param_list.size())
		{

			param_list[1] = StringTrim(param_list[1]);


			if ("led_current" == param_list[0])
			{
				instance_.config_param_.led_current = atoi(param_list[1].c_str());
			}
			else if ("exposure_num" == param_list[0])
			{
				instance_.config_param_.exposure_num = atoi(param_list[1].c_str());
			}
			else if ("exposure_param" == param_list[0])
			{
				std::vector<std::string> led_param_list = vStringSplit(param_list[1], ",");

				if (6 == led_param_list.size())
				{
					for (int i = 0; i < 6; i++)
					{
						instance_.config_param_.exposure_param[i] = atoi(led_param_list[i].c_str());
					}
				}
			}
			else if ("camera_exposure_time" == param_list[0])
			{
				instance_.config_param_.camera_exposure_time = atof(param_list[1].c_str());
			}
			else if ("camera_gain" == param_list[0])
			{
				instance_.config_param_.camera_gain = atof(param_list[1].c_str());
			}
			else if ("external_param_flag" == param_list[0])
			{
				instance_.config_param_.external_param_flag = atoi(param_list[1].c_str());
			}
			else if ("external_param" == param_list[0])
			{
				std::vector<std::string> external_param_list = vStringSplit(param_list[1], ",");

				if (12 == external_param_list.size())
				{
					for (int i = 0; i < 12; i++)
					{
						instance_.config_param_.standard_plane_external_param[i] = atof(external_param_list[i].c_str());
					}
				}
			}
			else if ("standard_plane" == param_list[0])
			{
				std::vector<std::string> plane_param_list = vStringSplit(param_list[1], ",");

				if (4 == plane_param_list.size())
				{
					for (int i = 0; i < 4; i++)
					{
						instance_.config_param_.standard_plane[i] = atof(plane_param_list[i].c_str());
					}
				}
			}

		}
	}


	return true;

}

bool SystemConfigDataStruct::saveToSettings(const std::string& f)
{

	std::ofstream ofile;
	ofile.open(f, std::ios::out);
	if (!ofile)
	{
		std::cerr << "Open File Fail." << std::endl;
		return false;
	}

	ofile << "led_current: " << instance_.config_param_.led_current << std::endl;
	ofile << "exposure_num: " << instance_.config_param_.exposure_num << std::endl;


	std::string exposure_param_str = "";

	for (int i = 0; i < 5; i++)
	{
		exposure_param_str += std::to_string(instance_.config_param_.exposure_param[i]);
		exposure_param_str += ",";
	}
	exposure_param_str += std::to_string(instance_.config_param_.exposure_param[5]);
	ofile << "exposure_param: " << exposure_param_str << std::endl;

	//
	ofile << "camera_exposure_time: " << instance_.config_param_.camera_exposure_time << std::endl;
	ofile << "camera_gain: " << instance_.config_param_.camera_gain << std::endl;
	ofile << "external_param_flag: " << instance_.config_param_.external_param_flag << std::endl;

	//
	std::string external_param_str = "";
	for (int i = 0; i < 11; i++)
	{
		external_param_str += std::to_string(instance_.config_param_.standard_plane_external_param[i]);
		external_param_str += ",";
	}

	external_param_str += std::to_string(instance_.config_param_.standard_plane_external_param[11]);

	ofile << "external_param: " << external_param_str << std::endl;

	//
	std::string plane_param_str = "";
	for (int i = 0; i < 3; i++)
	{
		plane_param_str += std::to_string(instance_.config_param_.standard_plane[i]);
		plane_param_str += ",";
	}
	plane_param_str += std::to_string(instance_.config_param_.standard_plane[3]);
	ofile << "standard_plane: " << plane_param_str << std::endl;


	ofile.close();

	return true;
}


/***************************************************************************************************************************/


std::string& StringTrim(std::string& str)
{
	if (str.empty()) {
		return str;
	}
	str.erase(0, str.find_first_not_of(" "));
	str.erase(str.find_last_not_of(" ") + 1);
	return str;
}

std::vector<std::string> vStringSplit(const std::string& s, const std::string& delim)
{
	std::vector<std::string> elems;
	size_t pos = 0;
	size_t len = s.length();
	size_t delim_len = delim.length();
	if (delim_len == 0)
		return elems;
	while (pos < len)
	{
		int find_pos = s.find(delim, pos);
		if (find_pos < 0)
		{
			elems.push_back(s.substr(pos, len - pos));
			break;
		}
		elems.push_back(s.substr(pos, find_pos - pos));
		pos = find_pos + delim_len;
	}
	return elems;
}
