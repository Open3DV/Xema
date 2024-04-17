#include "basic_function.h"
#include "../sdk/camera_status.h"
#include <map>
#include <iostream> 
#include "easylogging++.h"


std::map<std::string, std::string> map_version_;


void initVersion();

  

int inquireVersion(std::string git_version, std::string& version)
{

	LOG(INFO) << "git version: " << git_version << std::endl;
	std::vector<std::string> tokens;
	std::stringstream ss(git_version);
	std::string token;

	while (std::getline(ss, token, ' ')) {
		tokens.push_back(token);
	}

	int position = -1;
	auto it = std::find(tokens.begin(), tokens.end(), "commit:");
	if (it != tokens.end())
	{
		position = it - tokens.begin();
		position += 1;
	}


	if (-1 != position && position < tokens.size())
	{
		//LOG(INFO) << "Time: " << tokens[position] << std::endl; 
		initVersion();

		std::map<std::string, std::string>::iterator iter;

		iter = map_version_.find(tokens[position]);

		if (map_version_.end() != iter)
		{
			version = iter->second;
		}
		else
		{
			std::string version_str = "";
			for (const auto& map : map_version_) {
				std::cout << "Key: " << map.first << ", Value: " << map.second << std::endl;
				std::cout << "compare: " << (tokens[position].compare(map.first)) << std::endl;

				if (0 > tokens[position].compare(map.first))
				{
					break;
				}
				version_str = map.second;
			}
			version = version_str;
		}
	}
	else
	{
		return DF_ERROR_INVALID_VERSION;
	}



	return DF_SUCCESS;
}


void initVersion()
{
  
	map_version_.clear();
	map_version_.insert(std::pair<std::string, std::string>("2022-02-10", "v1.0.0"));
	map_version_.insert(std::pair<std::string, std::string>("2022-02-22", "v1.0.1"));
	map_version_.insert(std::pair<std::string, std::string>("2022-03-18", "v1.0.2"));
	map_version_.insert(std::pair<std::string, std::string>("2022-05-16", "v1.0.3"));
	map_version_.insert(std::pair<std::string, std::string>("2022-06-20", "v1.0.4"));
	map_version_.insert(std::pair<std::string, std::string>("2022-08-05", "v1.0.5"));
	map_version_.insert(std::pair<std::string, std::string>("2022-09-09", "v1.0.6"));
	map_version_.insert(std::pair<std::string, std::string>("2022-10-17", "v1.0.6.1"));

	map_version_.insert(std::pair<std::string, std::string>("2022-12-08", "v1.0.7"));
	map_version_.insert(std::pair<std::string, std::string>("2023-02-17", "v1.1.0"));
	map_version_.insert(std::pair<std::string, std::string>("2023-03-09", "v1.1.1"));
	map_version_.insert(std::pair<std::string, std::string>("2023-05-12", "v1.2.0"));
	map_version_.insert(std::pair<std::string, std::string>("2023-06-15", "v1.3.0"));
	map_version_.insert(std::pair<std::string, std::string>("2023-07-06", "v1.3.1"));
	map_version_.insert(std::pair<std::string, std::string>("2023-08-03", "v1.4.0"));
	map_version_.insert(std::pair<std::string, std::string>("2023-08-16", "v1.4.1"));
	map_version_.insert(std::pair<std::string, std::string>("2023-09-05", "v1.5.0"));
	map_version_.insert(std::pair<std::string, std::string>("2023-09-21", "v1.5.1"));
	map_version_.insert(std::pair<std::string, std::string>("2023-12-22", "v1.5.2"));
	map_version_.insert(std::pair<std::string, std::string>("2024-04-17", "v1.5.3"));
}
