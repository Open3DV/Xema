#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class DF_Encode
{
public:
	DF_Encode();
	~DF_Encode();

	bool sixStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence);

	bool fourStepPhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& mask, cv::Mat& confidence);

	bool computePhaseBaseSixStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence);

	bool computePhaseBaseFourStep(std::vector<cv::Mat> patterns, std::vector<cv::Mat>& wrap_maps, cv::Mat& mask_img, cv::Mat& confidence);

	void unwarpDualWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, cv::Mat& h_unwrap, cv::Mat& k_Mat);

	bool unwrapVariableWavelength(cv::Mat l_unwrap, cv::Mat h_wrap, double rate, cv::Mat& h_unwrap, cv::Mat& k_Mat, float threshold, cv::Mat& err_mat);

	bool unwrapVariableWavelengthPatterns(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool unwrapVariableWavelengthPatternsBaseConfidence(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool unwrapVariableWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, std::vector<double> rate_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool unwrapHalfWavelengthPatternsOpenmp(std::vector<cv::Mat> wrap_img_list, cv::Mat& unwrap_img, cv::Mat& mask);

	bool selectMaskBaseConfidence(cv::Mat confidence, int threshold, cv::Mat& mask);

	bool maskMap(cv::Mat mask, cv::Mat& map);

	bool mergePatterns(std::vector<std::vector<cv::Mat>> patterns_list, std::vector<cv::Mat>& patterns);

	bool decodeGrayCode(std::vector<cv::Mat> patterns, cv::Mat average_brightness, cv::Mat& k1_map, cv::Mat& k2_map);

	bool grayCodeToBinCode(std::vector<bool> gray_code, std::vector<bool>& bin_code); 

	bool grayCodeToXorCode(std::vector<cv::Mat> gray_code, std::vector<cv::Mat>& xor_code);

	bool computeXOR05(std::vector<cv::Mat> patterns, cv::Mat& k1_map, cv::Mat& k2_map, cv::Mat& mask_map);

	bool minsw10CodeToValue(int minsw, int& value);

	bool minsw8CodeToValue(int minsw, int& value);

	bool decodeMinswGrayCode(std::vector<cv::Mat> patterns, cv::Mat average_brightness, cv::Mat& k_map);

	bool decodeMinswGrayCode(std::vector<cv::Mat> patterns, std::vector<cv::Mat> threshold_list, cv::Mat& k_map);

	bool decodeMinswGrayCode(std::vector<cv::Mat> patterns, std::vector<cv::Mat> patterns_inv,
		cv::Mat direct,cv::Mat global,cv::Mat& uncertain, cv::Mat& k_map);

	bool decodeMinswGrayCode(std::vector<cv::Mat> patterns, cv::Mat threshold, cv::Mat direct, cv::Mat global, cv::Mat& uncertain, cv::Mat& k_map);

	bool computePhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& confidence_map, cv::Mat& average_map, cv::Mat& brightness_map, cv::Mat& mask_map);
	
	bool computePhaseShift(std::vector<cv::Mat> patterns, cv::Mat& wrap_map, cv::Mat& confidence_map,cv::Mat& global_map, 
		cv::Mat& average_map, cv::Mat& brightness_map, cv::Mat& mask_map);

	bool unwrapBase2Kmap(cv::Mat wrap_map, cv::Mat k1_map, cv::Mat k2_map, cv::Mat& unwrap_map);


};

