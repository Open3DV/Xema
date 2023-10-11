#include "LookupTableFunction.h"
#include "iostream"
#include <fstream>
#include "../firmware/protocol.h" 
//#include "FileIoFunction.h" 

LookupTableFunction::LookupTableFunction()
{
	image_size_.width = 1920;
	image_size_.height = 1200;

	min_low_z_ = 10;
	max_max_z_ = 30000;

	dlp_width_ = 1280;
	dlp_height_ = 720;
}


LookupTableFunction::~LookupTableFunction()
{
}

void LookupTableFunction::setCameraResolution(int width, int height)
{
	image_size_.width = width;
	image_size_.height = height;
}

void LookupTableFunction::setImageResolution(int width, int height)
{
	image_size_.width = width;
	image_size_.height = height;
}


bool LookupTableFunction::setProjectorVersion(int version)
{

	switch (version)
	{
	case DF_PROJECTOR_3010:
	{
		dlp_width_ = 1280;
		dlp_height_ = 720; 
		return true;
	}
	break;

	case DF_PROJECTOR_4710:
	{ 
		dlp_width_ = 1920;
		dlp_height_ = 1080; 

		return true;
	}
	break;

	default:
		break;
	}

	return false;

}

// bool LookupTableFunction::setCameraVersion(int version)
// {

// 	switch (version)
// 	{
// 	case DFX_800:
// 	{
// 		dlp_width_ = 1280;
// 		dlp_height_ = 720;

// 		min_low_z_ = 100;
// 		max_max_z_ = 5000;

// 		return true;
// 	}
// 	break;

// 	case DFX_1800:
// 	{

// 		dlp_width_ = 1920;
// 		dlp_height_ = 1080;

// 		min_low_z_ = 300;
// 		max_max_z_ = 10000;

// 		return true;
// 	}
// 	break;

// 	default:
// 		break;
// 	}

// 	return false;


// }

/*******************************************************************************************************/

bool LookupTableFunction::undistortedPoints(std::vector<cv::Point2d> distortPoints, cv::Mat intrinsic, cv::Mat distortion, std::vector<cv::Point2d>& undisted_points)
{

	if (!intrinsic.data)
	{
		return false;
	}

	if (!distortion.data)
	{
		return false;
	}

	if (distortPoints.empty())
	{
		return false;
	}


	double fc_x = intrinsic.at<double>(0, 0);
	double fc_y = intrinsic.at<double>(1, 1);
	double cc_x = intrinsic.at<double>(0, 2);
	double cc_y = intrinsic.at<double>(1, 2);

	double k1 = distortion.at<double>(0, 0);
	double k2 = distortion.at<double>(0, 1);
	double p1 = distortion.at<double>(0, 2);
	double p2 = distortion.at<double>(0, 3);
	double k3 = distortion.at<double>(0, 4);


	undisted_points.clear();

	for (int i = 0; i < distortPoints.size(); i++)
	{
		cv::Point2d n_p;

		normalizePoint(distortPoints[i].x, distortPoints[i].y, fc_x, fc_y,
			cc_x, cc_y, k1, k2, k3, p1, p2, n_p.x, n_p.y);

		undisted_points.push_back(n_p);
	}


	return true;
}


void LookupTableFunction::normalizePoint(
	double x, double y,
	double fc_x, double fc_y,
	double cc_x, double cc_y,
	double k1, double k2, double k3, double p1, double p2,
	double& x_norm, double& y_norm)
{
	double x_distort = (x - cc_x) / fc_x;
	double y_distort = (y - cc_y) / fc_y;

	double x_iter = x_distort;
	double y_iter = y_distort;

	for (int i = 0; i < 20; i++)
	{
		double r_2 = x_iter * x_iter + y_iter * y_iter;
		double r_4 = r_2 * r_2;
		double r_6 = r_4 * r_2;
		double k_radial = 1 + k1 * r_2 + k2 * r_4 + k3 * r_6;
		double delta_x = 2 * p1 * x_iter * y_iter + p2 * (r_2 + 2 * x_iter * x_iter);
		double delta_y = p1 * (r_2 + 2 * y_iter * y_iter) + 2 * p2 * x_iter * y_iter;
		x_iter = (x_distort - delta_x) / k_radial;
		y_iter = (y_distort - delta_y) / k_radial;
	}
	//x_norm = x_iter*fc_x+ cc_x;
	//y_norm = y_iter*fc_y + cc_y;

	x_norm = x_iter;
	y_norm = y_iter;

	return;
}


bool LookupTableFunction::generateRotateTable(cv::Mat camera_intrinsic_, cv::Mat camera_distortion_, cv::Mat rectify_R,
	cv::Size size, cv::Mat& rotate_x, cv::Mat& rotate_y)
{

	int nr = size.height;
	int nc = size.width;

	cv::Mat undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	std::vector<cv::Point2d> camera_points;

	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			cv::Point2d camera_p(c, r);
			camera_points.push_back(camera_p);
		}
	}
	/*********************************************************************************/


	std::vector<cv::Point2d> correct_camera_points;
	undistortedPoints(camera_points, camera_intrinsic_, camera_distortion_, correct_camera_points);

	//r1 rotate
	for (int r = 0; r < nr; r++)
	{
		double* ptr_x = undistort_map_x.ptr<double>(r);
		double* ptr_y = undistort_map_y.ptr<double>(r);
		for (int c = 0; c < nc; c++)
		{
			cv::Mat vec(3, 1, CV_64FC1, cv::Scalar(1));

			int offset = r * nc + c;

			vec.at<double>(0, 0) = correct_camera_points[offset].x;
			vec.at<double>(1, 0) = correct_camera_points[offset].y;

			cv::Mat rotate = rectify_R * vec;

			rotate = rotate / rotate.at<double>(2, 0);

			ptr_x[c] = rotate.at<double>(0, 0);
			ptr_y[c] = rotate.at<double>(1, 0);

		}
	}

	rotate_x = undistort_map_x.clone();
	rotate_y = undistort_map_y.clone();

	return true;
}


bool LookupTableFunction::generateGridMapping(cv::Mat rotate_x, cv::Mat rotate_y, cv::Mat& map)
{
	int table_rows = 4000;
	int table_cols = 2000;

	cv::Mat interpolation_map(table_rows, table_cols, CV_64FC1, cv::Scalar(-2));

	std::cout << "interpolation_map!" << std::endl;
	int nr = rotate_x.rows;
	int nc = rotate_x.cols;
	std::cout << nr << std::endl;
	std::cout << nc << std::endl;

	for (int r = 0; r < nr; r++)
	{
		double* ptr_x = rotate_x.ptr<double>(r);
		double* ptr_y = rotate_y.ptr<double>(r);

		for (int c = 0; c < nc; c++)
		{
			int m_r = 2000 * (ptr_y[c] + 1);

			if (m_r >= table_rows || m_r < 0)
			{
				// std::cout << "error m_r: " << m_r << std::endl;
			}
			else
			{
				interpolation_map.at<double>(m_r, c) = ptr_x[c];
			}

		}
	}



	for (int r = 1; r < interpolation_map.rows - 1; r++)
	{
		double* ptr_up = interpolation_map.ptr<double>(r - 1);
		double* ptr_data = interpolation_map.ptr<double>(r);
		double* ptr_next = interpolation_map.ptr<double>(r + 1);
		for (int c = 0; c < interpolation_map.cols; c++)
		{
			if (-2 == ptr_data[c])
			{
				if (-2 != ptr_up[c] && -2 != ptr_next[c])
				{
					ptr_data[c] = (ptr_up[c] + ptr_next[c]) / 2.0;
				}
			}
		}

	}


	map = interpolation_map.clone();

	return true;
}



bool LookupTableFunction::generateLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{

	if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
		!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	{
		return false;
	}

	cv::Mat R1, R2, P1, P2, Q, V1, V2;

	cv::stereoRectify(camera_intrinsic_, camera_distortion_, project_intrinsic_, projector_distortion_,
		image_size_, rotation_matrix_, translation_matrix_,
		R1, R2, P1, P2, Q);

	int nr = image_size_.height;
	int nc = image_size_.width;

	cv::Mat xL_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xL_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	cv::Mat xR_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xR_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	//cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));


	generateRotateTable(camera_intrinsic_, camera_distortion_, R1, image_size_, xL_undistort_map_x, xL_undistort_map_y);
	generateRotateTable(project_intrinsic_, projector_distortion_, R2, cv::Size(1920, 1200), xR_undistort_map_x, xR_undistort_map_y);


	cv::Mat mapping;
	generateGridMapping(xR_undistort_map_x, xR_undistort_map_y, mapping);


	single_pattern_mapping_ = mapping.clone();
	xL_rotate_x_ = xL_undistort_map_x.clone();
	xL_rotate_y_ = xL_undistort_map_y.clone();
	R_1_ = R1.clone();

	xL_rotate_x = xL_undistort_map_x.clone();
	xL_rotate_y = xL_undistort_map_y.clone();
	rectify_R1 = R1.clone();
	pattern_mapping = mapping.clone();


	return true;
}

/******************************************************************************************************/

bool LookupTableFunction::readTableFloat(std::string dir_path, cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y,
	cv::Mat& rectify_R1, cv::Mat& pattern_mapping, cv::Mat& pattern_minimapping, int width, int height)
{

	/****************************************************************************************************************************/

	if (!readBinMappingFloat(3, 3, dir_path + "/R1.bin", R_1_))
	{
		return false;
	}

	if (!readBinMappingFloat(128, 128, dir_path + "/single_pattern_minimapping.bin", pattern_minimapping))
	{
		std::cout << "read mini mapping error!";
	}

	if (!readBinMappingFloat(4000, 2000, dir_path + "/single_pattern_mapping.bin", single_pattern_mapping_))
	{
		return false;
	}

	if (!readBinMappingFloat(height, width, dir_path + "/combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x_))
	{
		return false;
	}

	if (!readBinMappingFloat(height, width, dir_path + "/combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y_))
	{
		return false;
	}
	/****************************************************************************************************************************/



	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}

	xL_rotate_x = xL_rotate_x_.clone();
	xL_rotate_y = xL_rotate_y_.clone();
	rectify_R1 = R_1_.clone();
	pattern_mapping = single_pattern_mapping_.clone();

	return true;
}

bool LookupTableFunction::readTableFloat(std::string dir_path, cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{

	/****************************************************************************************************************************/

	if (!readBinMappingFloat(3, 3, dir_path + "/R1.bin", R_1_))
	{
		return false;
	}

	if (!readBinMappingFloat(4000, 2000, dir_path + "/single_pattern_mapping.bin", single_pattern_mapping_))
	{
		return false;
	}

	if (!readBinMappingFloat(1200, 1920, dir_path + "/combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x_))
	{
		return false;
	}

	if (!readBinMappingFloat(1200, 1920, dir_path + "/combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y_))
	{
		return false;
	}
	/****************************************************************************************************************************/



	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}

	xL_rotate_x = xL_rotate_x_.clone();
	xL_rotate_y = xL_rotate_y_.clone();
	rectify_R1 = R_1_.clone();
	pattern_mapping = single_pattern_mapping_.clone();

	return true;
}

bool LookupTableFunction::readTable(std::string dir_path, int rows, int cols)
{

	//single_pattern_mapping_ = readmapping(4000, 2000, "../config_files/single_pattern_mapping.txt");
	//xL_rotate_x_ = readmapping(rows, cols, "../config_files/combine_xL_rotate_x_cam1_iter.txt");
	//xL_rotate_y_ = readmapping(rows, cols, "../config_files/combine_xL_rotate_y_cam1_iter.txt");
	//R_1_ = readmapping(3, 3, "../config_files/R1.txt");



	/****************************************************************************************************************************/

	if (!readBinMapping(3, 3, dir_path + "/R1.bin", R_1_))
	{
		return false;
	}

	if (!readBinMapping(4000, 2000, dir_path + "/single_pattern_mapping.bin", single_pattern_mapping_))
	{
		return false;
	}

	if (!readBinMapping(rows, cols, dir_path + "/combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x_))
	{
		return false;
	}

	if (!readBinMapping(rows, cols, dir_path + "/combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y_))
	{
		return false;
	}
	/****************************************************************************************************************************/

	cv::Mat test_single_pattern_mapping_ = single_pattern_mapping_.clone();
	cv::Mat test_xL_rotate_x_ = xL_rotate_x_.clone();
	cv::Mat test_xL_rotate_y_ = xL_rotate_y_.clone();
	cv::Mat test_R_1_ = R_1_.clone();


	//xR_rotate_x_ = readmapping(rows, cols, "../config_files/combine_xR_rotate_x_cam1_iter.txt");
	//xR_rotate_y_ = readmapping(rows, cols, "../config_files/combine_xR_rotate_y_cam1_iter.txt");

	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}


	//saveBinMapping("../test_write_single_pattern_mapping.dat", single_pattern_mapping_);

	//cv::Mat test_read_mat;

	//TestReadBinMapping(4000, 2000, "../test_write_single_pattern_mapping.dat", test_read_mat);

	//if (!readBinMapping(4000, 2000, dir_path + "/single_pattern_mapping.dat", single_pattern_mapping_))
	//{
	//	return false;
	//}

	return true;

}


bool LookupTableFunction::getLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{
	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}

	xL_rotate_x = xL_rotate_x_.clone();
	xL_rotate_y = xL_rotate_y_.clone();
	rectify_R1 = R_1_.clone();
	pattern_mapping = single_pattern_mapping_.clone();

	return true;
}

bool LookupTableFunction::setLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping)
{

	xL_rotate_x_ = xL_rotate_x.clone();
	xL_rotate_y_ = xL_rotate_y.clone();
	R_1_ = rectify_R1.clone();
	single_pattern_mapping_ = pattern_mapping.clone();

	return true;
}

void LookupTableFunction::setCalibData(struct CameraCalibParam calib_param)
{
	cv::Mat camera_intrinsic = cv::Mat(3, 3, CV_32F, calib_param.camera_intrinsic);
	cv::Mat camera_distortion = cv::Mat(1, 5, CV_32F, calib_param.camera_distortion);

	cv::Mat project_intrinsic = cv::Mat(3, 3, CV_32F, calib_param.projector_intrinsic);
	cv::Mat projector_distortion = cv::Mat(1, 5, CV_32F, calib_param.projector_distortion);

	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32F, calib_param.rotation_matrix);
	cv::Mat translation_matrix = cv::Mat(3, 1, CV_32F, calib_param.translation_matrix);

	camera_intrinsic.convertTo(camera_intrinsic, CV_64F);
	camera_distortion.convertTo(camera_distortion, CV_64F);
	project_intrinsic.convertTo(project_intrinsic, CV_64F);
	projector_distortion.convertTo(projector_distortion, CV_64F);
	rotation_matrix.convertTo(rotation_matrix, CV_64F);
	translation_matrix.convertTo(translation_matrix, CV_64F);

	camera_intrinsic_ = camera_intrinsic.clone();
	project_intrinsic_ = project_intrinsic.clone();
	camera_distortion_ = camera_distortion.clone();
	projector_distortion_ = projector_distortion.clone();
	rotation_matrix_ = rotation_matrix.clone();
	translation_matrix_ = translation_matrix.clone();

	value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));

}

bool LookupTableFunction::readCalibData(std::string path)
{
	std::ifstream myfile(path);

	if (!myfile.is_open())
	{
		std::cout << "can not open this file" << std::endl;
		return 0;
	}

	double I[40] = { 0 };


	for (int i = 0; i < 40; i++)
	{

		myfile >> I[i];
		//std::cout << I[i]<<std::endl;

	}

	myfile.close();

	cv::Mat camera_intrinsic = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	cv::Mat camera_distortion = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	cv::Mat project_intrinsic = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	cv::Mat projector_distortion = cv::Mat(1, 5, CV_64F, cv::Scalar(0.0));

	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_64F, cv::Scalar(0.0));
	cv::Mat translation_matrix = cv::Mat(3, 1, CV_64F, cv::Scalar(0.0));


	double* ptr_c_i = camera_intrinsic.ptr<double>(0);

	ptr_c_i[0] = I[0];
	ptr_c_i[1] = I[1];
	ptr_c_i[2] = I[2];
	ptr_c_i[3] = I[3];
	ptr_c_i[4] = I[4];
	ptr_c_i[5] = I[5];
	ptr_c_i[6] = I[6];
	ptr_c_i[7] = I[7];
	ptr_c_i[8] = I[8];


	double* ptr_c_d = camera_distortion.ptr<double>(0);
	ptr_c_d[0] = I[9];
	ptr_c_d[1] = I[10];
	ptr_c_d[2] = I[11];
	ptr_c_d[3] = I[12];
	ptr_c_d[4] = I[13];


	double* ptr_p_i = project_intrinsic.ptr<double>(0);
	ptr_p_i[0] = I[14];
	ptr_p_i[1] = I[15];
	ptr_p_i[2] = I[16];
	ptr_p_i[3] = I[17];
	ptr_p_i[4] = I[18];
	ptr_p_i[5] = I[19];
	ptr_p_i[6] = I[20];
	ptr_p_i[7] = I[21];
	ptr_p_i[8] = I[22];


	double* ptr_p_d = projector_distortion.ptr<double>(0);
	ptr_p_d[0] = I[23];
	ptr_p_d[1] = I[24];
	ptr_p_d[2] = I[25];
	ptr_p_d[3] = I[26];
	ptr_p_d[4] = I[27];


	double* ptr_r = rotation_matrix.ptr<double>(0);
	ptr_r[0] = I[28];
	ptr_r[1] = I[29];
	ptr_r[2] = I[30];
	ptr_r[3] = I[31];
	ptr_r[4] = I[32];
	ptr_r[5] = I[33];
	ptr_r[6] = I[34];
	ptr_r[7] = I[35];
	ptr_r[8] = I[36];


	double* ptr_t = translation_matrix.ptr<double>(0);
	ptr_t[0] = I[37];
	ptr_t[1] = I[38];
	ptr_t[2] = I[39];


	camera_intrinsic_ = camera_intrinsic.clone();
	project_intrinsic_ = project_intrinsic.clone();
	camera_distortion_ = camera_distortion.clone();
	projector_distortion_ = projector_distortion.clone();
	rotation_matrix_ = rotation_matrix.clone();
	translation_matrix_ = translation_matrix.clone();

	value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));

	return true;
}

bool LookupTableFunction::readCalibXml(std::string path)
{

	//FileIoFunction file_io_machine;
 //
	//file_io_machine.readCalibXml(QString::fromStdString(path), camera_intrinsic_, project_intrinsic_, camera_distortion_,
	//	projector_distortion_, rotation_matrix_, translation_matrix_);

	///**************************************************************************************************************/
	//if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
	//	!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	//{
	//	return false;
	//}

	//value_b_ = sqrt(pow(translation_matrix_.at<double>(0, 0), 2) + pow(translation_matrix_.at<double>(1, 0), 2) + pow(translation_matrix_.at<double>(2, 0), 2));


	return true;



}


void LookupTableFunction::setRebuildValueB(double  b)
{
	value_b_ = b;

}

bool LookupTableFunction::getRebuildValueB(double& b)
{
	if (!translation_matrix_.data)
	{
		return false;
	}
	b = value_b_;

	return true;
}


bool LookupTableFunction::TestReadBinMapping(int rows, int cols, std::string mapping_file, cv::Mat& out_map)
{
	cv::Mat map(rows, cols, CV_64F, cv::Scalar(0));

	std::ifstream inFile(mapping_file, std::ios::in | std::ios::binary); 
	if (!inFile) {
		std::cout << "error" << std::endl;
		return false;
	}
	while (inFile.read((char*)map.data, sizeof(double) * rows * cols)) {
		int readedBytes = inFile.gcount();
		std::cout << readedBytes << std::endl;
	}
	inFile.close();

	out_map = map.clone();
	return true;
}


bool LookupTableFunction::saveBinMappingFloat(std::string mapping_file, cv::Mat out_map)
{
	std::ofstream outFile(mapping_file, std::ios::out | std::ios::binary);

	outFile.write((char*)out_map.data, sizeof(float) * out_map.rows * out_map.cols);
	outFile.close();
	return 0;
}


bool LookupTableFunction::saveBinMapping(std::string mapping_file, cv::Mat out_map)
{
	std::ofstream outFile(mapping_file, std::ios::out | std::ios::binary);

	outFile.write((char*)out_map.data, sizeof(double) * out_map.rows * out_map.cols);
	outFile.close();
	return 0;
}


bool LookupTableFunction::readBinMappingFloat(int rows, int cols, std::string mapping_file, cv::Mat& out_map)
{
	cv::Mat map(rows, cols, CV_32F, cv::Scalar(0));
	std::ifstream inFile(mapping_file, std::ios::in | std::ios::binary); 
	if (!inFile) {
		std::cout << "error" << std::endl;
		out_map = map.clone();
		return false;
	}
	while (inFile.read((char*)map.data, sizeof(float) * rows * cols)) { 
		int readedBytes = inFile.gcount(); 
		//std::cout << readedBytes << std::endl;
	}
	inFile.close();

	out_map = map.clone();
	return true;
}

bool LookupTableFunction::readBinMapping(int rows, int cols, std::string mapping_file, cv::Mat& out_map)
{
	//QFile file(QString::fromStdString(mapping_file)); 

	//if (!file.open(QIODevice::ReadOnly))
	//{
	//	return false;
	//}
	//QDataStream in(&file);
	//cv::Mat m = cv::Mat(rows, cols, CV_64F);
	//for (int i = 0; i < rows; i++) {
	//	for (int j = 0; j < cols; j++) {
	//		in >> m.at<double>(i, j);
	//	}
	//}

	//out_map = m.clone();

	//return true;

	cv::Mat map(rows, cols, CV_64F, cv::Scalar(0));
	std::ifstream inFile(mapping_file, std::ios::in | std::ios::binary); 
	if (!inFile) {
		std::cout << "error" << std::endl;
		return false;
	}
	while (inFile.read((char*)map.data, sizeof(double) * rows * cols)) { 
		int readedBytes = inFile.gcount(); 
		//std::cout << readedBytes << std::endl;
	}
	inFile.close();

	out_map = map.clone();
	return true;
}

cv::Mat LookupTableFunction::readmapping(int rows, int cols, std::string mapping_file)
{

	cv::Mat m = cv::Mat(rows, cols, CV_64F);
	std::ifstream in(mapping_file);
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			in >> m.at<double>(i, j);
			//cout << m.at<double>(i, j) << endl;
		}
	}
	return m;

}

double LookupTableFunction::Bilinear_interpolation(double x, double y, cv::Mat& mapping)
{

	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	if (x2 >= mapping.cols || y2 >= mapping.rows)
	{
		return -2;
	}

	//row-y,col-x

	if (x1 == 1919) {
		double out = mapping.at<double>(y1, x1);
		return out;
	}
	else {
		double fq11 = mapping.at<double>(y1, x1);
		double fq21 = mapping.at<double>(y1, x2);
		double fq12 = mapping.at<double>(y2, x1);
		double fq22 = mapping.at<double>(y2, x2);

		if (-2 == fq11 || -2 == fq21 || -2 == fq12 || -2 == fq22)
		{
			return -2;
		}

		double out = fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1);

		return out;
	}


}

double LookupTableFunction::depth_per_point_6patterns_combine(double Xc, double Yc, double Xp, cv::Mat xL_rotate_x, cv::Mat xL_rotate_y, cv::Mat single_pattern_mapping, double b, double& disparity)
{


	double Xcr = Bilinear_interpolation(Xc, Yc, xL_rotate_x);
	double Ycr = Bilinear_interpolation(Xc, Yc, xL_rotate_y);
	double Xpr = Bilinear_interpolation(Xp, (Ycr + 1) * 2000, single_pattern_mapping);

	if (-2 == Xcr || -2 == Ycr || -2 == Xpr)
	{
		return 0;
	}


	//double delta_X = Xcr - Xpr;

	double delta_X = std::abs(Xpr - Xcr);
	double Z = b / delta_X;



	disparity = delta_X * 1000;



	//if (disparity < 50 || disparity> 85)
	//{
	//	Z = -10;
	//}

	return Z;

}


bool LookupTableFunction::mat_float_to_double(cv::Mat org_mat, cv::Mat& dst_mat)
{
	if (!org_mat.data)
	{
		return false;
	}

	if (org_mat.type() != CV_32F)
	{
		return false;
	}

	int rows = org_mat.rows;
	int cols = org_mat.cols;

	cv::Mat dst(rows, cols, CV_64F, cv::Scalar(0));


	for (int r = 0; r < rows; r++)
	{
		float* ptr_o = org_mat.ptr<float>(r);
		double* ptr_d = dst.ptr<double>(r);
		for (int c = 0; c < cols; c++)
		{
			ptr_d[c] = (double)ptr_o[c];
		}

	}

	dst_mat = dst.clone();


	return true;
}

bool LookupTableFunction::mat_double_to_float(cv::Mat org_mat, cv::Mat& dst_mat)
{
	if (!org_mat.data)
	{
		return false;
	}

	if (org_mat.type() != CV_64F)
	{
		return false;
	}

	int rows = org_mat.rows;
	int cols = org_mat.cols;

	cv::Mat dst(rows, cols, CV_32F, cv::Scalar(0));


	for (int r = 0; r < rows; r++)
	{
		double* ptr_o = org_mat.ptr<double>(r);
		float* ptr_d = dst.ptr<float>(r);
		for (int c = 0; c < cols; c++)
		{
			ptr_d[c] = (float)ptr_o[c];
		}

	}

	dst_mat = dst.clone();


	return true;
}

bool LookupTableFunction::undistortedImage(cv::Mat distort_img, cv::Mat& undistort_img)
{
	if (distort_img.empty())
	{
		return false;
	}

	if (camera_intrinsic_.empty() || camera_distortion_.empty())
	{
		return false;
	}

	cv::undistort(distort_img, undistort_img, camera_intrinsic_, camera_distortion_);


	return true;

}

bool LookupTableFunction::rebuildData(cv::Mat unwrap_map_x, int group_num, cv::Mat& deep_map, cv::Mat& mask)
{

	if (!unwrap_map_x.data) {
		return false;
	}

	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		std::cout << "Please read table first!";
		return false;
	}

	cv::Mat single_pattern_mapping = single_pattern_mapping_.clone();
	cv::Mat xL_rotate_y = xL_rotate_y_.clone();
	cv::Mat xL_rotate_x = xL_rotate_x_.clone();
	double b = value_b_;


	
	double phase_max = 2 * CV_PI * pow(2.0, group_num - 1);
	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;

	
	cv::Mat all_deep_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(0));
	deep_map = all_deep_map.clone();

	cv::Mat disparity_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(0));

#pragma omp parallel for
	for (int Yc = 0; Yc < nr; Yc++) {
		double* ptr_x = unwrap_map_x.ptr<double>(Yc);
		double* ptr_d = deep_map.ptr<double>(Yc);
		uchar* ptr_m = mask.ptr<uchar>(Yc);
		double* ptr_disparity = disparity_map.ptr<double>(Yc);

		for (int Xc = 0; Xc < nc; Xc++) {
			
			double Xp = dlp_width_ * (ptr_x[Xc] - 0*CV_2PI/128.0) / phase_max;
			double d = 0;
			
			if (ptr_m[Xc] == 255 && Xp > 0) {
				
				ptr_d[Xc] = depth_per_point_6patterns_combine(Xc, Yc, Xp, xL_rotate_x, xL_rotate_y, single_pattern_mapping, b, d);

				if (0 == ptr_d[Xc])
				{
					ptr_m[Xc] = 0;
				}

				ptr_disparity[Xc] = d;
			}
			else {
				ptr_d[Xc] = 0;
				ptr_m[Xc] = 0;
			}


		}
	}
	return true;
}


bool LookupTableFunction::generate_pointcloud(cv::Mat z, cv::Mat& mask, cv::Mat& map_xyz)
{

	if (!z.data)
	{
		return false;
	}

	unsigned char* row_pointer_z = (unsigned char*)z.data;
	double* pix_pointer_z;
	unsigned char* row_pointer_mask = (unsigned char*)mask.data;
	unsigned char* pix_pointer_mask;
	int width_step_float = z.step;
	int width_step_uchar = mask.step;
	std::vector<cv::Point3d> pcd;

	cv::Mat R1 = R_1_.clone();

	cv::Mat R1_t = R1.t();
	double R1_t1 = R1_t.at<double>(0, 0);
	double R1_t2 = R1_t.at<double>(0, 1);
	double R1_t3 = R1_t.at<double>(0, 2);
	double R1_t4 = R1_t.at<double>(1, 0);
	double R1_t5 = R1_t.at<double>(1, 1);
	double R1_t6 = R1_t.at<double>(1, 2);
	double R1_t7 = R1_t.at<double>(2, 0);
	double R1_t8 = R1_t.at<double>(2, 1);
	double R1_t9 = R1_t.at<double>(2, 2);


	cv::Mat xL_rotate_y = xL_rotate_y_.clone();
	cv::Mat xL_rotate_x = xL_rotate_x_.clone();

	int img_height = mask.rows;
	int img_width = mask.cols;

	cv::Mat xyz_mat(img_height, img_width, CV_64FC3, cv::Scalar(NULL, NULL, NULL));


	//#pragma omp parallel for
	for (int yc = 0; yc < img_height; yc++) {
		pix_pointer_z = (double*)(row_pointer_z + yc * width_step_float);
		pix_pointer_mask = row_pointer_mask + yc * width_step_uchar;
		cv::Vec3d* ptr_xyz = xyz_mat.ptr<cv::Vec3d>(yc);

		for (int xc = 0; xc < img_width; xc++) {

			cv::Point3d point;
			if (*pix_pointer_mask != 0) {
				double xcr = Bilinear_interpolation(xc, yc, xL_rotate_x);
				double ycr = Bilinear_interpolation(xc, yc, xL_rotate_y);

				point.x = (*pix_pointer_z) * xcr * R1_t1 + (*pix_pointer_z) * ycr * R1_t2 + (*pix_pointer_z) * R1_t3;
				point.y = (*pix_pointer_z) * xcr * R1_t4 + (*pix_pointer_z) * ycr * R1_t5 + (*pix_pointer_z) * R1_t6;
				point.z = (*pix_pointer_z) * xcr * R1_t7 + (*pix_pointer_z) * ycr * R1_t8 + (*pix_pointer_z) * R1_t9;


				if (point.z > min_low_z_ && point.z < max_max_z_)
				{
					ptr_xyz[xc][0] = point.x;
					ptr_xyz[xc][1] = point.y;
					ptr_xyz[xc][2] = point.z;
				}
				else
				{
					ptr_xyz[xc][0] = 0;
					ptr_xyz[xc][1] = 0;
					ptr_xyz[xc][2] = 0;
				}



				pcd.push_back(point);
			}
			pix_pointer_mask++;
			pix_pointer_z++;
		}
		//row_pointer_z += width_step_float;
		//row_pointer_mask += width_step_uchar;

	}

	//std::cout << "Point Cloud Numbers:" << pcd.size() << std::endl;

	map_xyz = xyz_mat.clone();


	return true;
}




/*------------------------------------class MiniLookUpTableFunction-------------------------------------*/


bool MiniLookupTableFunction::rebuildData(cv::Mat unwrap_map_x, int group_num, cv::Mat& deep_map, cv::Mat& mask)
{

	std::cout << "开始rebuildData" << std::endl;

	if (!unwrap_map_x.data) {
		return false;
	}

	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		std::cout << "Please read table first!";
		return false;
	}

	cv::Mat single_pattern_mapping = the_mini_map_.clone();

	//std::cout << the_mini_map_ << std::endl;

	cv::Mat xL_rotate_y = xL_rotate_y_.clone();
	cv::Mat xL_rotate_x = xL_rotate_x_.clone();
	double b = value_b_;


	
	double phase_max = 2 * CV_PI * pow(2.0, group_num - 1);
	int nr = unwrap_map_x.rows;
	int nc = unwrap_map_x.cols;

	
	cv::Mat all_deep_map = cv::Mat(nr, nc, CV_64F, cv::Scalar(0));
	deep_map = all_deep_map.clone();


#pragma omp parallel for
	for (int Yc = 0; Yc < nr; Yc++) {
		// ptr_x是相位
		double* ptr_x = unwrap_map_x.ptr<double>(Yc);
		// ptr_d是存储深度信息的
		double* ptr_d = deep_map.ptr<double>(Yc);
		// ptr_m是确定是否生成
		uchar* ptr_m = mask.ptr<uchar>(Yc);

		for (int Xc = 0; Xc < nc; Xc++) {
			
			double Xp = 1280 * ptr_x[Xc] / phase_max;
			
			if (ptr_m[Xc] == 255) {
				
				ptr_d[Xc] = depth_per_point_6patterns_combine(Xc, Yc, Xp, xL_rotate_x, xL_rotate_y, single_pattern_mapping, b);
			}
			else {
				ptr_d[Xc] = 0;
			}


		}
	}
	return true;
}

// 用虚函数重写生成点云的函数
double MiniLookupTableFunction::depth_per_point_6patterns_combine(double Xc, double Yc, double Xp, cv::Mat xL_rotate_x, cv::Mat xL_rotate_y, cv::Mat single_pattern_mapping, double b)
{

	// 去畸变后的x
	double Xcr = Bilinear_interpolation(Xc, Yc, xL_rotate_x);
	// 去畸变后的y
	double Ycr = Bilinear_interpolation(Xc, Yc, xL_rotate_y);
	double Xpr = Bilinear_mini_interpolation(Xp, (Ycr + 1) * 2000, single_pattern_mapping);
	//double Xpr = Bilinear_interpolation(Xp, (Ycr + 1) * 2000, single_pattern_mapping_);
	double delta_X = std::abs(Xcr - Xpr);


	double Z = b / delta_X;
	return Z;

}

double MiniLookupTableFunction::Bilinear_mini_interpolation(double x, double y, cv::Mat& mapping)
{

	//先找到这个点所对应的mini中的四个角点
	//然后将这四个点算出来
	//最后双线性插值

	int index_x1 = floor(x / 16);
	int index_y1 = floor((y - 1301) / 16);
	int index_x2 = index_x1 + 1;
	int index_y2 = index_y1 + 1;

	int x1 = index_x1 * 16;
	int y1 = index_y1 * 16 + 1301;
	int x2 = x1 + 16;
	int y2 = y1 + 16;

	//因为我生成的表比原来大，所以无需考虑边界条件
	//fq_xy

	double fq11 = mapping.at<double>(index_y1, index_x1);
	double fq21 = mapping.at<double>(index_y1, index_x2);
	double fq12 = mapping.at<double>(index_y2, index_x1);
	double fq22 = mapping.at<double>(index_y2, index_x2);

	//std::cout << the_mini_map_ << std::endl;


	double out = (fq11 * (x2 - x) * (y2 - y) + fq21 * (x - x1) * (y2 - y) + fq12 * (x2 - x) * (y - y1) + fq22 * (x - x1) * (y - y1)) / 256.;
	//std::cout << out << std::endl;
	return out;


}

// 用虚函数重写读查找表的函数
bool MiniLookupTableFunction::readTableFloat(std::string dir_path, cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_minimapping)
{

	/****************************************************************************************************************************/

	if (!readBinMappingFloat(3, 3, dir_path + "/R1.bin", R_1_))
	{
		return false;
	}

	if (!readBinMappingFloat(128, 128, dir_path + "/single_pattern_minimapping.bin", single_pattern_mapping_))
	{
		return false;
	}

	if (!readBinMappingFloat(1200, 1920, dir_path + "/combine_xL_rotate_x_cam1_iter.bin", xL_rotate_x_))
	{
		return false;
	}

	if (!readBinMappingFloat(1200, 1920, dir_path + "/combine_xL_rotate_y_cam1_iter.bin", xL_rotate_y_))
	{
		return false;
	}
	/****************************************************************************************************************************/



	if (!single_pattern_mapping_.data || !xL_rotate_x_.data || !xL_rotate_y_.data)
	{
		return false;
	}

	xL_rotate_x = xL_rotate_x_.clone();
	xL_rotate_y = xL_rotate_y_.clone();
	rectify_R1 = R_1_.clone();
	pattern_minimapping = single_pattern_mapping_.clone();

	return true;
}

bool MiniLookupTableFunction::generateLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_minimapping)
{

	if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
		!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	{
		return false;
	}

	cv::Mat R1, R2, P1, P2, Q, V1, V2;

	cv::stereoRectify(camera_intrinsic_, camera_distortion_, project_intrinsic_, projector_distortion_,
		image_size_, rotation_matrix_, translation_matrix_,
		R1, R2, P1, P2, Q);

	int nr = image_size_.height;
	int nc = image_size_.width;

	cv::Mat xL_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xL_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	cv::Mat xR_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xR_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	//cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));


	generateRotateTable(camera_intrinsic_, camera_distortion_, R1, image_size_, xL_undistort_map_x, xL_undistort_map_y);
	generateRotateTable(project_intrinsic_, projector_distortion_, R2, cv::Size(1920, 1200), xR_undistort_map_x, xR_undistort_map_y);

	// 取四个点作为裁剪map的依据
	projector_left_up_y = (xR_undistort_map_y.at<double>(5, 0) + 1) * 2000;
	projector_right_up_y = (xR_undistort_map_y.at<double>(5, (dlp_width_ -1)) + 1) * 2000;
	projector_left_down_y = (xR_undistort_map_y.at<double>(dlp_height_ - 1, 0) + 1) * 2000;
	projector_right_down_y = (xR_undistort_map_y.at<double>(dlp_height_ - 1, (dlp_width_ -1)) + 1) * 2000;

	cv::Mat mapping, mini_mapping;
	generateMiniGridMapping(mapping, mini_mapping);

	single_pattern_mapping_ = mapping.clone();
	xL_rotate_x_ = xL_undistort_map_x.clone();
	xL_rotate_y_ = xL_undistort_map_y.clone();
	R_1_ = R1.clone();

	xL_rotate_x = xL_undistort_map_x.clone();
	xL_rotate_y = xL_undistort_map_y.clone();
	rectify_R1 = R1.clone();
	pattern_minimapping = mini_mapping.clone();


	return true;
}


bool MiniLookupTableFunction::generateBigLookTable(cv::Mat& xL_rotate_x, cv::Mat& xL_rotate_y, cv::Mat& rectify_R1, cv::Mat& pattern_mapping, cv::Mat& pattern_minimapping)
{

	if (!camera_intrinsic_.data || !camera_distortion_.data || !project_intrinsic_.data ||
		!projector_distortion_.data || !rotation_matrix_.data || !translation_matrix_.data)
	{
		return false;
	}

	cv::Mat R1, R2, P1, P2, Q, V1, V2;

	cv::stereoRectify(camera_intrinsic_, camera_distortion_, project_intrinsic_, projector_distortion_,
		image_size_, rotation_matrix_, translation_matrix_,
		R1, R2, P1, P2, Q);

	int nr = image_size_.height;
	int nc = image_size_.width;

	cv::Mat xL_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xL_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	cv::Mat xR_undistort_map_x(nr, nc, CV_64F, cv::Scalar(-2));
	cv::Mat xR_undistort_map_y(nr, nc, CV_64F, cv::Scalar(-2));

	//cv::Mat mask(nr, nc, CV_8U, cv::Scalar(0));


	generateRotateTable(camera_intrinsic_, camera_distortion_, R1, image_size_, xL_undistort_map_x, xL_undistort_map_y);
	generateRotateTable(project_intrinsic_, projector_distortion_, R2, image_size_, xR_undistort_map_x, xR_undistort_map_y);

	// 取四个点作为裁剪map的依据
	//projector_left_up_y = (xR_undistort_map_y.at<double>(5, 0) + 1) * 2000;
	//projector_right_up_y = (xR_undistort_map_y.at<double>(5, 1279) + 1) * 2000;
	//projector_left_down_y = (xR_undistort_map_y.at<double>(714, 0) + 1) * 2000;
	//projector_right_down_y = (xR_undistort_map_y.at<double>(714, 1279) + 1) * 2000;

	projector_left_up_y = (xR_undistort_map_y.at<double>(5, 0) + 1) * 2000;
	projector_right_up_y = (xR_undistort_map_y.at<double>(5, dlp_width_ - 1) + 1) * 2000;
	projector_left_down_y = (xR_undistort_map_y.at<double>(dlp_height_ - 1, 0) + 1) * 2000;
	projector_right_down_y = (xR_undistort_map_y.at<double>(dlp_height_ - 1, dlp_width_ - 1) + 1) * 2000;

	cv::Mat mapping, mini_mapping;
	generateMiniGridMapping(mapping, mini_mapping);

	single_pattern_mapping_ = mapping.clone();
	xL_rotate_x_ = xL_undistort_map_x.clone();
	xL_rotate_y_ = xL_undistort_map_y.clone();
	R_1_ = R1.clone();

	xL_rotate_x = xL_undistort_map_x.clone();
	xL_rotate_y = xL_undistort_map_y.clone();
	rectify_R1 = R1.clone();
	pattern_mapping = mapping.clone();
	pattern_minimapping = mini_mapping.clone();


	return true;
}


// 注意事项：这里的行是归一化后的row经过式子：(row+1)*2000
// 输入参数：1.一个数是去畸变后的行；2.一个数是未去畸变的列；
// 输出参数：返回值是一个畸变矫正后的列
double MiniLookupTableFunction::findTheColByRowAndCol(cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs,
	cv::Mat& _matR, double _rowUndistorted, double _colDistorted)
{
	double col_undistorted = 0;
	double row_undistorted_normalized = _rowUndistorted / 2000 - 1;
	//std::cout << "row_undistorted_normalized" << row_undistorted_normalized << std::endl;
	// 这是归一化的相机的内参
	cv::Mat newCameraMatrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	col_undistorted = calculateNormalizedCol(_cameraMatrix, _distCoeffs, _matR, newCameraMatrix, row_undistorted_normalized, _colDistorted);
	//std::cout << "col_undistorted" << col_undistorted << std::endl;
	return col_undistorted;
}

// 函数功能：输入一个有空隙的表，查找中间的空值，将所有空隙填满
// 输入参数：一个4000*2000的表，这个表是没有进行线性插值的表
// 输出参数：输出一个1606*1921的经过裁切的表
bool MiniLookupTableFunction::cutTheMap(cv::Mat& _originalMap, cv::Mat& _cuttedMap)
{
	cv::Mat cuttedMap(2033, 2033, CV_32F, cv::Scalar(-2));
	for (int i = 0; i < 2033; i += 1)
	{
		float* ptr_original = _originalMap.ptr<float>(i + 1301);
		float* ptr_cutted = cuttedMap.ptr<float>(i);
		for (int j = 0; j < 2033; j += 1)
		{
			ptr_cutted[j] = ptr_original[j];
		}
	}
	_cuttedMap = cuttedMap.clone();
	return true;
}


// 输入参数：1.一个原始的4000*2000的空表；2.一个掩膜的表，表中的数据都是需要计算的；3.一个表去接收这个值；
// 输出参数：一个在mask范围上已经完成插值的表，只差一步下采样
bool MiniLookupTableFunction::makeMapFull(cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs,
	cv::Mat& _matR, cv::Mat& _notFullData, cv::Mat& _theMask, cv::Mat& _isFullData)
{
	int nc = _notFullData.cols;
	int nr = _notFullData.rows;
	cv::Mat isFullData(nr, nc, CV_32F, cv::Scalar(-2));
	//cv::Mat cuttedFullData;
	//cutTheMap(_notFullData, cuttedFullData);
	// 先计算填充满这个不满的表，然后计算

	for (int row = 0; row < nr; row += 1)
	{
		float* ptr_mask = _theMask.ptr<float>(row);
		float* ptr_data = isFullData.ptr<float>(row);
		for (int col = 0; col < nc; col += 1)
		{
			if (ptr_mask[col] == 1)
			{
				//std::cout << "找到点" << ptr_data[col] << std::endl;
				ptr_data[col] = findTheColByRowAndCol(_cameraMatrix, _distCoeffs,
					_matR, row, col);
				//std::cout << "找到点" << ptr_data[col] << std::endl;
			}
		}
	}
	//isFullData = isFullData + _notFullData;
	_isFullData = isFullData.clone();
	return true;
}


//功能：从一个已经填满的表中下采样出一个表格
//输入参数：1.已经填充满的map，尺寸是2033*2033
//输出参数：1.下采样后的map，尺寸是128*128
//先假设这个map的大小
//注意：在实际书写代码的时候，要将需要输出的Mat数据在函数内部实现一个，最后使用.clone()函数将其作为输出
bool MiniLookupTableFunction::generateMiniLookupTable(cv::Mat _theFullGridMap, cv::Mat& _theMiniMap)
{
	//cv::Mat Grid_map = cv::Mat_<double>(3, 3);
	//cv::Mat Grid(3, 3, CV_32FC1, cv::Scalar(-2));
	cv::Mat theMiniMap((_theFullGridMap.rows - 1) / 16 + 1, (_theFullGridMap.cols - 1) / 16 + 1, CV_32F, cv::Scalar(0));
	for (int i = 0; i < (_theFullGridMap.rows - 1) / 16 + 1; i += 1)
	{
		float* ptr_f = _theFullGridMap.ptr<float>(16 * i);
		float* ptr_m = theMiniMap.ptr<float>(i);
		for (int j = 0; j < (_theFullGridMap.cols - 1) / 16 + 1; j += 1)
		{
			ptr_m[j] = ptr_f[16 * j];
		}
	}
	_theMiniMap = theMiniMap.clone();
	return true;

}


// 函数功能：将压缩的表还原回去
bool MiniLookupTableFunction::takeMiniMapBack(cv::Mat& _theMiniMap, cv::Mat& _theOriginalMap, cv::Mat& _theFullSizeMask)
{
	cv::Mat theOriginalMap(4000, 2033, CV_32F, cv::Scalar(-2));
	// 1.先返回一个裁切过的map
	cv::Mat cuttedMap(2033, 2033, CV_32F, cv::Scalar(-2));
	// 先填充最右侧的点
	for (int row = 0; row < 2032; row += 1)
	{
		int col = 2032;
		float* ptr_cutted = cuttedMap.ptr<float>(row);
		// 确认比例
		double weight_of_up = (16. - (row % 16)) / 16.;
		double weight_of_down = (row % 16) / 16.;




		// 确认点的位置
		int up_1 = floor(row / 16);
		int down_1 = floor(row / 16) + 1;
		//
		float* ptr_up = _theMiniMap.ptr<float>(up_1);
		float* ptr_down = _theMiniMap.ptr<float>(down_1);
		// (1606-1)/15+1 = 108,(1921-1)/15+1 = 129

		ptr_cutted[col] = ptr_up[127] * weight_of_up + ptr_down[127] * weight_of_down;

	}
	// 填充最下侧的点
	for (int col = 0; col < 2032; col += 1)
	{
		int row = 2032;
		float* ptr_cutted = cuttedMap.ptr<float>(row);
		// 确认比例
		double weight_of_left = (16. - (col % 16)) / 16.;
		double weight_of_right = (col % 16) / 16.;


		// 确认点的位置
		int left_1 = floor(col / 16);
		int right_1 = floor(col / 16) + 1;
		//
		float* ptr_left = _theMiniMap.ptr<float>(127);
		float* ptr_right = _theMiniMap.ptr<float>(127);
		// (1606-1)/15+1 = 108
		ptr_cutted[col] = ptr_left[left_1] * weight_of_left + ptr_right[right_1] * weight_of_right;

	}
	// 填充右下的点
	cuttedMap.ptr<float>(2032)[2032] = _theMiniMap.ptr<float>(127)[127];
	// 填充中间部分

	for (int row = 0; row < 2032; row += 1)
	{
		float* ptr_cutted = cuttedMap.ptr<float>(row);
		for (int col = 0; col < 2032; col += 1)
		{
			double m = row % 16;
			double n = col % 16;
			double weight_l_up = (16 - m) * (16 - n) / 256;
			double weight_r_up = (16 - m) * (n) / 256;
			double weight_l_down = (m) * (16 - n) / 256;
			double weight_r_down = (m) * (n) / 256;
			// 确认插值使用的四个点
			int l_up1 = floor(row / 16);
			int l_up2 = floor(col / 16);
			int r_up1 = floor(row / 16);
			int r_up2 = floor(col / 16) + 1;
			int l_down1 = floor(row / 16) + 1;
			int l_down2 = floor(col / 16);
			int r_down1 = floor(row / 16) + 1;
			int r_down2 = floor(col / 16) + 1;
			float* ptr_up = _theMiniMap.ptr<float>(l_up1);
			float* ptr_down = _theMiniMap.ptr<float>(l_down1);
			// 将计算的值填入表中
			ptr_cutted[col] = weight_l_up * ptr_up[l_up2] + weight_r_up * ptr_up[r_up2] + weight_l_down * ptr_down[l_down2] + weight_r_down * ptr_down[r_down2];
		}
	}
	// 返回结果
	// 根据掩膜的情况返回值回去
	for (int row = 1301; row < 3334; row += 1)
	{
		float* ptr_original = theOriginalMap.ptr<float>(row);
		float* ptr_cutted = cuttedMap.ptr<float>(row - 1301);
		float* ptr_mask = _theFullSizeMask.ptr<float>(row);
		for (int col = 0; col < 2033; col += 1)
		{
			if (ptr_mask[col] == 1)
			{
				ptr_original[col] = ptr_cutted[col];
			}
		}
	}
	_theOriginalMap = theOriginalMap.clone();
	return true;
}

double MiniLookupTableFunction::calculateNormalizedCol(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
	cv::InputArray _matR, cv::InputArray _newCameraMatrix, double _row_undistorted, double _col_distorted)
{

	// 精度
	double precision = 1e-8;
	// 需要返回的参数
	double col_undistorted;
	//相机内参、畸变矩阵
	cv::Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();
	//旋转矩阵、摄像机参数矩阵
	cv::Mat matR = _matR.getMat(), newCameraMatrix = _newCameraMatrix.getMat();


	cv::Mat_<double> R = cv::Mat_<double>::eye(3, 3);
	//A为相机内参
	cv::Mat_<double> A = cv::Mat_<double>(cameraMatrix), Ar;

	//Ar 为摄像机坐标参数
	Ar = cv::Mat_<double>(newCameraMatrix);
	//R  为旋转矩阵
	R = cv::Mat_<double>(matR);
	// distCoeffs为畸变矩阵
	distCoeffs = cv::Mat_<double>(distCoeffs);
	// 取逆
	cv::Mat_<double> iR = (Ar.colRange(0, 3) * R).inv(cv::DECOMP_LU);
	//ir IR矩阵的指针
	const double* ir = &iR(0, 0);
	//获取相机的内参 u0  v0 为主坐标点   fx fy 为焦距
	double u0 = A(0, 2), v0 = A(1, 2);
	double fx = A(0, 0), fy = A(1, 1);

	if (distCoeffs.rows != 1 && !distCoeffs.isContinuous())
		distCoeffs = distCoeffs.t();

	//畸变参数计算
	double k1 = ((double*)distCoeffs.data)[0];
	double k2 = ((double*)distCoeffs.data)[1];
	double p1 = ((double*)distCoeffs.data)[2];
	double p2 = ((double*)distCoeffs.data)[3];
	double k3 = distCoeffs.cols + distCoeffs.rows - 1 >= 5 ? ((double*)distCoeffs.data)[4] : 0.;
	double k4 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[5] : 0.;
	double k5 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[6] : 0.;
	double k6 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[7] : 0.;

	// 核心算法：输入的行已知，但输入的列是未知的，所以要根据猜测的列计算出一个畸变矫正前的列，根据大小的对比可以优化输入的列，当精度达到时就退出
	double guess_left = 0;
	double guess_right = 1;
	double guess_col_distorted = -1;
	double guess_col_distorted_r = -1.1;

	double _xx = _row_undistorted * ir[1] + ir[2];
	double _yy = _row_undistorted * ir[4] + ir[5];
	double _ww = _row_undistorted * ir[7] + ir[8];

	int num = 0;
	double right_maximum;
	double left_minimum;

	// 寻找极值点
	for (col_undistorted = (guess_left + guess_right) / 2; abs(guess_col_distorted_r - guess_col_distorted) > 1e-16; col_undistorted = (guess_left + guess_right) / 2)
	{
		//std::cout << "运行进入函数" << col_undistorted << std::endl;
		double _x = _xx + col_undistorted * ir[0];
		double _y = _yy + col_undistorted * ir[3];
		double _w = _ww + col_undistorted * ir[6];
		//获取摄像机坐标系第四列参数
		//归一化坐标，此处的X，Y是已经投影结束的坐标
		double w = 1. / _w, x = _x * w, y = _y * w;
		double x2 = x * x, y2 = y * y;
		double r2 = x2 + y2, _2xy = 2 * x * y;
		double kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
		//归一化坐标转化为图像坐标
		double u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
		double v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;
		guess_col_distorted = u;

		//std::cout << "运行进入函数" << col_undistorted << std::endl;
		_x = _xx + (col_undistorted + 1e-8) * ir[0];
		_y = _yy + (col_undistorted + 1e-8) * ir[3];
		_w = _ww + (col_undistorted + 1e-8) * ir[6];
		//获取摄像机坐标系第四列参数
		//归一化坐标，此处的X，Y是已经投影结束的坐标
		w = 1. / _w, x = _x * w, y = _y * w;
		x2 = x * x, y2 = y * y;
		r2 = x2 + y2, _2xy = 2 * x * y;
		kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
		//归一化坐标转化为图像坐标
		u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
		v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;
		guess_col_distorted_r = u;

		if (guess_col_distorted_r - guess_col_distorted > 0.0)
		{
			guess_left = col_undistorted;
		}
		else
		{
			guess_right = col_undistorted;
		}
		num += 1;
		if (num >= 100)
		{
			break;
		}
	}

	right_maximum = col_undistorted;
	guess_left = -1;
	guess_right = 0;
	guess_col_distorted = -1;
	guess_col_distorted_r = -1.1;

	// 寻找极值点
	for (col_undistorted = (guess_left + guess_right) / 2; abs(guess_col_distorted_r - guess_col_distorted) > 1e-16; col_undistorted = (guess_left + guess_right) / 2)
	{
		double _x = _xx + col_undistorted * ir[0];
		double _y = _yy + col_undistorted * ir[3];
		double _w = _ww + col_undistorted * ir[6];
		//获取摄像机坐标系第四列参数
		//归一化坐标，此处的X，Y是已经投影结束的坐标
		double w = 1. / _w, x = _x * w, y = _y * w;
		double x2 = x * x, y2 = y * y;
		double r2 = x2 + y2, _2xy = 2 * x * y;
		double kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
		//归一化坐标转化为图像坐标
		double u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
		double v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;
		guess_col_distorted = u;

		_x = _xx + (col_undistorted + 1e-8) * ir[0];
		_y = _yy + (col_undistorted + 1e-8) * ir[3];
		_w = _ww + (col_undistorted + 1e-8) * ir[6];
		//获取摄像机坐标系第四列参数
		//归一化坐标，此处的X，Y是已经投影结束的坐标
		w = 1. / _w, x = _x * w, y = _y * w;
		x2 = x * x, y2 = y * y;
		r2 = x2 + y2, _2xy = 2 * x * y;
		kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
		//归一化坐标转化为图像坐标
		u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
		v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;
		guess_col_distorted_r = u;

		if (guess_col_distorted_r - guess_col_distorted > 0.0)
		{
			guess_right = col_undistorted;
		}
		else
		{
			guess_left = col_undistorted;
		}
		num += 1;
		if (num >= 100)
		{
			//std::cout << "1001" << std::endl;
			break;
		}
	}
	left_minimum = col_undistorted;
	guess_left = left_minimum;
	guess_right = right_maximum;
	guess_col_distorted = -1;
	num = 0;

	for (col_undistorted = (guess_left + guess_right) / 2; abs(_col_distorted - guess_col_distorted) > precision; col_undistorted = (guess_left + guess_right) / 2)
	{
		double _x = _xx + col_undistorted * ir[0];
		double _y = _yy + col_undistorted * ir[3];
		double _w = _ww + col_undistorted * ir[6];
		//获取摄像机坐标系第四列参数
		//归一化坐标，此处的X，Y是已经投影结束的坐标
		double w = 1. / _w, x = _x * w, y = _y * w;
		double x2 = x * x, y2 = y * y;
		double r2 = x2 + y2, _2xy = 2 * x * y;
		double kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
		//归一化坐标转化为图像坐标
		double u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
		double v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;
		guess_col_distorted = u;

		num += 1;
		if ((_col_distorted - guess_col_distorted) < 0)
		{
			guess_right = (guess_right + guess_left) / 2;
		}
		else
		{
			guess_left = (guess_right + guess_left) / 2;
		}
		if (num >= 200)
		{
			return -2.;
			break;
		}
	}
	return col_undistorted;
}


bool MiniLookupTableFunction::generateColOfDistorted(cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
	cv::InputArray _matR, cv::InputArray _newCameraMatrix, cv::Mat dis_col_data)
{

	// 精度
	double precision = 1e-8;
	// 需要返回的参数
	double col_undistorted;
	//相机内参、畸变矩阵
	cv::Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();
	//旋转矩阵、摄像机参数矩阵
	cv::Mat matR = _matR.getMat(), newCameraMatrix = _newCameraMatrix.getMat();


	cv::Mat_<double> R = cv::Mat_<double>::eye(3, 3);
	//A为相机内参
	cv::Mat_<double> A = cv::Mat_<double>(cameraMatrix), Ar;

	//Ar 为摄像机坐标参数
	Ar = cv::Mat_<double>(newCameraMatrix);
	//R  为旋转矩阵
	R = cv::Mat_<double>(matR);
	// distCoeffs为畸变矩阵
	distCoeffs = cv::Mat_<double>(distCoeffs);
	// 取逆
	cv::Mat_<double> iR = (Ar.colRange(0, 3) * R).inv(cv::DECOMP_LU);
	//ir IR矩阵的指针
	const double* ir = &iR(0, 0);
	//获取相机的内参 u0  v0 为主坐标点   fx fy 为焦距
	double u0 = A(0, 2), v0 = A(1, 2);
	double fx = A(0, 0), fy = A(1, 1);

	if (distCoeffs.rows != 1 && !distCoeffs.isContinuous())
		distCoeffs = distCoeffs.t();

	//畸变参数计算
	double k1 = ((double*)distCoeffs.data)[0];
	double k2 = ((double*)distCoeffs.data)[1];
	double p1 = ((double*)distCoeffs.data)[2];
	double p2 = ((double*)distCoeffs.data)[3];
	double k3 = distCoeffs.cols + distCoeffs.rows - 1 >= 5 ? ((double*)distCoeffs.data)[4] : 0.;
	double k4 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[5] : 0.;
	double k5 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[6] : 0.;
	double k6 = distCoeffs.cols + distCoeffs.rows - 1 >= 8 ? ((double*)distCoeffs.data)[7] : 0.;

	//k1 = 0;
	//k2 = 0;
	//p1 = 0;
	//p2 = 0;
	//k3 = 0;
	//k4 = 0;
	//k5 = 0;
	//k6 = 0;

	// 核心算法：输入的行已知，但输入的列是未知的，所以要根据猜测的列计算出一个畸变矫正前的列，根据大小的对比可以优化输入的列，当精度达到时就退出
	double guess_left = -1;
	double guess_right = 1;
	double guess_col_distorted = -1;
	double guess_col_distorted_r = -1.1;

	double _xx = 0 * ir[1] + ir[2];
	double _yy = 0 * ir[4] + ir[5];
	double _ww = 0 * ir[7] + ir[8];

	cv::Mat col_of_distorted_data(1, 2000, CV_32F, cv::Scalar(-2));
	float* dis_col = col_of_distorted_data.ptr<float>(0);
	//std::cout << "_col_distorted" << _col_distorted << std::endl;

	for (int col = 0; col < 2000; col += 1)
	{
		col_undistorted = col / 1000. - 1.;
		//std::cout << "运行进入函数" << col_undistorted << std::endl;
		double _x = _xx + col_undistorted * ir[0];
		double _y = _yy + col_undistorted * ir[3];
		double _w = _ww + col_undistorted * ir[6];
		//获取摄像机坐标系第四列参数
		//归一化坐标，此处的X，Y是已经投影结束的坐标
		double w = 1. / _w, x = _x * w, y = _y * w;
		double x2 = x * x, y2 = y * y;
		double r2 = x2 + y2, _2xy = 2 * x * y;
		double kr = (1 + ((k3 * r2 + k2) * r2 + k1) * r2) / (1 + ((k6 * r2 + k5) * r2 + k4) * r2);
		//归一化坐标转化为图像坐标
		double u = fx * (x * kr + p1 * _2xy + p2 * (r2 + 2 * x2)) + u0;
		double v = fy * (y * kr + p1 * (r2 + 2 * y2) + p2 * _2xy) + v0;

		dis_col[col] = u;

	}
	dis_col_data = col_of_distorted_data.clone();

	std::cout << "great!!success!!" << std::endl;
	return 1;
}
// 
bool MiniLookupTableFunction::generateMiniGridMapping(cv::Mat& _LookupTable, cv::Mat& _MiniLookupTable)
{

	//cv::Size image_size = cv::Size(1920, 1200);
	cv::Mat RL, RR, PL, PR, Q;


	cv::stereoRectify(camera_intrinsic_, camera_distortion_, project_intrinsic_, projector_distortion_,
		image_size_, rotation_matrix_, translation_matrix_, RL, RR, PL, PR, Q);


	// std::cout << intrinsicMatrix_projector << std::endl;

	//readBinMappingFloat(4000, 2000, "C:\\Users\\68253\\Desktop\\test_map_0618\\single_pattern_mapping.bin", theOriginalMap);
	// 1.generateTheMaskToFillData:输入未插值的原始表；输出一个掩膜以方便直到哪些地方是需要插值的；
	// 测试之后问题大
	// 直接对原始的表进行插值
	cv::Mat theMask(4000, 2033, CV_32F, cv::Scalar(0));
	cv::Mat theLookUpTable(4000, 2033, CV_32F, cv::Scalar(0));
	cv::Mat resultTable(4000, 2033, CV_32F, cv::Scalar(-2));

	for (int row = 1301; row < 3334; row += 1)
	{
		float* ptr_mask = theMask.ptr<float>(row);
		for (int col = 0; col < 2033; col += 1)
		{
			ptr_mask[col] = 1;
		}
	}

	// 2.3.calculateNormalizedCol:输入归一化的行和未归一化的列，计算出归一化的列
	// 2.2.findTheColByRowAndCol:输入表中的坐标，就可以计算出这个值
	// 2.1.makeMapFull:输入一个有空值的表，输出一个没有空值的表（4000*2000），而且符合下采样条件

	cv::Mat intrinsicMatrix_projector, dist_projector;
	project_intrinsic_.convertTo(intrinsicMatrix_projector, CV_32F);
	projector_distortion_.convertTo(dist_projector, CV_32F);
	makeMapFull(intrinsicMatrix_projector, dist_projector, RR, theLookUpTable, theMask, theLookUpTable);
	//saveBinMappingFloat("C:\\Users\\68253\\Desktop\\test_map_0618\\myTable.bin", theLookUpTable);
	//std::cout << "myTable.bin" << theLookUpTable.size << std::endl;

	cv::Mat cuttedMap;
	// 3.cutTheMap:输入一个裁剪前的表，输出一个裁剪后的表；
	cutTheMap(theLookUpTable, cuttedMap);

	// 4.generateMiniLookupTable:下采样生成我需要的表；
	cv::Mat theMiniMap;
	generateMiniLookupTable(cuttedMap, theMiniMap);

	// 5.takeMiniMapBack:将下采样的表根据插值回去，并且根据掩膜还原回去，得到一个原始表；
	cv::Mat theOriginalMap1;
	cv::Mat LookupTable(4000, 2000, CV_32F, cv::Scalar(-2));
	takeMiniMapBack(theMiniMap, theOriginalMap1, theMask);

	// 将4000*2033的表裁剪为4000*2000
	for (int row = 0; row < 4000; row += 1)
	{
		float* ptr_before = theOriginalMap1.ptr<float>(row);
		float* ptr_after = LookupTable.ptr<float>(row);
		for (int col = 0; col < 2000; col += 1)
		{
			ptr_after[col] = ptr_before[col];
		}

	}

	//将插值的数据存入theBigLookUpTable，以便使用
	cv::Mat theBigLookUpTable(4000, 2000, CV_32F, cv::Scalar(-2));
	// 斜率
	double k_up = (projector_right_up_y - projector_left_up_y) / (dlp_width_ -1);
	double k_down = (projector_right_down_y - projector_left_down_y) / (dlp_width_-1);

	for (int row = 0; row < 4000; row += 1)
	{
		float* ptr_before = theLookUpTable.ptr<float>(row);
		float* ptr_after = theBigLookUpTable.ptr<float>(row);
		for (int col = 0; col < 2000; col += 1)
		{
			if (col < 7 || col > dlp_width_ - 7)
			{
				ptr_after[col] = -2;
			}
			else if (row < (k_up * col + projector_left_up_y) || row >(k_down * col + projector_left_down_y))
			{
				ptr_after[col] = -2;
			}
			else
			{
				ptr_after[col] = ptr_before[col];
			}

		}

	}


	cv::Mat MiniLookupTable;
	LookupTable.convertTo(LookupTable, CV_64F);		//插值还原的表
	theMiniMap.convertTo(MiniLookupTable, CV_64F);		//压缩表
	theBigLookUpTable.convertTo(theBigLookUpTable, CV_64F);		//计算得出的表

	_MiniLookupTable = MiniLookupTable.clone();
	_LookupTable = theBigLookUpTable.clone();

	// 将计算结果写入成员变量
	single_pattern_mapping_ = LookupTable.clone();
	the_mini_map_ = MiniLookupTable.clone();
	std::cout << "the_mini_map_";

	return true;
}


bool MiniLookupTableFunction::readCameraCalibData(std::string path, struct CameraCalibParam& param)
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