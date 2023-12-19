#include "management.cuh"
#include "minsw.cuh"

#include <opencv2/photo.hpp>

int h_image_width_ = 0;
int h_image_height_ = 0;
 
bool merge_brightness_flag_ = true;

#define CHECK(call)\
{\
  const cudaError_t error=call;\
  if(error!=cudaSuccess)\
  {\
      printf("ERROR: %s:%d,",__FILE__,__LINE__);\
      printf("code:%d,reason:%s\n",error,cudaGetErrorString(error));\
      exit(1);\
  }\
}


dim3 threadsPerBlock(8, 8);
dim3 blocksPerGrid((d_image_width_ + threadsPerBlock.x - 1) / threadsPerBlock.x,
(d_image_height_ + threadsPerBlock.y - 1) / threadsPerBlock.y);


// int d_image_width_ = 1920;
// int d_image_height_ = 1200;
// bool load_calib_data_flag_ = false; 

SystemConfigDataStruct cuda_system_config_settings_machine_;
void cuda_set_param_system_config(SystemConfigDataStruct param)
{
	cuda_system_config_settings_machine_ = param;
}

bool cuda_set_projector_version(int version)
{
    switch (version)
    {
    case DF_PROJECTOR_3010:
    {
		int dlp_width = 1280;
		int dlp_height = 720;
		cuda_set_param_dlp_resolution(dlp_width,dlp_height);
 

        return true;
    }
    break;

    case DF_PROJECTOR_4710:
    {
		int dlp_width = 1920;
		int dlp_height = 1080;
 
		cuda_set_param_dlp_resolution(dlp_width,dlp_height);

 
        return true;
    }
    break;

    default:
        break;
    }

	return false;
}

bool cuda_set_camera_resolution(int width,int height)
{
	h_image_width_ = width;
	h_image_height_ = height;
 
	d_image_width_ = width;
	d_image_height_ = height;

	cudaError_t error_code = cudaMemcpyToSymbol(d_image_width_, &width, sizeof(int)); 
	if(error_code!= cudaSuccess)
	{
		return false;
	}
		
	error_code = cudaMemcpyToSymbol(d_image_height_, &height, sizeof(int));

	if(error_code!= cudaSuccess)
	{
		return false;
	}
 

	blocksPerGrid.x = (width + threadsPerBlock.x - 1) / threadsPerBlock.x;
	blocksPerGrid.y = (height + threadsPerBlock.y - 1) / threadsPerBlock.y;
 

	return true;
}


//分配basic内存
bool cuda_malloc_basic_memory()
{
    for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
    {
        cudaMalloc((void **)&d_patterns_list_[i], d_image_height_ * d_image_width_ * sizeof(unsigned char)); 
    }
    for (int i = 0; i < LAST_STEPS_NUM; i++)
    {
        cudaMalloc((void **)&d_six_step_patterns_list_[i], d_image_height_ * d_image_width_ * sizeof(unsigned char)); 
    }
    for (int i = 0; i < LAST_STEPS_NUM; i++)
    {
        cudaMalloc((void **)&d_six_step_patterns_convolved_list_[i], d_image_height_ * d_image_width_ * sizeof(unsigned char)); 
    }
	for (int i = 0; i < LAST_STEPS_NUM; i++)
    {
        cudaMalloc((void **)&d_repetition_02_merge_patterns_convolved_list_[i], d_image_height_ * d_image_width_ * sizeof(unsigned short)); 
    }

    // cudaBindTexture(0,texture_patterns_0,d_patterns_list_[0]);
	// cudaBindTexture(0,texture_patterns_1,d_patterns_list_[1]);
	// cudaBindTexture(0,texture_patterns_2,d_patterns_list_[2]);
	// cudaBindTexture(0,texture_patterns_3,d_patterns_list_[3]);
	// cudaBindTexture(0,texture_patterns_4,d_patterns_list_[4]);
	// cudaBindTexture(0,texture_patterns_5,d_patterns_list_[5]);
	// cudaBindTexture(0,texture_patterns_6,d_patterns_list_[6]);
	// cudaBindTexture(0,texture_patterns_7,d_patterns_list_[7]);
	// cudaBindTexture(0,texture_patterns_8,d_patterns_list_[8]);
	// cudaBindTexture(0,texture_patterns_9,d_patterns_list_[9]);
	// cudaBindTexture(0,texture_patterns_10,d_patterns_list_[10]);
	// cudaBindTexture(0,texture_patterns_11,d_patterns_list_[11]);
	// cudaBindTexture(0,texture_patterns_12,d_patterns_list_[12]);
	// cudaBindTexture(0,texture_patterns_13,d_patterns_list_[13]);
	// cudaBindTexture(0,texture_patterns_14,d_patterns_list_[14]);
	// cudaBindTexture(0,texture_patterns_15,d_patterns_list_[15]);
	// cudaBindTexture(0,texture_patterns_16,d_patterns_list_[16]);
	// cudaBindTexture(0,texture_patterns_17,d_patterns_list_[17]);
	// cudaBindTexture(0,texture_patterns_18,d_patterns_list_[18]);

 

	for (int i = 0; i< MAX_WRAP_NUMBER; i++)
	{
		cudaMalloc((void**)&d_wrap_map_list_[i], d_image_height_*d_image_width_ * sizeof(float));
		cudaMalloc((void**)&d_confidence_map_list_[i], d_image_height_*d_image_width_ * sizeof(float)); 
	}

	for (int i = 0; i< MAX_UNWRAP_NUMBER; i++)
	{
		cudaMalloc((void**)&d_unwrap_map_list_[i], d_image_height_*d_image_width_ * sizeof(float)); 
	} 

	for (int i = 0; i< 2; i++)
	{
		cudaMalloc((void**)&d_six_step_pattern_convolution_phase_list_[i], d_image_height_*d_image_width_ * sizeof(float)); 
	} 

	cudaMalloc((void**)&d_brightness_map_, d_image_height_*d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_brightness_short_map_, d_image_height_*d_image_width_ * sizeof(unsigned short));  

	cudaMalloc((void**)&d_mask_map_, d_image_height_*d_image_width_ * sizeof(unsigned char)); 
	cudaMalloc((void**)&d_fisher_mask_, d_image_height_ * d_image_width_ * sizeof(unsigned char));


	cudaMalloc((void**)&d_camera_intrinsic_, 3*3 * sizeof(float));
	cudaMalloc((void**)&d_project_intrinsic_, 3 * 3 * sizeof(float));

	cudaMalloc((void**)&d_camera_distortion_, 1* 5 * sizeof(float));
	cudaMalloc((void**)&d_projector_distortion_, 1 * 5 * sizeof(float));

	cudaMalloc((void**)&d_rotation_matrix_, 3 * 3 * sizeof(float));
	cudaMalloc((void**)&d_translation_matrix_, 1 * 3 * sizeof(float));
 

	cudaMalloc((void**)&d_minsw8_table_, 256* sizeof(unsigned char));

	unsigned char array[256] = { 0, 1, 105, 2, 155, 154, 156, 3, 19, 172, 106, 107, 70, 69, 157, 4, 169,
		66, 192, 193, 220, 67, 91, 90, 170, 171, 211, 108, 221, 68, 6, 5, 255, 152, 126, 23, 50, 
		153, 177, 176, 20, 173, 21, 22, 71, 174, 72, 175, 190, 87, 191, 88, 241, 240, 242, 89, 85,
		86, 212, 109, 136, 239, 7, 110, 233, 130, 128, 129, 28, 131, 27, 26, 234, 235, 147, 44, 29,
		132, 198, 197, 64, 65, 41, 194, 219, 218, 92, 195, 83, 236, 42, 43, 134, 133, 93, 196, 254,
		151, 127, 24, 49, 48, 178, 25, 149, 150, 148, 45, 200, 47, 199, 46, 63, 216, 62, 215, 114, 
		217, 113, 112, 84, 237, 213, 214, 135, 238, 8, 111, 103, 206, 104, 207, 52, 205, 53, 54, 
		18, 121, 209, 208, 223, 120, 158, 55, 168, 15, 39, 142, 117, 118, 244, 141, 17, 16, 210, 57,
		222, 119, 159, 56, 102, 101, 125, 228, 51, 204, 74, 75, 123, 122, 124, 227, 224, 225, 73, 
		226, 189, 36, 38, 37, 138, 139, 243, 140, 188, 35, 59, 58, 137, 34, 160, 161, 232, 79, 231,
		78, 181, 182, 180, 77, 81, 80, 146, 249, 30, 183, 95, 248, 167, 14, 40, 143, 116, 13, 245,
		246, 82, 185, 145, 144, 31, 184, 94, 247, 253, 100, 230, 229, 202, 203, 179, 76, 252, 99, 
		251, 250, 201, 98, 96, 97, 166, 165, 61, 164, 115, 12, 10, 11, 187, 186, 60, 163, 32, 33, 9, 162 };

		
	CHECK(cudaMemcpy(d_minsw8_table_, array, 256 * sizeof(unsigned char), cudaMemcpyHostToDevice));

	cudaMalloc((void**)&d_fisher_confidence_map, d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_convolution_kernal_map, SIZE_OF_CONVOLUTION_KERNAL*SIZE_OF_CONVOLUTION_KERNAL * sizeof(float));
	cudaMalloc((void**)&d_point_cloud_map_, 3*d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_depth_map_, d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_depth_map_temp_, d_image_height_*d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_triangulation_error_map_, d_image_height_*d_image_width_ * sizeof(float));

	
	cudaMalloc((void**)&d_global_light_map_, d_image_height_*d_image_width_ * sizeof(char));
	cudaMalloc((void**)&d_direct_light_map_, d_image_height_*d_image_width_ * sizeof(char));
	cudaMalloc((void**)&d_uncertain_map_, d_image_height_*d_image_width_ * sizeof(char));
 
 	cudaMalloc((void**)&d_single_pattern_mapping_, 4000*2000 * sizeof(float)); 
	cudaMalloc((void**)&d_single_pattern_minimapping_, 128*128 * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_x_, d_image_height_*d_image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_xL_rotate_y_, d_image_height_*d_image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_R_1_, 3*3 * sizeof(float)); 

	cudaMalloc((void**)&d_undistort_map_x_, d_image_height_*d_image_width_ * sizeof(float)); 
	cudaMalloc((void**)&d_undistort_map_y_, d_image_height_*d_image_width_ * sizeof(float)); 

    LOG(INFO)<<"d_image_width_: "<<d_image_width_;
    LOG(INFO)<<"d_image_height_: "<<d_image_height_;
	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_basic_memory()
{

	for (int i = 0; i< MAX_PATTERNS_NUMBER; i++)
	{  
		cudaFree(d_patterns_list_[i]); 
	}
	for (int i = 0; i< LAST_STEPS_NUM; i++)
	{  
		cudaFree(d_six_step_patterns_list_[i]); 
	}
	for (int i = 0; i< LAST_STEPS_NUM; i++)
	{  
		cudaFree(d_six_step_patterns_convolved_list_[i]); 
	}
	for (int i = 0; i< LAST_STEPS_NUM; i++)
	{  
		cudaFree(d_repetition_02_merge_patterns_convolved_list_[i]); 
	}
	// cudaUnbindTexture(texture_patterns_0);
	// cudaUnbindTexture(texture_patterns_1);
	// cudaUnbindTexture(texture_patterns_2);
	// cudaUnbindTexture(texture_patterns_3);
	// cudaUnbindTexture(texture_patterns_4);
	// cudaUnbindTexture(texture_patterns_5);
	// cudaUnbindTexture(texture_patterns_6);
	// cudaUnbindTexture(texture_patterns_7);
	// cudaUnbindTexture(texture_patterns_8);
	// cudaUnbindTexture(texture_patterns_9);
	// cudaUnbindTexture(texture_patterns_10);
	// cudaUnbindTexture(texture_patterns_11);
	// cudaUnbindTexture(texture_patterns_12);
	// cudaUnbindTexture(texture_patterns_13);
	// cudaUnbindTexture(texture_patterns_14);
	// cudaUnbindTexture(texture_patterns_15);
	// cudaUnbindTexture(texture_patterns_16);
	// cudaUnbindTexture(texture_patterns_17);
	// cudaUnbindTexture(texture_patterns_18);

	for (int i = 0; i< MAX_WRAP_NUMBER; i++)
	{  
		cudaFree(d_wrap_map_list_[i]);
		cudaFree(d_confidence_map_list_[i]); 
	}

	for (int i = 0; i< MAX_UNWRAP_NUMBER; i++)
	{ 
		cudaFree(d_unwrap_map_list_[i]); 
	}

	for (int i = 0; i< 2; i++)
	{ 
		cudaFree(d_six_step_pattern_convolution_phase_list_[i]); 
	}

	cudaFree(d_fisher_confidence_map);
	cudaFree(d_convolution_kernal_map);
	cudaFree(d_fisher_mask_);
    cudaFree(d_mask_map_);
    cudaFree(d_brightness_map_);
    cudaFree(d_brightness_short_map_);

    cudaFree(d_point_cloud_map_);
    cudaFree(d_depth_map_);
	cudaFree(d_depth_map_temp_);
    cudaFree(d_triangulation_error_map_);

    cudaFree(d_global_light_map_);
    cudaFree(d_direct_light_map_);
    cudaFree(d_uncertain_map_);
	


    cudaFree(d_camera_intrinsic_);
	cudaFree(d_project_intrinsic_); 
	cudaFree(d_camera_distortion_);
	cudaFree(d_projector_distortion_); 
	cudaFree(d_rotation_matrix_);
	cudaFree(d_translation_matrix_);

    cudaFree(d_minsw8_table_);
	
 
    cudaFree(d_single_pattern_mapping_);
    cudaFree(d_single_pattern_minimapping_);
    cudaFree(d_xL_rotate_x_);
    cudaFree(d_xL_rotate_y_);
    cudaFree(d_R_1_);

    cudaFree(d_undistort_map_x_);
    cudaFree(d_undistort_map_y_);
 

	return true;
}

 //分配hdr内存
bool cuda_malloc_hdr_memory()
{
	for (int i = 0; i< D_HDR_MAX_NUM; i++)
	{
		cudaMalloc((void**)&d_hdr_depth_map_list_[i], d_image_height_*d_image_width_ * sizeof(float));
		cudaMalloc((void**)&d_hdr_brightness_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned char));  
		cudaMalloc((void**)&d_hdr_bright_pixel_sum_list_[i], 1 * sizeof(float)); 
	}
	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_hdr_memory()
{
    for (int i = 0; i< D_HDR_MAX_NUM; i++)
	{ 
		cudaFree(d_hdr_depth_map_list_[i]);
		cudaFree(d_hdr_brightness_list_[i]); 
		cudaFree(d_hdr_bright_pixel_sum_list_[i]);
	}
	
	return true;
}

//分配repetition内存
bool cuda_malloc_repetition_memory()
{
	//分配重复patterns数据
	for(int i= 0;i< D_REPETITIONB_MAX_NUM*6;i++)
	{
		cudaMalloc((void**)&d_repetition_patterns_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned char)); 
	}

	for(int i= 0;i< 6;i++)
	{
		cudaMalloc((void**)&d_repetition_merge_patterns_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned short)); 
	}
 
 	for(int i= 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
		cudaMalloc((void**)&d_repetition_02_merge_patterns_list_[i], d_image_height_*d_image_width_ * sizeof(unsigned short)); 
	}

	cudaMalloc((void**)&d_merge_brightness_map_, d_image_height_*d_image_width_ * sizeof(unsigned short)); 
	
	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_repetition_memory()
{

	//分配重复patterns数据
	for(int i= 0;i< D_REPETITIONB_MAX_NUM*6;i++)
	{
		cudaFree(d_repetition_patterns_list_[i]); 
	}

	for(int i= 0;i< 6;i++)
	{
		cudaFree(d_repetition_merge_patterns_list_[i]);  
	}

	for(int i= 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
		cudaFree(d_repetition_02_merge_patterns_list_[i]);  
	}

	cudaFree(d_merge_brightness_map_);  
	
	return true;
}


/********************************************************************************************/
//copy 
void cuda_copy_calib_data(float* camera_intrinsic, float* project_intrinsic, float* camera_distortion,
	float* projector_distortion, float* rotation_matrix, float* translation_matrix)
{
  
	CHECK(cudaMemcpy(d_camera_intrinsic_, camera_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_project_intrinsic_, project_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpy(d_camera_distortion_, camera_distortion, 1 * 5 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_projector_distortion_, projector_distortion, 1 * 5 * sizeof(float), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpy(d_rotation_matrix_, rotation_matrix, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_translation_matrix_, translation_matrix, 1* 3 * sizeof(float), cudaMemcpyHostToDevice));

	load_calib_data_flag_ = 1;

 
}

void cuda_copy_talbe_to_memory(float* mapping,float* mini_mapping,float* rotate_x,float* rotate_y,float* r_1,float base_line)
{
   
	CHECK(cudaMemcpyAsync(d_R_1_, r_1, 3*3 * sizeof(float), cudaMemcpyHostToDevice)); 
	CHECK(cudaMemcpyAsync(d_single_pattern_minimapping_, mini_mapping, 128 * 128 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_single_pattern_mapping_, mapping, 4000*2000 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_x_, rotate_x, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_xL_rotate_y_, rotate_y, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	
    d_baseline_ = base_line;  
 

	LOG(INFO)<<"d_baseline_: "<<d_baseline_;
	cudaDeviceSynchronize();
}


void coud_copy_undistort_table_to_memory(float* undistort_x_map,float* undistort_y_map)
{
	CHECK(cudaMemcpyAsync(d_undistort_map_x_, undistort_x_map, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_undistort_map_y_, undistort_y_map, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyHostToDevice));
}


bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr,int serial_flag)
{
	if(serial_flag> MAX_PATTERNS_NUMBER)
	{
		return false;
	}

	cv::Mat smooth_mat(d_image_height_, d_image_width_, CV_8UC1, pattern_ptr);
	if (serial_flag < 12)
	{
		LOG(INFO) << "Start GaussianBlur:";
		cv::GaussianBlur(smooth_mat, smooth_mat, cv::Size(5, 5), 1, 1);

		LOG(INFO) << "finished GaussianBlur!";
	}

	CHECK(cudaMemcpyAsync(d_patterns_list_[serial_flag], smooth_mat.data, d_image_height_*d_image_width_* sizeof(unsigned char), cudaMemcpyHostToDevice)); 
}

void cuda_copy_pointcloud_from_memory(float* pointcloud)
{ 
	CHECK(cudaMemcpy(pointcloud, d_point_cloud_map_, 3 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
}

void cuda_copy_depth_from_memory(float* depth)
{
	CHECK(cudaMemcpy(depth, d_depth_map_, d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
} 

void cuda_copy_brightness_from_memory(unsigned char* brightness)
{
	CHECK(cudaMemcpy(brightness, d_brightness_map_, d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost)); 
}

void cuda_copy_convolution_kernal_to_memory(float* convolution_kernal, int kernal_diameter)
{
	CHECK(cudaMemcpyAsync(d_convolution_kernal_map, convolution_kernal, kernal_diameter*kernal_diameter* sizeof(float), cudaMemcpyHostToDevice)); 
}


void cuda_copy_brightness_16_to_memory(unsigned short* brightness)
{
	CHECK(cudaMemcpyAsync(d_brightness_short_map_, brightness, d_image_height_*d_image_width_* sizeof(unsigned short), cudaMemcpyHostToDevice)); 

}

void cuda_copy_brightness_to_memory(unsigned char* brightness)
{ 
	CHECK(cudaMemcpyAsync(d_brightness_map_, brightness, d_image_height_*d_image_width_* sizeof(unsigned char), cudaMemcpyHostToDevice)); 
}


void cuda_clear_reconstruct_cache()
{
	
	CHECK(cudaMemset(d_brightness_map_,0, d_image_height_*d_image_width_ * sizeof(char))); 
	CHECK(cudaMemset(d_depth_map_,0, d_image_height_*d_image_width_ * sizeof(float))); 
	CHECK(cudaMemset(d_point_cloud_map_,0,3* d_image_height_*d_image_width_ * sizeof(float))); 
	CHECK(cudaMemset(d_mask_map_,0,d_image_height_*d_image_width_ * sizeof(unsigned char))); 
    CHECK(cudaMemset(d_fisher_confidence_map,0,d_image_height_*d_image_width_ * sizeof(float))); 

	CHECK(cudaMemset(d_hdr_brightness_list_[0],0, d_image_height_*d_image_width_ * sizeof(char))); 
	CHECK(cudaMemset(d_hdr_brightness_list_[1],0, d_image_height_*d_image_width_ * sizeof(char))); 
	CHECK(cudaMemset(d_hdr_brightness_list_[2],0, d_image_height_*d_image_width_ * sizeof(char))); 
	CHECK(cudaMemset(d_hdr_brightness_list_[3],0, d_image_height_*d_image_width_ * sizeof(char))); 
	CHECK(cudaMemset(d_hdr_brightness_list_[4],0, d_image_height_*d_image_width_ * sizeof(char))); 
	CHECK(cudaMemset(d_hdr_brightness_list_[5],0, d_image_height_*d_image_width_ * sizeof(char)));

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{
		CHECK(cudaMemset(d_direct_light_map_, 0, d_image_height_ * d_image_width_ * sizeof(char)));
		CHECK(cudaMemset(d_global_light_map_, 0, d_image_height_ * d_image_width_ * sizeof(char)));
		CHECK(cudaMemset(d_uncertain_map_, 0, d_image_height_ * d_image_width_ * sizeof(char)));
	}
}


/********************************************************************************************/


bool cuda_compute_phase_shift(int serial_flag)
{
	 
	switch(serial_flag)
	{
		case 0:
		{ 
        	LOG(INFO)<<"kernel_four_step_phase_shift:"<<d_image_width_;
			int i= 0;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);

				// kernel_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 1:
		{

			int i= 4;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				
				// kernel_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
			
		}
		break;
		case 2:
		{ 
			int i= 8;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				
				// kernel_four_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> >(serial_flag,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 3:
		{ 
			int i= 12; 
			kernel_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3],d_patterns_list_[i + 4],d_patterns_list_[i + 5] ,d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);

			if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
			{

				kernel_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[i + 0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
																				 d_patterns_list_[i + 3], d_patterns_list_[i + 4], d_patterns_list_[i + 5],
																				 cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
			
			
			
			}

				// cuda_six_step_phase_shift_texture<< <blocksPerGrid, threadsPerBlock >> > (d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
				// cudaDeviceSynchronize();

				// cv::Mat phase(1200, 1920, CV_32F, cv::Scalar(0));
				// CHECK(cudaMemcpy(phase.data, d_wrap_map_list_[serial_flag], 1 * image_height_ * image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
				// cv::imwrite("phase1.tiff",phase);
		}
		break;
		case 4:
		{
			int i= 18;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 5:
		{
			int i= 22;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
		case 6:
		{
			int i= 26;
			kernel_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0], d_patterns_list_[i + 1], d_patterns_list_[i + 2],
				d_patterns_list_[i + 3], d_wrap_map_list_[serial_flag], d_confidence_map_list_[serial_flag]);
		}
		break;
  
		default :
			break;
	}

	
	
	return true;
}

bool cuda_compute_convolved_image_phase_shift(int serial_flag)
{
	switch (serial_flag)
	{
	case 0:
	{
		kernel_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,
		d_six_step_patterns_convolved_list_[0], d_six_step_patterns_convolved_list_[1], 
		d_six_step_patterns_convolved_list_[2], d_six_step_patterns_convolved_list_[3],
		d_six_step_patterns_convolved_list_[4],d_six_step_patterns_convolved_list_[5],
		d_six_step_pattern_convolution_phase_list_[1], d_confidence_map_list_[3]);
	}
		break;

	case 1:
	{
		kernel_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_convolved_list_[0],
		 d_repetition_02_merge_patterns_convolved_list_[1], d_repetition_02_merge_patterns_convolved_list_[2],
		  d_repetition_02_merge_patterns_convolved_list_[3],d_repetition_02_merge_patterns_convolved_list_[4],
		  d_repetition_02_merge_patterns_convolved_list_[5], 1, d_image_height_, d_image_width_, 
		  d_six_step_pattern_convolution_phase_list_[1], d_confidence_map_list_[3]);
	}
		break;
	
	default:
		break;
	}


	return true;
}

bool cuda_rectify_six_step_pattern_phase(int mode, int kernal_diameter)
{	
	switch (mode)
	{
		case 0:
		{
			cudaDeviceSynchronize();
			LOG(INFO)<<"start six_step blur mode 0";
			for (int i = 0; i < LAST_STEPS_NUM; i += 1)
			{
				kernal_convolution_2D<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_, d_patterns_list_[12 + i],
				 d_six_step_patterns_convolved_list_[i], d_convolution_kernal_map, kernal_diameter);
			}
			cudaDeviceSynchronize();
			LOG(INFO)<<"end six_step blur";

			cuda_compute_convolved_image_phase_shift(0);

			// 计算相位并且补偿
			kernel_six_step_phase_rectify<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[3], 
			d_six_step_pattern_convolution_phase_list_[1], d_wrap_map_list_[3]);
		}
		break;

		case 1:
		{
			cudaDeviceSynchronize();
			LOG(INFO)<<"start six_step blur mode 1";
			for (int i = 0; i < LAST_STEPS_NUM; i += 1)
			{
				kernal_convolution_2D_short<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,
				 d_repetition_02_merge_patterns_list_[12 + i], d_repetition_02_merge_patterns_convolved_list_[i],
				  d_convolution_kernal_map, kernal_diameter);
			}
			cudaDeviceSynchronize();
			LOG(INFO)<<"end six_step blur";

			cuda_compute_convolved_image_phase_shift(1);

			// 计算相位并且补偿
			kernel_six_step_phase_rectify<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[3],
			 d_six_step_pattern_convolution_phase_list_[1], d_wrap_map_list_[3]);
		}
		break;
		case 2:
		{
			//minsw相位校正
			cudaDeviceSynchronize();
			LOG(INFO)<<"start six_step blur mode 0";
			for (int i = 0; i < LAST_STEPS_NUM; i += 1)
			{
				kernal_convolution_2D<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,
				 d_patterns_list_[2 + i], d_six_step_patterns_convolved_list_[i], d_convolution_kernal_map, kernal_diameter);
			}
			cudaDeviceSynchronize();
			LOG(INFO)<<"end six_step blur";

			cuda_compute_convolved_image_phase_shift(0);

			// 计算相位并且补偿
			kernel_six_step_phase_rectify<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[3],
			 d_six_step_pattern_convolution_phase_list_[1], d_wrap_map_list_[3]);
		}
		break; 
		case 3:
		{
			//minsw相位校正repetition
			cudaDeviceSynchronize();
			LOG(INFO)<<"start six_step blur mode 1";
			for (int i = 0; i < LAST_STEPS_NUM; i += 1)
			{
				kernal_convolution_2D_short<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,
				 d_repetition_02_merge_patterns_list_[2 + i], d_repetition_02_merge_patterns_convolved_list_[i],
				  d_convolution_kernal_map, kernal_diameter);
			}
			cudaDeviceSynchronize();
			LOG(INFO)<<"end six_step blur";

			cuda_compute_convolved_image_phase_shift(1);

			// 计算相位并且补偿
			kernel_six_step_phase_rectify<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,
			d_wrap_map_list_[3], d_six_step_pattern_convolution_phase_list_[1], d_wrap_map_list_[3]);
		}
		break;
		default:
			break;
	}

	
}

bool cuda_normalize_phase(int serial_flag)
{
    switch(serial_flag)
	{ 
        case 0:
		{   
            kernel_normalize_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], (float)128.0, d_unwrap_map_list_[0]);  
		}
		break; 
		case 1:
		{   
  
            kernel_normalize_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], (float)18., d_unwrap_map_list_[1]); 
		}
		break;

		case 2:
		{ 
			kernel_normalize_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], (float)72., d_unwrap_map_list_[1]); 
		}
		break;

		default :
			break;
	}


	return true;
}

bool cuda_unwrap_phase_shift(int serial_flag)
{

	switch(serial_flag)
	{ 
		case 1:
		{  
            kernel_unwrap_variable_phase<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[0], d_wrap_map_list_[1], 8.0, CV_PI, d_unwrap_map_list_[0]);
  
		}
		break;

		case 2:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[2], 4.0,CV_PI, d_unwrap_map_list_[0]); 
			// CHECK ( cudaGetLastError () );
		}
		break;
		case 3:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[3], 4.0,1.5, d_unwrap_map_list_[0]); 
 
		}
		break;
		case 4:
		{
 
		}
		break;
		case 5:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[4], d_wrap_map_list_[5], 8.0,CV_PI, d_unwrap_map_list_[1]);
		}
		break;
		case 6:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], d_wrap_map_list_[6], 4.0,CV_PI, d_unwrap_map_list_[1]);
 
			LOG(INFO)<<"unwrap 6:  ";

		}
		break;
		case 7:
		{
			kernel_unwrap_variable_phase << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[1], d_wrap_map_list_[7], 4.0,CV_PI, d_unwrap_map_list_[1]);
 
		 	LOG(INFO)<<"unwrap 7:  ";

		}
		break;
 

		default :
			break;
	}


	return true;
}

bool cuda_unwrap_phase_shift_base_fisher_confidence(int serial_flag)
{

	switch(serial_flag)
	{ 
		case 1:
		{  
            kernel_unwrap_variable_phase_base_confidence<< <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_wrap_map_list_[0], d_wrap_map_list_[1], 8.0, CV_PI, FISHER_RATE_1, d_fisher_confidence_map, d_unwrap_map_list_[0]);
  
		}
		break;

		case 2:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase_base_confidence << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[2], 4.0,CV_PI, FISHER_RATE_2, d_fisher_confidence_map, d_unwrap_map_list_[0]); 
			// CHECK ( cudaGetLastError () );
		}
		break;
		case 3:
		{ 
			// CHECK( cudaFuncSetCacheConfig (kernel_unwrap_variable_phase, cudaFuncCachePreferL1) );
			kernel_unwrap_variable_phase_base_confidence << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_unwrap_map_list_[0], d_wrap_map_list_[3], 4.0,1.5, FISHER_RATE_3, d_fisher_confidence_map, d_unwrap_map_list_[0]); 
 
		}
		break;
		default :
			break;
	}


	return true;
}

/********************************************************************************************************************************************/

bool cuda_generate_pointcloud_base_minitable()
{
		if(1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_reflect_filter)
	{ 
		LOG(INFO)<<"filter_reflect_noise start:"; 
		cuda_filter_reflect_noise(d_unwrap_map_list_[0]); 

		cudaDeviceSynchronize();
		LOG(INFO)<<"filter_reflect_noise end";
	}

	kernel_reconstruct_pointcloud_base_minitable<< <blocksPerGrid, threadsPerBlock>> > (d_image_width_,d_image_height_,d_xL_rotate_x_,d_xL_rotate_y_,d_single_pattern_minimapping_,d_R_1_,d_baseline_,
	d_confidence_map_list_[3],d_unwrap_map_list_[0],d_point_cloud_map_,d_depth_map_);

 
}


bool cuda_merge_brigntness(int hdr_num, unsigned char* brightness)
{
	if(!merge_brightness_flag_)
	{ 
		return false;
	} 

	std::vector<cv::Mat> brightness_list;
	cv::Mat image_b(h_image_height_, h_image_width_, CV_8U, cv::Scalar(0));

	cudaDeviceSynchronize();
	for (int i = 0; i < hdr_num; i++)
	{

		CHECK(cudaMemcpy(image_b.data, d_hdr_brightness_list_[i], 1 * h_image_height_ * h_image_width_ * sizeof(uchar), cudaMemcpyDeviceToHost));
		brightness_list.push_back(image_b.clone());
	}

	LOG(INFO) << "process: "<<hdr_num;
	cv::Mat exposureFusion;
	cv::Ptr<cv::MergeMertens> mergeMertens = cv::createMergeMertens();
	mergeMertens->process(brightness_list, exposureFusion);
  
	for (int r = 0; r < h_image_height_; r++)
	{ 
		float *ptr_fusion = exposureFusion.ptr<float>(r);

		for (int c = 0; c < h_image_width_; c++)
		{
			if (ptr_fusion[c] > 1)
			{ 
				brightness[r*h_image_width_+c] = 255;
			}
			else
			{ 
				brightness[r*h_image_width_+c] = 255 * ptr_fusion[c];
			}
		}
	}

	LOG(INFO) << "get exposureFusion!"; 
}

bool cuda_generate_pointcloud_base_table()
{
	// cv::Mat phase(d_image_height_,d_image_width_,CV_32FC1,cv::Scalar(0));
	// CHECK(cudaMemcpy(phase.data, d_unwrap_map_list_[0], 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// cv::imwrite("phase.tiff", phase);
	
	// if(1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	// { 
	// 	LOG(INFO)<<"filter_reflect_noise start:"; 
	// 	cuda_filter_reflect_noise(d_unwrap_map_list_[0]); 
	
	// 	// cudaDeviceSynchronize();
	// 	LOG(INFO)<<"filter_reflect_noise end";
	// }



	kernel_reconstruct_pointcloud_base_table<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_xL_rotate_x_, d_xL_rotate_y_, d_single_pattern_mapping_, d_R_1_, d_baseline_,
																				 d_confidence_map_list_[3], d_unwrap_map_list_[0], d_point_cloud_map_, d_depth_map_);

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{
		kernel_remove_mask_result<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_uncertain_map_,
		cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_threshold,d_depth_map_,d_point_cloud_map_);

		// cudaDeviceSynchronize();

		// cv::Mat uncertain_map(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
		// CHECK(cudaMemcpy(uncertain_map.data, d_uncertain_map_, 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
		// cv::imwrite("uncertain_map.bmp", uncertain_map);
	}

	// cv::Mat depth(d_image_height_,d_image_width_,CV_32FC1,cv::Scalar(0));
	// CHECK(cudaMemcpy(depth.data, d_depth_map_, 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// cv::imwrite("depth.tiff", depth);
}

/********************************************************************************************************************************************/


bool cuda_copy_result_to_hdr_color(int serial_flag,int brigntness_serial,cv::Mat brightness)
{
	CHECK(cudaMemcpyAsync(d_hdr_brightness_list_[serial_flag], brightness.data, 1 * d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));


	if(!load_calib_data_flag_)
	{
		return false;
	}
 
	// cv::imwrite("brightness.bmp",brightness);

	CHECK(cudaMemcpyAsync(d_hdr_depth_map_list_[serial_flag], d_depth_map_, 1 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToDevice)); 

	float val  = 0;
	CHECK(cudaMemcpyAsync(d_hdr_bright_pixel_sum_list_[serial_flag], &val, sizeof(float), cudaMemcpyHostToDevice)); 
 	cuda_count_sum_pixel << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_list_[serial_flag],d_image_height_,d_image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
	LOG(INFO)<<"cuda_copy_result_to_hdr color: "<<serial_flag;
	return true;
}

// bool cuda_copy_result_to_hdr_16(int serial_flag,int brigntness_serial)
// {
	 
// 	// CHECK(cudaMemcpyAsync(d_hdr_brightness_short_list_[serial_flag], d_brightness_short_map_, 1 * d_image_height_*d_image_width_ * sizeof(unsigned short), cudaMemcpyDeviceToDevice));
// 	CHECK(cudaMemcpyAsync(d_hdr_brightness_list_[serial_flag], d_brightness_map_, 1 * d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));


// 	if(!load_calib_data_flag_)
// 	{
// 		return false;
// 	}
 

// 	CHECK(cudaMemcpyAsync(d_hdr_depth_map_list_[serial_flag], d_depth_map_, 1 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToDevice)); 

// 	float val  = 0;
// 	CHECK(cudaMemcpyAsync(d_hdr_bright_pixel_sum_list_[serial_flag], &val, sizeof(float), cudaMemcpyHostToDevice)); 
//  	// cuda_count_sum_pixel_16 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_short_list_[serial_flag],d_image_height_,d_image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
//   	cuda_count_sum_pixel << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_list_[serial_flag],d_image_height_,d_image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
// 	LOG(INFO)<<"cuda_copy_result_to_hdr: "<<serial_flag;
// 	return true;

// }

bool cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial)
{
	CHECK(cudaMemcpyAsync(d_hdr_brightness_list_[serial_flag], d_brightness_map_, 1 * d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));


	if(!load_calib_data_flag_)
	{
		return false;
	}
 

	CHECK(cudaMemcpyAsync(d_hdr_depth_map_list_[serial_flag], d_depth_map_, 1 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToDevice)); 

	float val  = 0;
	CHECK(cudaMemcpyAsync(d_hdr_bright_pixel_sum_list_[serial_flag], &val, sizeof(float), cudaMemcpyHostToDevice)); 
 	cuda_count_sum_pixel << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_list_[serial_flag],d_image_height_,d_image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
	LOG(INFO)<<"cuda_copy_result_to_hdr: "<<serial_flag;
	return true;
}

bool cuda_merge_hdr_data_16(int hdr_num,float* depth_map, unsigned char* brightness)
{
 

		LOG(INFO) << "sum pixels ";
		float sum_pixels_list[6];

		for (int i = 0; i < hdr_num; i++)
		{
			CHECK(cudaMemcpy(&sum_pixels_list[i], d_hdr_bright_pixel_sum_list_[i], 1 * sizeof(float), cudaMemcpyDeviceToHost));
		}

		std::vector<float> param_list;
		std::vector<int> id;
		std::vector<bool> flag_list;

		for (int i = 0; i < hdr_num; i++)
		{
			param_list.push_back(sum_pixels_list[i]);
			id.push_back(0);
			flag_list.push_back(true);
		}
		std::sort(param_list.begin(), param_list.end(), std::greater<float>());

		for (int i = 0; i < hdr_num; i++)
		{

			for (int j = 0; j < hdr_num; j++)
			{
				if (param_list[i] == sum_pixels_list[j])
				{
					if (flag_list[j])
					{
						id[i] = j;
						flag_list[j] = false;
						break;
					}
				}
			}
		}

		for (int i = 0; i < hdr_num; i++)
		{
			LOG(INFO) << "sum pixels " << i << ": " << sum_pixels_list[i] << " _ " << id[i];
		}

		switch (hdr_num)
		{
		case 1:
		{

			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_list_[0], 1 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[0], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		}
		break;
		case 2:
		{
			cuda_merge_hdr_2<<<blocksPerGrid, threadsPerBlock>>>(d_hdr_depth_map_list_[id[0]], d_hdr_depth_map_list_[id[1]], d_hdr_brightness_list_[id[0]],
																 d_hdr_brightness_list_[id[1]], h_image_height_, h_image_width_, d_depth_map_, d_brightness_map_);

			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		}
		break;
		case 3:
		{
			cuda_merge_hdr_3<<<blocksPerGrid, threadsPerBlock>>>(d_hdr_depth_map_list_[id[0]], d_hdr_depth_map_list_[id[1]], d_hdr_depth_map_list_[id[2]], d_hdr_brightness_list_[id[0]],
																 d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], h_image_height_, h_image_width_, d_depth_map_, d_brightness_map_);

			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		}
		break;
		case 4:
		{
			cuda_merge_hdr_4<<<blocksPerGrid, threadsPerBlock>>>(d_hdr_depth_map_list_[id[0]], d_hdr_depth_map_list_[id[1]], d_hdr_depth_map_list_[id[2]], d_hdr_depth_map_list_[id[3]],
																 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]],
																 h_image_height_, h_image_width_, d_depth_map_, d_brightness_map_);

			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		}
		break;
		case 5:
		{
			cuda_merge_hdr_5<<<blocksPerGrid, threadsPerBlock>>>(d_hdr_depth_map_list_[id[0]], d_hdr_depth_map_list_[id[1]], d_hdr_depth_map_list_[id[2]],
																 d_hdr_depth_map_list_[id[3]], d_hdr_depth_map_list_[id[4]],
																 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]],
																 h_image_height_, h_image_width_, d_depth_map_, d_brightness_map_);

			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		}
		break;
		case 6:
		{
			cuda_merge_hdr_6<<<blocksPerGrid, threadsPerBlock>>>(d_hdr_depth_map_list_[id[0]], d_hdr_depth_map_list_[id[1]], d_hdr_depth_map_list_[id[2]],
																 d_hdr_depth_map_list_[id[3]], d_hdr_depth_map_list_[id[4]], d_hdr_depth_map_list_[id[5]],
																 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]],
																 d_hdr_brightness_list_[id[5]],
																 h_image_height_, h_image_width_, d_depth_map_, d_brightness_map_);

			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		}
		break;

		default:
			return false;
		}

		// kernel_merge_brigntness_map<<<blocksPerGrid, threadsPerBlock>>>(d_hdr_brightness_short_list_[hdr_num - 1], 16,
		// 																h_image_height_, h_image_width_, d_brightness_map_);


		CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[hdr_num - 1], 1*h_image_height_ * h_image_width_  * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		// CHECK(cudaMemcpy(brightness, d_brightness_map_, 1 * h_image_height_ * h_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		LOG(INFO) << "DHR Finished!";

		// cv::Mat depth(d_image_height_,d_image_width_,CV_32FC1,cv::Scalar(0));
		// CHECK(cudaMemcpy(depth.data, d_depth_map_, 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
		// cv::imwrite("depth.tiff", depth);

		// cv::Mat brightness_mat(d_image_height_,d_image_width_,CV_8U,brightness);
		// // CHECK(cudaMemcpy(brightness_mat.data, brightness, 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
		// cv::imwrite("brightness.tiff", brightness_mat);

		return true;
	
}

bool cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness)
{
	
	LOG(INFO)<<"sum pixels ";
	float sum_pixels_list[6];  

    for(int i= 0;i<hdr_num;i++)
    { 
		CHECK(cudaMemcpy(&sum_pixels_list[i], d_hdr_bright_pixel_sum_list_[i], 1* sizeof(float), cudaMemcpyDeviceToHost));
    }
 
 
	std::vector<float> param_list;
	std::vector<int> id; 
	std::vector<bool> flag_list;

	for (int i = 0; i < hdr_num; i++)
	{ 
        param_list.push_back(sum_pixels_list[i]);
		id.push_back(0);
		flag_list.push_back(true);
    } 
   	std::sort(param_list.begin(),param_list.end(),std::greater<float>());
 
 
	for (int i = 0; i < hdr_num; i++)
	{ 
		
		for(int j= 0;j< hdr_num;j++)
		{
			if(param_list[i] == sum_pixels_list[j])
			{
				if(flag_list[j])
				{ 
					id[i] = j;
					flag_list[j] = false; 
					break;
				}
			}
		}
		 
    } 

 
	for (int i = 0; i < hdr_num; i++)
	{ 
        LOG(INFO)<<"sum pixels "<<i<<": "<<sum_pixels_list[i]<<" _ "<<id[i];
    }
 

	switch(hdr_num)
	{
		case 1:
		{

			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_list_[0], 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[0], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		} 
		break;
		case 2:
		{
			cuda_merge_hdr_2 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);

				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 3:
		{
			cuda_merge_hdr_3 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 4:
		{
			cuda_merge_hdr_4 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],d_hdr_depth_map_list_[id[3]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], 
				h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 5:
		{
			cuda_merge_hdr_5 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 6:
		{
			cuda_merge_hdr_6 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],d_hdr_depth_map_list_[id[5]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				 d_hdr_brightness_list_[id[5]], 
				h_image_height_, h_image_width_, d_depth_map_,d_brightness_map_);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;

		default:
		 		return false;

	}

 	// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[id[0]], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
 	CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[hdr_num-1], 1*h_image_height_*h_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
	LOG(INFO)<<"DHR Finished!";

	return true;
}
/********************************************************************************************************************************************/

bool cuda_copy_repetition_pattern_to_memory(unsigned char* patterns_ptr,int serial_flag)
{
	CHECK(cudaMemcpyAsync(d_repetition_patterns_list_[serial_flag], patterns_ptr, h_image_height_*h_image_width_* sizeof(unsigned char), cudaMemcpyHostToDevice));
}

bool cuda_merge_repetition_patterns(int repetition_serial)
{

	int merge_serial = repetition_serial%6; 
	kernel_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_patterns_list_[repetition_serial],h_image_height_, h_image_width_,d_repetition_merge_patterns_list_[merge_serial]);

	return true;
}


bool cuda_compute_merge_phase(int repetition_count)
{

	kernel_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_merge_patterns_list_[0], d_repetition_merge_patterns_list_[1],
		d_repetition_merge_patterns_list_[2],d_repetition_merge_patterns_list_[3],d_repetition_merge_patterns_list_[4],d_repetition_merge_patterns_list_[5] ,
		repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{

		kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,d_repetition_merge_patterns_list_[0], d_repetition_merge_patterns_list_[1],
		d_repetition_merge_patterns_list_[2],d_repetition_merge_patterns_list_[3],d_repetition_merge_patterns_list_[4],d_repetition_merge_patterns_list_[5] ,
		repetition_count, cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
	}
 

	return true;
}


bool cuda_clear_repetition_02_patterns()
{
	for(int i = 0;i< D_REPETITION_02_MAX_NUM;i++)
	{
				cudaMemset(d_repetition_02_merge_patterns_list_[i], 0, h_image_height_ * h_image_width_ * sizeof(ushort));
				// CHECK(cudaMemcpyAsync(d_repetition_02_merge_patterns_list_[i], &val,image_width_* image_height_*sizeof(ushort), cudaMemcpyHostToDevice));
	}
	cudaMemset(d_merge_brightness_map_, 0, h_image_height_ * h_image_width_ * sizeof(ushort));
	cudaMemset(d_brightness_short_map_, 0, h_image_height_ * h_image_width_ * sizeof(ushort));
 
	// cudaDeviceSynchronize();
  
  return true;
}

bool cuda_merge_repetition_02_patterns(int repetition_serial)
{
	if(0 == repetition_serial)
	{
		kernel_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_brightness_map_,
		h_image_height_, h_image_width_,d_merge_brightness_map_);
	}

 
	// int merge_serial = repetition_serial%19; 
	kernel_merge_pattern<< <blocksPerGrid, threadsPerBlock >> >(d_patterns_list_[repetition_serial],h_image_height_, h_image_width_,d_repetition_02_merge_patterns_list_[repetition_serial]);

	return true;
}


bool cuda_merge_repetition_02_patterns_16(unsigned short * const d_in_pattern,int repetition_serial)
{


	if (0 == repetition_serial)
	{
		
		CHECK(cudaMemcpyAsync(d_brightness_short_map_, d_in_pattern, d_image_height_*d_image_width_* sizeof(unsigned short), cudaMemcpyHostToDevice)); 

		kernel_merge_pattern_16<<<blocksPerGrid, threadsPerBlock>>>(d_brightness_short_map_,
																 h_image_height_, h_image_width_, d_merge_brightness_map_);
	}

	cv::Mat smooth_mat(d_image_height_, d_image_width_, CV_16UC1, d_in_pattern);
	if (7 < repetition_serial || 2 > repetition_serial)
	{
		LOG(INFO) << "Start GaussianBlur:";
		cv::GaussianBlur(smooth_mat, smooth_mat, cv::Size(5, 5), 1, 1);
		LOG(INFO) << "finished GaussianBlur!";
	}

	CHECK(cudaMemcpyAsync(d_brightness_short_map_, smooth_mat.data, d_image_height_*d_image_width_* sizeof(unsigned short), cudaMemcpyHostToDevice)); 
 
	// int merge_serial = repetition_serial%19;
	kernel_merge_pattern_16<<<blocksPerGrid, threadsPerBlock>>>(d_brightness_short_map_, h_image_height_, h_image_width_, 
	d_repetition_02_merge_patterns_list_[repetition_serial]);

	return true;
}

bool cuda_compute_merge_repetition_02_phase(int repetition_count,int phase_num)
{
	
	kernel_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],
		d_repetition_02_merge_patterns_list_[2],d_repetition_02_merge_patterns_list_[3],repetition_count, h_image_height_, h_image_width_,d_wrap_map_list_[0], d_confidence_map_list_[0]);
			
	kernel_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5],
		d_repetition_02_merge_patterns_list_[6],d_repetition_02_merge_patterns_list_[7],repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[1], d_confidence_map_list_[1]);

	kernel_merge_four_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[8], d_repetition_02_merge_patterns_list_[9],
		d_repetition_02_merge_patterns_list_[10],d_repetition_02_merge_patterns_list_[11],repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[2], d_confidence_map_list_[2]);
	
	kernel_merge_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_repetition_02_merge_patterns_list_[12], d_repetition_02_merge_patterns_list_[13],
		d_repetition_02_merge_patterns_list_[14],d_repetition_02_merge_patterns_list_[15],d_repetition_02_merge_patterns_list_[16],d_repetition_02_merge_patterns_list_[17] ,
		repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{

		kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_repetition_02_merge_patterns_list_[12], d_repetition_02_merge_patterns_list_[13],
		d_repetition_02_merge_patterns_list_[14],d_repetition_02_merge_patterns_list_[15],d_repetition_02_merge_patterns_list_[16],d_repetition_02_merge_patterns_list_[17],
																			   repetition_count, cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
	}

	if(1 == phase_num)
	{
		kernel_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_02_merge_patterns_list_[18],repetition_count,h_image_height_, h_image_width_,d_brightness_map_);
	}
	else if (2 == phase_num)
	{

		int i = 18;
		kernel_merge_four_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			   d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], repetition_count, h_image_height_, h_image_width_,d_wrap_map_list_[4], d_confidence_map_list_[4]);

		i = 22;
		kernel_merge_four_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			   d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[5], d_confidence_map_list_[5]);

		i = 26;
		kernel_merge_four_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			   d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], repetition_count,h_image_height_, h_image_width_, d_wrap_map_list_[6], d_confidence_map_list_[6]);

		i = 30;
		kernel_merge_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			  d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], d_repetition_02_merge_patterns_list_[i + 4], d_repetition_02_merge_patterns_list_[i + 5],
																			  repetition_count, h_image_height_, h_image_width_, d_wrap_map_list_[7], d_confidence_map_list_[7]);

		if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
		{

			kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_repetition_02_merge_patterns_list_[i + 0], d_repetition_02_merge_patterns_list_[i + 1],
																			  d_repetition_02_merge_patterns_list_[i + 2], d_repetition_02_merge_patterns_list_[i + 3], d_repetition_02_merge_patterns_list_[i + 4], d_repetition_02_merge_patterns_list_[i + 5],
																			  repetition_count,cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
		}

		kernel_merge_brigntness_map<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[36], repetition_count, h_image_height_, h_image_width_,d_brightness_map_);

		 
	}

	return true;
}

/********************************************************************************************************************************************/
//filter
void cuda_remove_points_base_radius_filter(float dot_spacing,float radius,int threshold_num)
{

	// cv::Mat pointcloud(1200, 1920, CV_32FC3, cv::Scalar(0));
	// CHECK(cudaMemcpy(pointcloud.data, d_point_cloud_map_, 3 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// std::vector<cv::Mat> channels;
	// cv::split(pointcloud, channels);
	// cv::imwrite("depth_f.tiff", channels[2]);

	// cudaDeviceSynchronize();
	LOG(INFO)<<"kernel_reconstruct_pointcloud_base_depth:"; 
	kernel_reconstruct_pointcloud_base_depth << <blocksPerGrid, threadsPerBlock >> > (h_image_width_,h_image_height_,d_undistort_map_x_,d_undistort_map_y_,
	d_camera_intrinsic_,d_camera_distortion_,d_depth_map_,d_point_cloud_map_);

	// cudaDeviceSynchronize();

	// CHECK(cudaMemcpy(pointcloud.data, d_point_cloud_map_, 3 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// channels.clear();
	// cv::split(pointcloud, channels);
	// cv::imwrite("depth_e.tiff", channels[2]);
// cudaDeviceSynchronize();
	// LOG(INFO) << "remove_base_radius_filter start:";

	// //相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200）

	float d2 = dot_spacing * dot_spacing;
	float r2 = radius * radius;

	// cudaFuncSetCacheConfig (cuda_filter_radius_outlier_removal, cudaFuncCachePreferL1);

	// kernel_filter_radius_outlier_removal<<<blocksPerGrid, threadsPerBlock>>>(h_image_height_, h_image_width_, d_point_cloud_map_, d_mask_map_, d2, r2, threshold_num);
	// cudaDeviceSynchronize();
	// LOG(INFO)<<"kernel_filter_radius_outlier_removal finished!";
	dim3 dimBlock(BLOCK_WIDTH, BLOCK_WIDTH);
    dim3 dimGrid((d_image_width_ - 1) / O_TILE_WIDTH + 1, (d_image_height_ - 1) / O_TILE_WIDTH + 1, 1);
    kernel_filter_radius_outlier_removal_shared << <dimGrid, dimBlock >> > (d_image_height_, d_image_width_, d_point_cloud_map_, d_mask_map_, d2, r2, threshold_num);
    // cudaDeviceSynchronize();
	// LOG(INFO)<<"kernel_filter_radius_outlier_removal_shared finished!";

	// LOG(INFO) << "remove start:";
	kernel_removal_points_base_mask<<<blocksPerGrid, threadsPerBlock>>>(h_image_height_, h_image_width_, d_point_cloud_map_, d_depth_map_, d_mask_map_);

	// cudaDeviceSynchronize();

	// LOG(INFO)<<"removal finished!";
}


void cuda_filter_reflect_noise(float * const unwrap_map)
{
    // dim3 threadsPerBlock_p(img_width);
    // dim3 blocksPerGrid_p(img_height);

	//按行来组织线程
    dim3 threadsPerBlock_p(4, 4);
    // dim3 blocksPerGrid_p(15,2);
    dim3 blocksPerGrid_p;
	if(1200 == h_image_height_)
	{
		blocksPerGrid_p.x = (40 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (30 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if(2048 == h_image_height_)
	{
		blocksPerGrid_p.x = (64 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (32 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}

 
 	kernel_filter_reflect_noise << <blocksPerGrid_p, threadsPerBlock_p >> > ( h_image_height_,h_image_width_, unwrap_map);
}


void fisher_filter(float fisher_confidence_val)
{
	//按行来组织线程
    dim3 threadsPerBlock_p(32, 1);
    dim3 blocksPerGrid_p;
	if(1200 == h_image_height_)//1920
	{
		blocksPerGrid_p.x = 1;
		blocksPerGrid_p.y = 1200;
	}
	else if(2048 == h_image_height_)//2448
	{
		blocksPerGrid_p.x = 1;
		blocksPerGrid_p.y = 2048;
	}
	cudaDeviceSynchronize();
	LOG(INFO)<<"fisher start"; 
	kernel_fisher_filter <<< blocksPerGrid_p, threadsPerBlock_p >>> (h_image_height_, h_image_width_, (FISHER_CENTER_LOW + (fisher_confidence_val * FISHER_CENTER_RATE)), d_fisher_confidence_map, d_fisher_mask_, d_unwrap_map_list_[0]);
	cudaDeviceSynchronize();
	LOG(INFO)<<"fisher end"; 
}

void phase_monotonicity_filter(float monotonicity_val)
{
	// 传入的monotonicity_val应当在（-10， 2）之间，-10 - monotonicity_val之间的被认为是噪声，传入参数是0 - 100之间的数字
	monotonicity_val = monotonicity_val / 100. - 0.5;
	//按照每个像素均独立的思想来组织线程
	cudaDeviceSynchronize();
	LOG(INFO)<<"monotonicity start";
	kernel_monotonicity_filter <<< blocksPerGrid, threadsPerBlock >>> (h_image_height_, h_image_width_, -10, monotonicity_val, d_fisher_mask_, d_unwrap_map_list_[0]);
	cudaDeviceSynchronize();
	kernel_removal_phase_base_mask <<< blocksPerGrid, threadsPerBlock >>> (h_image_height_, h_image_width_, d_unwrap_map_list_[0], d_fisher_mask_);
	cudaDeviceSynchronize();
	LOG(INFO)<<"monotonicity end";
}

void depth_filter(float depth_threshold_val)
{
	dim3 threadsPerBlock_p(4, 4);
    dim3 blocksPerGrid_p;
	if(1200 == h_image_height_)
	{
		blocksPerGrid_p.x = (40 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (30 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if(2048 == h_image_height_)
	{
		blocksPerGrid_p.x = (64 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (32 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	LOG(INFO)<<"depth filter start"; 
	kernel_depth_filter_step_1 <<< blocksPerGrid_p, threadsPerBlock_p >>> (h_image_height_, h_image_width_, depth_threshold_val, d_depth_map_, d_depth_map_temp_, d_fisher_mask_);//
	cudaDeviceSynchronize();
	kernel_depth_filter_step_2 <<< blocksPerGrid_p, threadsPerBlock_p >>> (h_image_height_, h_image_width_, depth_threshold_val, d_depth_map_, d_depth_map_temp_, d_fisher_mask_);
	cudaDeviceSynchronize();
	LOG(INFO)<<"depth filter end"; 
}

/****************************************************************************************************************************/
int cuda_copy_minsw8_pattern_to_memory_16(unsigned short* pattern_ptr,int serial_flag)
{
	if(serial_flag> 16)
	{
		return -1;
	}
 
	if(0 == serial_flag)
	{

		CHECK(cudaMemcpyAsync(d_brightness_short_map_, pattern_ptr, d_image_height_*d_image_width_* sizeof(unsigned short), cudaMemcpyHostToDevice)); 

		kernel_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_brightness_short_map_,
							16,h_image_height_, h_image_width_,d_brightness_map_);
	}


	cv::Mat smooth_mat(d_image_height_, d_image_width_, CV_16UC1, pattern_ptr);
	if (7< serial_flag || serial_flag < 2)
	{
		LOG(INFO) << "Start GaussianBlur:";
		cv::GaussianBlur(smooth_mat, smooth_mat, cv::Size(5, 5), 1, 1);

		LOG(INFO) << "finished GaussianBlur!";
	}


	LOG(INFO) << "start copy:";
	// CHECK(cudaMemcpyAsync(d_repetition_02_merge_patterns_list_[serial_flag], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_repetition_02_merge_patterns_list_[serial_flag], smooth_mat.data, 
	d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice));
	LOG(INFO) << "copy finished!";
}


/****************************************************************************************************************************/
int cuda_copy_minsw8_pattern_to_memory(unsigned char* pattern_ptr,int serial_flag)
{
	if(serial_flag> 16)
	{
		return -1;
	}

	cv::Mat smooth_mat(d_image_height_, d_image_width_, CV_8UC1, pattern_ptr);
	if (7 < serial_flag || 2> serial_flag)
	{
		LOG(INFO) << "Start GaussianBlur:";
		cv::GaussianBlur(smooth_mat, smooth_mat, cv::Size(5, 5), 1, 1); 
		LOG(INFO) << "finished GaussianBlur!";
	}
	LOG(INFO) << "start copy:";
	CHECK(cudaMemcpyAsync(d_patterns_list_[serial_flag], smooth_mat.data, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
	LOG(INFO) << "copy finished!";
}


int cuda_handle_model06_16()
{
	
	kernel_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_repetition_02_merge_patterns_list_[0],16,
	h_image_height_, h_image_width_,d_brightness_map_);
	 

    kernel_generate_merge_threshold_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,
	d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],
	d_repetition_02_merge_patterns_list_[ThresholdMapSeries]);

	// cv::Mat threshold_map(d_image_height_,d_image_width_,CV_16F,cv::Scalar(0));
	// CHECK(cudaMemcpy(threshold_map.data, d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
	//  1 * d_image_height_ * d_image_width_ * sizeof(ushort), cudaMemcpyDeviceToHost));
	// cv::imwrite("threshold_map.tiff", threshold_map);
 
	// 六步相移
	kernel_merge_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																			  d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																			  16, h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{

		kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																			  d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																			  16, cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
	}

	for(int i= 8;i<16;i++)
	{
		kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
																			d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
																			i - 8, d_patterns_list_[Minsw8MapSeries]);

		// if (0 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
		// {
		// 	kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																		d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																		i - 8, d_patterns_list_[Minsw8MapSeries]);
		// }
		// else
		// {

		// 	kernel_threshold_merge_patterns_with_uncertain<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																					   d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																					   i - 8, d_patterns_list_[Minsw8MapSeries], d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
		// }
	}

	kernel_minsw8_to_bin<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
	 d_minsw8_table_, d_patterns_list_[Minsw8MapSeries], d_patterns_list_[binMapSeries]);

	kernel_bin_unwrap<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[binMapSeries], 
	d_wrap_map_list_[3], d_unwrap_map_list_[0]);


}


int cuda_handle_repetition_model06_16(int repetition_count)
{
	 
	kernel_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_merge_brightness_map_,
	repetition_count*16,h_image_height_, h_image_width_,d_brightness_map_);

	// cv::Mat brigntness_map(d_image_height_,d_image_width_,CV_8UC1,cv::Scalar(0));
	// CHECK(cudaMemcpy(brigntness_map.data, d_brightness_map_, 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
	// cv::imwrite("brigntness_map.bmp", brigntness_map);

    kernel_generate_merge_threshold_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,
	d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],d_repetition_02_merge_patterns_list_[ThresholdMapSeries]);
 
	// 六步相移
	kernel_merge_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																			  d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																			  repetition_count, h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	// cv::Mat wrap_map(d_image_height_,d_image_width_,CV_32FC1,cv::Scalar(0));
	// CHECK(cudaMemcpy(wrap_map.data, d_wrap_map_list_[3], 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// cv::imwrite("wrap_map.tiff", wrap_map);

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{

		kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																			   d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																			   repetition_count, cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_,d_uncertain_map_);
	}

	//相位校正
				//相位校正
	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify)
	{
		cv::Mat convolution_kernal = cv::getGaussianKernel(cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r,
														   cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma * 0.02, CV_32F);
		convolution_kernal = convolution_kernal * convolution_kernal.t();
		cuda_copy_convolution_kernal_to_memory((float *)convolution_kernal.data,
											   cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
		cuda_rectify_six_step_pattern_phase(3, cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
	}
 
	for(int i= 8;i<16;i++)
	{
		kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
																			d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
																			i - 8, d_patterns_list_[Minsw8MapSeries]);

		// if (0 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
		// {
		// 	kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																		d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																		i - 8, d_patterns_list_[Minsw8MapSeries]);
		// }
		// else
		// {

		// 	kernel_threshold_merge_patterns_with_uncertain<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																					   d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																					   i - 8, d_patterns_list_[Minsw8MapSeries], d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
		// }
	}

	kernel_minsw8_to_bin<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_minsw8_table_, d_patterns_list_[Minsw8MapSeries], d_patterns_list_[binMapSeries]);

	kernel_bin_unwrap<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[binMapSeries], d_wrap_map_list_[3], d_unwrap_map_list_[0]);


// cudaDeviceSynchronize();
	return DF_SUCCESS;
}


int cuda_handle_repetition_model06(int repetition_count)
{
 
	kernel_merge_brigntness_map<< <blocksPerGrid, threadsPerBlock >> >(d_merge_brightness_map_,
	repetition_count,h_image_height_, h_image_width_,d_brightness_map_);
	 

    kernel_generate_merge_threshold_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,
	d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],d_repetition_02_merge_patterns_list_[ThresholdMapSeries]);

	// 六步相移
	kernel_merge_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																		  d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																		  repetition_count, h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
	{

		kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																		  d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																		repetition_count, cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_,d_uncertain_map_);
	}

	//相位校正
				//相位校正
	if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify)
	{
		cv::Mat convolution_kernal = cv::getGaussianKernel(cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r,
														   cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma * 0.02, CV_32F);
		convolution_kernal = convolution_kernal * convolution_kernal.t();
		cuda_copy_convolution_kernal_to_memory((float *)convolution_kernal.data,
											   cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
		cuda_rectify_six_step_pattern_phase(2, cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
	}

	for (int i = 8; i < 16; i++)
	{
		kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
																			d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
																			i - 8, d_patterns_list_[Minsw8MapSeries]);

		// if (0 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
		// {
		// 	kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																		d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																		i - 8, d_patterns_list_[Minsw8MapSeries]);
		// }
		// else
		// {

		// 	kernel_threshold_merge_patterns_with_uncertain<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																		d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																		i - 8, d_patterns_list_[Minsw8MapSeries], d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
		// }
	}

	kernel_minsw8_to_bin<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_minsw8_table_, d_patterns_list_[Minsw8MapSeries], d_patterns_list_[binMapSeries]);

	kernel_bin_unwrap<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[binMapSeries], d_wrap_map_list_[3], d_unwrap_map_list_[0]);

	return 0;
}

/**********************************************************************************************************************/


int cuda_handle_minsw8_16(int flag)
{
	
    dim3 threadsPerBlock(8, 8);
    dim3 blocksPerGrid((d_image_width_ + threadsPerBlock.x - 1) / threadsPerBlock.x,
    (d_image_height_ + threadsPerBlock.y - 1) / threadsPerBlock.y);

	switch(flag)
	{
		case 2:
		{ 
            //生成阈值图
            //  kernel_generate_threshold_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[0], d_patterns_list_[1],d_patterns_list_[ThresholdMapSeries]);
			kernel_generate_merge_threshold_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,
			d_repetition_02_merge_patterns_list_[0], d_repetition_02_merge_patterns_list_[1],
			d_repetition_02_merge_patterns_list_[ThresholdMapSeries]);
			 
        } 
    		break;
        case 8:
		{ 

				//六步相移
				// int i= 2; 
				// kernel_six_step_phase_shift << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0],
				// d_patterns_list_[i + 1], d_patterns_list_[i + 2],d_patterns_list_[i + 3],d_patterns_list_[i + 4],d_patterns_list_[i + 5]
				// ,d_wrap_map_list_[3], d_confidence_map_list_[3]);

				kernel_merge_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																					  d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																					  16, h_image_height_, h_image_width_, d_wrap_map_list_[3], d_confidence_map_list_[3]);

				if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
				{

					kernel_merge_computre_global_light<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_repetition_02_merge_patterns_list_[2], d_repetition_02_merge_patterns_list_[3],
																						   d_repetition_02_merge_patterns_list_[4], d_repetition_02_merge_patterns_list_[5], d_repetition_02_merge_patterns_list_[6], d_repetition_02_merge_patterns_list_[7],
																						   16, cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b, d_direct_light_map_, d_global_light_map_,d_uncertain_map_);
				}

			//相位校正
			if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify)
            {
                cv::Mat convolution_kernal = cv::getGaussianKernel(cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r, 
				cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma * 0.02, CV_32F);
	            convolution_kernal = convolution_kernal * convolution_kernal.t();
                cuda_copy_convolution_kernal_to_memory((float*)convolution_kernal.data, 
				cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
                cuda_rectify_six_step_pattern_phase(3, cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
 
				/*********************************************************************************************************/
            }
 
 
        } 
    		break;
 
  
		default :
			break;
	}

	if (flag > 7 && flag < 16)
	{
		// kernel_threshold_patterns << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[flag], d_patterns_list_[ThresholdMapSeries],
		// flag-8,d_patterns_list_[Minsw8MapSeries]);

		// if (0 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
		{
			kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
																				d_repetition_02_merge_patterns_list_[flag], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
																				flag - 8, d_patterns_list_[Minsw8MapSeries]);

			// kernel_threshold_merge_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
			// 																	d_repetition_02_merge_patterns_list_[i], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
			// 																	i - 8, d_patterns_list_[Minsw8MapSeries]);
		}
		// else
		// {

		// 	kernel_threshold_merge_patterns_with_uncertain<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		// 																					   d_repetition_02_merge_patterns_list_[flag], d_repetition_02_merge_patterns_list_[ThresholdMapSeries],
		// 																					   flag - 8, d_patterns_list_[Minsw8MapSeries], d_direct_light_map_, d_global_light_map_, d_uncertain_map_);
		// }
	}

	if(15 == flag)
    {
        // kernel_minsw8_to_bin << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_minsw8_table_,d_patterns_list_[Minsw8MapSeries], d_patterns_list_[binMapSeries]);

        // kernel_bin_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_patterns_list_[binMapSeries],d_wrap_map_list_[3],d_unwrap_map_list_[0]);
		
		kernel_minsw8_to_bin<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_,
		d_minsw8_table_, d_patterns_list_[Minsw8MapSeries], d_patterns_list_[binMapSeries]);

		kernel_bin_unwrap<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[binMapSeries], 
		d_wrap_map_list_[3], d_unwrap_map_list_[0]);


		// cudaDeviceSynchronize();
		// cv::Mat phase(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
		// CHECK(cudaMemcpy(phase.data, d_patterns_list_[binMapSeries], 1 * d_image_height_ * d_image_width_ * sizeof(uchar), cudaMemcpyDeviceToHost));
		// cv::imwrite("code.bmp", phase);

		// cv::Mat threshold_map(d_image_height_, d_image_width_, CV_32F, cv::Scalar(0));
		// CHECK(cudaMemcpy(threshold_map.data, d_unwrap_map_list_[0], 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
		// cv::imwrite("threshold_map.tiff", threshold_map);
	}

    return 0;
}




/*************************************************************************************************************************/

 int cuda_handle_minsw8(int flag)
 {

    dim3 threadsPerBlock(8, 8);
    dim3 blocksPerGrid((d_image_width_ + threadsPerBlock.x - 1) / threadsPerBlock.x,
    (d_image_height_ + threadsPerBlock.y - 1) / threadsPerBlock.y);

	switch(flag)
	{
		case 2:
		{ 
            //生成阈值图
             kernel_generate_threshold_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[0], d_patterns_list_[1],d_patterns_list_[ThresholdMapSeries]);
 
			 
        } 
    		break;
        case 8:
		{ 

				//六步相移
				int i= 2;

				// kernel_six_step_phase_shift_with_average<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[i + 0],
				// 																			 d_patterns_list_[i + 1], d_patterns_list_[i + 2], d_patterns_list_[i + 3], d_patterns_list_[i + 4],
				// 																			 d_patterns_list_[i + 5], d_wrap_map_list_[3], d_confidence_map_list_[3], d_patterns_list_[ThresholdMapSeries], d_brightness_map_);

				kernel_six_step_phase_shift<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[i + 0],
																				d_patterns_list_[i + 1], d_patterns_list_[i + 2], d_patterns_list_[i + 3], d_patterns_list_[i + 4], d_patterns_list_[i + 5], d_wrap_map_list_[3], d_confidence_map_list_[3]);

				if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
				{
					// kernel_six_step_phase_shift_global << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_patterns_list_[i+0],
					// d_patterns_list_[i + 1], d_patterns_list_[i + 2],d_patterns_list_[i + 3],d_patterns_list_[i + 4],d_patterns_list_[i + 5]
					// ,d_wrap_map_list_[3], d_confidence_map_list_[3],0.25,
					// d_direct_light_map_,d_global_light_map_); //cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b

					kernel_computre_global_light_with_background<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[i + 0],
																					 d_patterns_list_[i + 1], d_patterns_list_[i + 2], d_patterns_list_[i + 3], d_patterns_list_[i + 4], d_patterns_list_[i + 5],
																					d_patterns_list_[0],d_patterns_list_[1] ,cuda_system_config_settings_machine_.Instance().firwmare_param_.global_light_filter_b,
																					  d_direct_light_map_, d_global_light_map_,d_uncertain_map_);

					// cudaDeviceSynchronize();
					// cv::Mat black_map(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
					// CHECK(cudaMemcpy(black_map.data, d_patterns_list_[1], 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
					// cv::imwrite("black_map.bmp", black_map);

					// cv::Mat direct_map(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
					// CHECK(cudaMemcpy(direct_map.data, d_direct_light_map_, 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
					// cv::imwrite("direct_map.bmp", direct_map);

					// cv::Mat global_map(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
					// CHECK(cudaMemcpy(global_map.data, d_global_light_map_, 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
					// cv::imwrite("global_map.bmp", global_map);
				}

			//相位校正
			if (1 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_gray_rectify)
            {
                cv::Mat convolution_kernal = cv::getGaussianKernel(cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r, 
				cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_sigma * 0.02, CV_32F);
	            convolution_kernal = convolution_kernal * convolution_kernal.t();
                cuda_copy_convolution_kernal_to_memory((float*)convolution_kernal.data, 
				cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
                cuda_rectify_six_step_pattern_phase(2, cuda_system_config_settings_machine_.Instance().firwmare_param_.gray_rectify_r);
 
				/*********************************************************************************************************/
            }
 
 
        } 
    		break;
 
  
		default :
			break;
	}


    if(flag> 7 && flag< 16)
    {

		// if (0 == cuda_system_config_settings_machine_.Instance().firwmare_param_.use_global_light_filter)
		{
			kernel_threshold_patterns<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[flag], d_patterns_list_[ThresholdMapSeries],
																		  flag - 8, d_patterns_list_[Minsw8MapSeries]);
		}
		// else
		// {

		// 	kernel_threshold_patterns_with_uncertain<<<blocksPerGrid, threadsPerBlock>>>(d_image_width_, d_image_height_, d_patterns_list_[flag], d_patterns_list_[ThresholdMapSeries],
		// 																				 flag - 8, d_patterns_list_[Minsw8MapSeries], d_direct_light_map_, d_global_light_map_, d_uncertain_map_);

			// cudaDeviceSynchronize();

			// cv::Mat uncertain_map(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
			// CHECK(cudaMemcpy(uncertain_map.data, d_uncertain_map_, 1 * d_image_height_ * d_image_width_ * sizeof(char), cudaMemcpyDeviceToHost));
			// cv::imwrite("uncertain_map.bmp", uncertain_map);
		// }
	}

    if(15 == flag)
    {
        kernel_minsw8_to_bin << <blocksPerGrid, threadsPerBlock >> > (d_image_width_,d_image_height_,d_minsw8_table_,d_patterns_list_[Minsw8MapSeries], d_patterns_list_[binMapSeries]);

        kernel_bin_unwrap << <blocksPerGrid, threadsPerBlock >> >(d_image_width_,d_image_height_,d_patterns_list_[binMapSeries],d_wrap_map_list_[3],d_unwrap_map_list_[0]);

		// cudaDeviceSynchronize();
		// cv::Mat phase(d_image_height_, d_image_width_, CV_8U, cv::Scalar(0));
		// CHECK(cudaMemcpy(phase.data, d_patterns_list_[binMapSeries], 1 * d_image_height_ * d_image_width_ * sizeof(uchar), cudaMemcpyDeviceToHost));
		// cv::imwrite("code.bmp", phase);

		// cv::Mat threshold_map(d_image_height_, d_image_width_, CV_32F, cv::Scalar(0));
		// CHECK(cudaMemcpy(threshold_map.data, d_unwrap_map_list_[0], 1 * d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
		// cv::imwrite("threshold_map.tiff", threshold_map);
	}

    return 0;
 }

/*****************************************************************************************************************************************************/
//repetition

void cuda_copy_phase_from_cuda_memory(float* phase_x,float* phase_y)
{
	CHECK(cudaMemcpy(phase_x, d_unwrap_map_list_[0], d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
	CHECK(cudaMemcpy(phase_y, d_unwrap_map_list_[1], d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost)); 
}




/*****************************************************************************************************************************************************/









