#pragma once
#include "filter_module.cuh"

#include "easylogging++.h"

__global__ void kernel_filter_reflect_noise(uint32_t img_height, uint32_t img_width,float * const unwrap_map)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
  
	int offset_y = idy * blockDim.x * gridDim.x + idx;  
 
    int nr = img_height;
    int nc = img_width;

	if (offset_y < img_height)
	{ 
        //读数据

        // float* phasePtr = unwrap_map +offset_y*img_width ;

        float* phasePtr = new float[img_width]; 
        for(int c= 0;c< img_width;c++)
        {
            int offset = offset_y*img_width + c;
            //  printf("offset:%d\n", offset); 
            phasePtr[c] = unwrap_map[offset];
        }

		/*****************************************************************************/
        int flag_here_is_up = 0;//为零表示没有突变，否则表示突变的点的col
        int flag_here_is_down;// 为零表示没有下降趋势，否则表示开始下降的点
        int flag_the_up_count;// 大于10表示没有突起且稀疏的点，否则表示应当给予删除

        int flag_the_up_num = 0;

        int c_before = 0;
		flag_here_is_up = 0;
		float max = -1; 
		// unsigned char* maskPtr = myMask.ptr<unsigned char>(r);
		int count = 0; 
		int flag = 0;
		for (int c = 10; c < nc; c += 1)
		{
			if (phasePtr[c_before] == -10.)
			{
				// 初次进入 
				for (int c_temp = 0; c_temp < nc; c_temp += 1)
				{
					if (phasePtr[c_temp] != -10.)
					{
						c_before = c_temp;
						c = c_before;
						flag = 1;
						break;
					}
				}
				if (flag == 0)
				{
					c = nc;
					//std::cout << "出" << std::endl;
					continue;
				}
				//if (phasePtr[c_before] != -10)
					//std::cout << "有值" << c_before << std::endl;
			}
			if (phasePtr[c] == -10)
			{
				int flag = 0;
				for (int c_temp = c; c_temp < nc; c_temp += 1)
				{
					if (phasePtr[c_temp] != -10)
					{
						c = c_temp;
						flag = 1;
						break;
					}
				}
				if (flag == 0)
				{
					c = nc;
					continue;
				}
			}

			if (phasePtr[c] <= phasePtr[c_before] && count < 100)
			{
				if (count == 0)
				{
					flag_here_is_up = c;
				}
				// 若右边比左边小
				count += 1;
				//phasePtr[c] = -1;
				//std::cout << "f" << std::endl;
			}
			else
			{
				c_before = c;
				if (count == 0)
				{
					flag_the_up_num += 1;
					//count = 0;

					flag_here_is_up = 0;
				}
				else
				{
					int num_up = 0;
					//if (1)
					//{
					//	flag_here_is_up = c - count - 5;
					//}
					int temp_num = -1;
					for (int del = 0; del < 3; del += 1)
					{

						while (phasePtr[flag_here_is_up + temp_num] == -10)
						{
							if (flag_here_is_up + temp_num == 0)
							{
								break;
							}
							temp_num -= 1;
						}

						phasePtr[flag_here_is_up + temp_num] = -10;
					}
					float min = 0;
					int num_temp_ = 0;
					int count_temp = 0;
					for (int cc = flag_here_is_up; cc < c; cc += 1)
					{
						// 添加判断语句，使得需要使用的
						// 倒着循环过去，记录最小值，小于最小值的要保留，否则删除
						num_temp_ += 1;
						// 通过while寻找到前一个点，若更大则删除，若更小，则记录
						while (phasePtr[c - num_temp_] == -10)
						{
							num_temp_ += 1;
							cc += 1;
						}
						if (min == 0)
						{
							min = phasePtr[c - num_temp_];
							phasePtr[c - num_temp_] = -10;
							continue;
						}
						if (phasePtr[c - num_temp_] < min)
						{
							min = phasePtr[c - num_temp_];
						}
						else
						{
							phasePtr[c - num_temp_] = -10;
							// maskPtr[c - num_temp_] = 255;
						}
					}
					flag_here_is_up = 0;
					count = 0;
				}
			}

		}

 

		/*****************************************************************************/
        for(int c= 0;c< img_width;c++)
        {
            unwrap_map[offset_y*img_width + c] = phasePtr[c];
        }

        delete []phasePtr;
        /*****************************************************************************/
	}
}


__global__ void kernel_fisher_filter(uint32_t img_height, uint32_t img_width, float fisher_confidence, float * const fisher_map, unsigned char* mask_output, float * const unwrap_map)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	float unique_pixels_per_thread = (float)img_width / blockDim.x;
	unique_pixels_per_thread = unique_pixels_per_thread > (int)unique_pixels_per_thread ? (int)unique_pixels_per_thread + 1 : unique_pixels_per_thread;

	int offset = idy * img_width + idx * unique_pixels_per_thread;

	if (idy < img_height)
	{
		int data_num = (idx + 1) * unique_pixels_per_thread + 7 > img_width ? img_width - idx * unique_pixels_per_thread : unique_pixels_per_thread + 7;

        float* fisherPtr = fisher_map + offset; 
		float neighborPtr[84]; 
		float* phasePtr = unwrap_map + offset;

        for(int c= 0;c< data_num;c++)
        {
			neighborPtr[c] = 0;
        }

		int numR = 0, numC = 0;
		for (int c = 0; c < data_num - 1; c += 1)
		{
			// 在此处需要循环找到非-10的值
			while (phasePtr[c] == -10. && c < data_num - 1)
			{
				c += 1;
			}
			numC = c;
			while (phasePtr[c + 1] == -10. && c < data_num - 1)
			{
				c += 1;
			}
			numR = c + 1;
			neighborPtr[c] = phasePtr[numR] - phasePtr[numC];

			
		}
		// 非线性变换
		for (int c = 0; c < data_num - 1; c += 1)
		{
			if (neighborPtr[c] < -1)
			{
				neighborPtr[c] = -1;
			}
			if (neighborPtr[c] > 1)
			{
				neighborPtr[c] = 1;
			}
			neighborPtr[c] = neighborPtr[c] * (-0.5) + 0.5;
		}
		// 膨胀操作
		float neighbor_max = 0;
		for (int c = 0; c < data_num - 7; c += 1)
		{
			neighborPtr[c] = neighbor_max;
			neighbor_max = 0;
			for (int i = 1; i < 7; i += 1)
			{
				if (neighbor_max < neighborPtr[c + i])
				{
					neighbor_max = neighborPtr[c + i];
				}
			}
			
		}
		// 实现相加
		for (int c = 0; c < data_num; c += 1)
		{
			if (fisherPtr[c] + neighborPtr[c] * (-2.89004663e-05) < fisher_confidence)
			{
				phasePtr[c] = -10.;
			}
		}
	}
}

__global__ void kernel_monotonicity_filter(uint32_t img_height, uint32_t img_width, float monotonicity_threshold_val, float monotonicity_filter_val, unsigned char* mask_output, float * const unwrap_map)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx > 0 && idx < img_width - 1 && idy < img_height)
	{
		float before_phase = unwrap_map[offset - 1];
		float this_phase = unwrap_map[offset];
		float next_phase = unwrap_map[offset + 1];
		float diff1 = this_phase - before_phase;
		float diff2 = next_phase - this_phase;

		if (this_phase < 0.1 || (before_phase < 0.1 && next_phase <0.1))
		{
			mask_output[offset] = 255;
			return;
		}

		if (before_phase >= 0.1)
		{
			if (diff1 > monotonicity_threshold_val && diff1 < monotonicity_filter_val)
			{
				mask_output[offset] = 0;
				return;
			}
		}

		if (next_phase >= 0.1)
		{
			if (diff2 > monotonicity_threshold_val && diff2 < monotonicity_filter_val)
			{
				mask_output[offset] = 0;
				return;
			}
		}

		mask_output[offset] = 255;
		return;
	}
}

__global__ void kernel_depth_filter_step_1(uint32_t img_height, uint32_t img_width, float depth_threshold, float * const depth_map, float * const depth_map_temp, unsigned char* mask_temp)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
  
    // int offset_y = idy * 64 + idx; 
	int offset_y = idy * blockDim.x * gridDim.x + idx;  
 
    int nr = img_height;
    int nc = img_width;

	if (offset_y < img_height - 1 && offset_y > 1)
	{
        //读数据

		float* depthPtr = depth_map + (offset_y*img_width);
		float* beforeDepthPtr = depth_map + ((offset_y - 1)*img_width);
		float* nextDepthPtr = depth_map + ((offset_y + 1)*img_width);

		float* featureTemp = depth_map_temp + (offset_y*img_width);
		unsigned char * maskPtr = mask_temp + (offset_y*img_width);

		float depth_diff[9];
		for (int col = 1; col < img_width; col += 1)
		{
			maskPtr[col] = 255;
			if (depthPtr[col] <= 0)
			{
				featureTemp[col] = -1;
				continue;
			}

			// 总共是0-7八个点的计算
			depth_diff[0] = beforeDepthPtr[col - 1] > 0 ? abs(beforeDepthPtr[col - 1] - depthPtr[col]) * 2. / (beforeDepthPtr[col - 1] + depthPtr[col]) : -1;
			depth_diff[1] = beforeDepthPtr[col] > 0 ? abs(beforeDepthPtr[col] - depthPtr[col]) * 2. / (beforeDepthPtr[col] + depthPtr[col]) : -1;
			depth_diff[2] = beforeDepthPtr[col + 1] > 0 ? abs(beforeDepthPtr[col + 1] - depthPtr[col]) * 2. / (beforeDepthPtr[col + 1] + depthPtr[col]) : -1;
			depth_diff[3] = depthPtr[col - 1] > 0 ? abs(depthPtr[col - 1] - depthPtr[col]) * 2. / (depthPtr[col - 1] + depthPtr[col]) : -1;
			depth_diff[4] = depthPtr[col + 1] > 0 ? abs(depthPtr[col + 1] - depthPtr[col]) * 2. / (depthPtr[col + 1] + depthPtr[col]) : -1;
			depth_diff[5] = nextDepthPtr[col - 1] > 0 ? abs(nextDepthPtr[col - 1] - depthPtr[col]) * 2. / (nextDepthPtr[col - 1] + depthPtr[col]) : -1;
			depth_diff[6] = nextDepthPtr[col] > 0 ? abs(nextDepthPtr[col] - depthPtr[col]) * 2. / (nextDepthPtr[col] + depthPtr[col]) : -1;
			depth_diff[7] = nextDepthPtr[col + 1] > 0 ? abs(nextDepthPtr[col + 1] - depthPtr[col]) * 2. / (nextDepthPtr[col + 1] + depthPtr[col]) : -1;

			// 这个点的值等于depth的最大值
			float maxDepthDiff = -1;
			for (int i = 0; i < 8; i += 1)
			{
				if (depth_diff[i] > maxDepthDiff)
				{
					maxDepthDiff = depth_diff[i];
				}
			}
			// 孤立点直接过滤
			if (maxDepthDiff == -1)
			{
				depthPtr[col] = 0;
				continue;
			}

			featureTemp[col] = abs(maxDepthDiff);
			
			if (featureTemp[col] > depth_threshold)
			{
				maskPtr[col] = 0;
			}
		}
	}
}

__global__ void kernel_depth_filter_step_2(uint32_t img_height, uint32_t img_width, float depth_threshold, float * const depth_map, float * const depth_map_temp, unsigned char* mask_temp)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
  
    // int offset_y = idy * 64 + idx; 
	int offset_y = idy * blockDim.x * gridDim.x + idx;  
 
    int nr = img_height;
    int nc = img_width;

	if (offset_y < img_height - 2 && offset_y > 2)
	{ 
        //读数据

		unsigned char* maskPtr = mask_temp + (offset_y*img_width);

		float* depthPtr = depth_map + (offset_y*img_width);

		float* featureTempBeforePtr = depth_map_temp + ((offset_y - 1)*img_width);
		float* featureTempPtr = depth_map_temp + (offset_y*img_width);
		float* featureTempNextPtr = depth_map_temp + ((offset_y + 1)*img_width);

		float depthFeatureResult;
		float depthDiff[8];

		for (int col = 0; col < img_width; col += 1)
		{
			if (maskPtr[col] == 255)
			{
				maskPtr[col] = 0;
				continue;
			}
			// 比较相邻9个点的值，然后获取
			depthDiff[0] = featureTempBeforePtr[col - 1];
			depthDiff[1] = featureTempBeforePtr[col];
			depthDiff[2] = featureTempBeforePtr[col + 1];
			depthDiff[3] = featureTempPtr[col - 1];

			depthDiff[4] = featureTempPtr[col + 1];
			depthDiff[5] = featureTempNextPtr[col - 1];
			depthDiff[6] = featureTempNextPtr[col];
			depthDiff[7] = featureTempNextPtr[col + 1];

			float compareTemp;
			for (int i = 0; i < DEPTH_DIFF_NUM_THRESHOLD; i += 1)
			{
				for (int j = i + 1; j < 8; j += 1)
				{
					if (depthDiff[j] == -1)
					{
						continue;
					}
					if (depthDiff[i] > depthDiff[j])
					{
						compareTemp = depthDiff[i];
						depthDiff[i] = depthDiff[j];
						depthDiff[j] = compareTemp;
					}
				}
			}

			depthFeatureResult = depthDiff[DEPTH_DIFF_NUM_THRESHOLD - 1];

			if (depthFeatureResult > depth_threshold || depthFeatureResult == -1)
			{
				depthPtr[col] = 0;
			}

		}
	}
}

__global__ void kernel_filter_radius_outlier_removal_shared(uint32_t img_height, uint32_t img_width, float* const point_cloud_map,
    unsigned char* remove_mask, float dot_spacing_2, float r_2, int threshold)
{

    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

    const unsigned int serial_id = idy * img_width + idx;
  
    int tx = threadIdx.x;
    int ty = threadIdx.y;
    int row_o = blockIdx.y * O_TILE_WIDTH + ty;
    int col_o = blockIdx.x * O_TILE_WIDTH + tx;
 

    int maskwidth = O_KERNEL_WIDTH;  
    int row_i = row_o - maskwidth / 2;
    int col_i = col_o - maskwidth / 2; 
    __shared__ float Ns[BLOCK_WIDTH][BLOCK_WIDTH * 3];

    if ((row_i >= 0) && (row_i < img_height) &&
        (col_i >= 0) && (col_i < img_width))
    {
        int id = 3 * (row_i * img_width + col_i);
        Ns[ty][3 * tx + 0] = point_cloud_map[id + 0];
        Ns[ty][3 * tx + 1] = point_cloud_map[id + 1];
        Ns[ty][3 * tx + 2] = point_cloud_map[id + 2];
    }
    else
    {
        Ns[ty][3 * tx + 0] = 0.0f;
        Ns[ty][3 * tx + 1] = 0.0f;
        Ns[ty][3 * tx + 2] = 0.0f;
    }

    // int offset = row_o * img_width + col_o;
 
    if ((ty < O_TILE_WIDTH) && (tx < O_TILE_WIDTH))
    {
        __syncthreads();

        // offset = row_o * img_width + col_o; 

        int ns_ty = ty + maskwidth / 2;
        int ns_tx = tx + maskwidth / 2;

        // remove_mask[row_o * img_width + col_o] = 255;
		uchar mask_val = 255;
        int num = 0;
        float x_o = Ns[ns_ty][3 * ns_tx + 0];
        float y_o = Ns[ns_ty][3 * ns_tx + 1];
        float z_o = Ns[ns_ty][3 * ns_tx + 2];
         //if (row_o == 1024 && col_o == 1024)
         //{
         //	printf("x_o:%f\n", x_o);
         //	printf("y_o:%f\n", y_o);
         //	printf("z_o:%f\n", z_o);
         //   float x_test = point_cloud_map[3* offset + 0];
         //   float y_test = point_cloud_map[3 * offset + 1];
         //   float z_test = point_cloud_map[3 * offset + 2];
         //  printf("x_0:%f\n", x_test);
         //  printf("y_0:%f\n", y_test);
         //  printf("z_0:%f\n", z_test);
         //}

        if (z_o <= 0)
        {
            // remove_mask[row_o * img_width + col_o] = 0;
			mask_val = 0;
        }
        else
        {
  
            for (int r = -maskwidth / 2; r <= maskwidth / 2; r++)
            {
                for (int c = -maskwidth / 2; c <= maskwidth / 2; c++)
                {

                    int nx_r = ns_ty + r;
                    int nx_c = ns_tx + c;

                    if (nx_r < 0 || nx_c < 0)
                    {
                        continue;
                    }

                    if (nx_r >= BLOCK_WIDTH || nx_c >= BLOCK_WIDTH)
                    {
                        continue;
                    }

                    // float space2 = (c * c + r * r) * dot_spacing_2;
 

                    // int pos = r * img_width + c; 
                    if (Ns[nx_r][3 * nx_c + 2] > 0)
                    {
           
                        float dx = Ns[nx_r][3 * nx_c + 0] - x_o;
                        float dy = Ns[nx_r][3 * nx_c + 1] - y_o;
                        float dz = Ns[nx_r][3 * nx_c + 2] - z_o;  
     
                        float d2 = dx * dx + dy * dx + dz * dz; 

                        // if (radius > dist)
                        if (r_2 > d2)
                        {
                            num++;
                        }
                    }
                }
            }

            if (num < threshold)
            {
                // remove_mask[row_o * img_width + col_o] = 0;
				mask_val = 0;
            }
        }

		// __syncthreads();
        remove_mask[row_o * img_width + col_o] = mask_val;
    }
  


}


//滤波
__global__ void kernel_filter_radius_outlier_removal(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,unsigned char* remove_mask,
float dot_spacing_2, float r_2,int threshold)
{
 	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		/****************************************************************************/
		//定位区域
		if (point_cloud_map[3 * serial_id + 2] > 0)
		{
			remove_mask[serial_id] = 255;
			int w = 5;

			int s_r = idy - w;
			int s_c = idx - w;

			int e_r = idy + w;
			int e_c = idx + w;

			if (s_r < 0)
			{
				s_r = 0;
			}
			if (s_c < 0)
			{
				s_c = 0;
			}

			if (e_r >= img_height)
			{
				e_r = img_height - 1;
			}

			if (e_c >= img_width)
			{
				e_c = img_width - 1;
			}

			int num = 0;

			for (int r = s_r; r <= e_r; r++)
			{
				for (int c = s_c; c <= e_c; c++)
				{
					float space2 = ((idx - c) * (idx - c) + (idy - r) * (idy - r)) * dot_spacing_2;
					if (space2 > r_2)
						continue;

					int pos = r * img_width + c;
					if (point_cloud_map[3 * pos + 2] > 0)
					{  
						float dx= point_cloud_map[3 * serial_id + 0] - point_cloud_map[3 * pos + 0];
						float dy= point_cloud_map[3 * serial_id + 1] - point_cloud_map[3 * pos + 1];
						float dz= point_cloud_map[3 * serial_id + 2] - point_cloud_map[3 * pos + 2];

						float d2 = dx * dx + dy * dx + dz * dz;
						// float dist = std::sqrt(dx * dx + dy * dx + dz * dz); 
 
						// if (radius > dist)
						if (r_2 > d2)
						{
							num++;
						}
					}
				}
			} 

			if (num < threshold)
			{ 
				remove_mask[serial_id] = 0;
			} 
		}
		else
		{ 
			remove_mask[serial_id] = 0;
		}

		/******************************************************************/
	}
}

//滤波
__global__ void kernel_removal_points_base_mask(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,float* const depth_map,uchar* remove_mask)
{
  	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		if(0 == remove_mask[serial_id])
		{
			depth_map[serial_id] = 0;
			point_cloud_map[3 * serial_id + 0] = 0;
			point_cloud_map[3 * serial_id + 1] = 0;
			point_cloud_map[3 * serial_id + 2] = 0;
		}

	}

}

__global__ void kernel_removal_phase_base_mask(uint32_t img_height, uint32_t img_width, float* const unwrap_map, uchar* remove_mask)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		if(0 == remove_mask[serial_id])
		{
			unwrap_map[serial_id] = 0;
		}
	}
}
