#include "encode.cuh"

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


//kernel
__global__ void kernel_four_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,float * const d_out, float * const confidence)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 1)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = CV_PI + std::atan2(a, b);
		}
 

	}
}


__global__ void kernel_merge_computre_global_light(int width,int height,unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short *  d_in_2, unsigned short * const d_in_3,
unsigned short* const d_in_4,unsigned short* const d_in_5, int repetition_count,float b,unsigned char * const direct_out,unsigned char * const global_out,unsigned char * const uncertain_out)
{

	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	
	if (idx < width && idy < height)
	{
 
		ushort max_val= 1;
		ushort min_val = 255;

		if(d_in_0[offset]> max_val)
		{
			max_val = d_in_0[offset]; 
		}
 
		if(d_in_1[offset]> max_val)
		{
			max_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]> max_val)
		{
			max_val = d_in_2[offset];
		}

		if(d_in_3[offset]> max_val)
		{
			max_val = d_in_3[offset];
		}

		if(d_in_4[offset]> max_val)
		{
			max_val = d_in_4[offset];
		}

		if(d_in_5[offset]> max_val)
		{
			max_val = d_in_5[offset];
		}

/*******************************************************************************************************************************************************/

		if(d_in_0[offset]< min_val)
		{
			min_val = d_in_0[offset];
		}
		
		if(d_in_1[offset]< min_val)
		{
			min_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]< min_val)
		{
			min_val = d_in_2[offset];
		}
		
		if(d_in_3[offset]< min_val)
		{
			min_val = d_in_3[offset];
		}
		
		if(d_in_4[offset]< min_val)
		{
			min_val = d_in_4[offset];
		}
		
		if(d_in_5[offset]< min_val)
		{
			min_val = d_in_5[offset];
		}

/****************************************************************************************************************************************************/
		float d = 0.5 + (max_val - min_val) /(repetition_count* (1 - b));

		if (d > 255)
		{
			direct_out[offset] = 255;
		}
		else
		{
			direct_out[offset] = d;
		}

		float g = 0.5 + 2 * (min_val - max_val * b) /(repetition_count* (1 - b * b));

		if (g < 0)
		{
			global_out[offset] = 0;
		}
		else
		{
			global_out[offset] = g;
		}

 		if(d< g)
		{
			uncertain_out[offset] = 32;
		}
		
/*****************************************************************************************************************************************************/

	 
	}
}

__global__ void kernel_computre_global_light_with_background(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5,unsigned char* const d_in_white,unsigned char* const d_in_black, float b,unsigned char * const direct_out,unsigned char * const global_out,unsigned char * const uncertain_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	
	if (idx < width && idy < height)
	{
 
		uchar max_val= 1;
		uchar min_val = 255;

		if(d_in_0[offset]> max_val)
		{
			max_val = d_in_0[offset]; 
		}
 
		if(d_in_1[offset]> max_val)
		{
			max_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]> max_val)
		{
			max_val = d_in_2[offset];
		}

		if(d_in_3[offset]> max_val)
		{
			max_val = d_in_3[offset];
		}

		if(d_in_4[offset]> max_val)
		{
			max_val = d_in_4[offset];
		}

		if(d_in_5[offset]> max_val)
		{
			max_val = d_in_5[offset];
		}

/*******************************************************************************************************************************************************/

		if(d_in_0[offset]< min_val)
		{
			min_val = d_in_0[offset];
		}
		
		if(d_in_1[offset]< min_val)
		{
			min_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]< min_val)
		{
			min_val = d_in_2[offset];
		}
		
		if(d_in_3[offset]< min_val)
		{
			min_val = d_in_3[offset];
		}
		
		if(d_in_4[offset]< min_val)
		{
			min_val = d_in_4[offset];
		}
		
		if(d_in_5[offset]< min_val)
		{
			min_val = d_in_5[offset];
		}


		int m = min_val - d_in_black[offset];

		if (m< 0)
		{
			m = 0;
		}

		min_val = m;

/****************************************************************************************************************************************************/
		float d = 0.5 + (max_val - min_val) / (1 - b);

		if (d > 255)
		{
			direct_out[offset] = 255;
		}
		else if(d< 0)
		{
			direct_out[offset] = 0;
		}
		else
		{
			direct_out[offset] = d;
		}

		float g = 0.5 + 2 * (min_val - max_val * b) / (1 - b * b);


		if (g < 0)
		{
			global_out[offset] = 0;
		}
		else if(g> 255)
		{
			global_out[offset] = 255;
		}
		else
		{
			global_out[offset] = g;
		}
 
		if(d< g)
		{
			uncertain_out[offset] = 32;
		}

		 
/*****************************************************************************************************************************************************/

	 
	}
}


__global__ void kernel_computre_global_light(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float b,unsigned char * const direct_out,unsigned char * const global_out,unsigned char * const uncertain_out)
{

	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	
	if (idx < width && idy < height)
	{
 
		uchar max_val= 1;
		uchar min_val = 255;

		if(d_in_0[offset]> max_val)
		{
			max_val = d_in_0[offset]; 
		}
 
		if(d_in_1[offset]> max_val)
		{
			max_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]> max_val)
		{
			max_val = d_in_2[offset];
		}

		if(d_in_3[offset]> max_val)
		{
			max_val = d_in_3[offset];
		}

		if(d_in_4[offset]> max_val)
		{
			max_val = d_in_4[offset];
		}

		if(d_in_5[offset]> max_val)
		{
			max_val = d_in_5[offset];
		}

/*******************************************************************************************************************************************************/

		if(d_in_0[offset]< min_val)
		{
			min_val = d_in_0[offset];
		}
		
		if(d_in_1[offset]< min_val)
		{
			min_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]< min_val)
		{
			min_val = d_in_2[offset];
		}
		
		if(d_in_3[offset]< min_val)
		{
			min_val = d_in_3[offset];
		}
		
		if(d_in_4[offset]< min_val)
		{
			min_val = d_in_4[offset];
		}
		
		if(d_in_5[offset]< min_val)
		{
			min_val = d_in_5[offset];
		}

/****************************************************************************************************************************************************/
		float d = 0.5 + (max_val - min_val) / (1 - b);

		if (d > 255)
		{
			direct_out[offset] = 255;
		}
		else if(d< 0)
		{
			direct_out[offset] = 0;
		}
		else
		{
			direct_out[offset] = d;
		}

		float g = 0.5 + 2 * (min_val - max_val * b) / (1 - b * b);

		if (g < 0)
		{
			global_out[offset] = 0;
		}
		else if(g> 255)
		{
			global_out[offset] = 255;
		}
		else
		{
			global_out[offset] = g;
		}
 
		if(d< g)
		{
			uncertain_out[offset] = 32;
		}

		

		
/*****************************************************************************************************************************************************/

	 
	}

}

__global__ void kernel_six_step_phase_shift_global(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence,float b,unsigned char * const direct_out,unsigned char * const global_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < width && idy < height)
	{


		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];
  

	/***************************************************************************************************************************************/
		uchar max_val= 1;
		uchar min_val = 255;

		if(d_in_0[offset]> max_val)
		{
			max_val = d_in_0[offset]; 
		}
 
		if(d_in_1[offset]> max_val)
		{
			max_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]> max_val)
		{
			max_val = d_in_2[offset];
		}

		if(d_in_3[offset]> max_val)
		{
			max_val = d_in_3[offset];
		}

		if(d_in_4[offset]> max_val)
		{
			max_val = d_in_4[offset];
		}

		if(d_in_5[offset]> max_val)
		{
			max_val = d_in_5[offset];
		}

/*******************************************************************************************************************************************************/

		if(d_in_0[offset]< min_val)
		{
			min_val = d_in_0[offset];
		}
		
		if(d_in_1[offset]< min_val)
		{
			min_val = d_in_1[offset];
		}
		
		if(d_in_2[offset]< min_val)
		{
			min_val = d_in_2[offset];
		}
		
		if(d_in_3[offset]< min_val)
		{
			min_val = d_in_3[offset];
		}
		
		if(d_in_4[offset]< min_val)
		{
			min_val = d_in_4[offset];
		}
		
		if(d_in_5[offset]< min_val)
		{
			min_val = d_in_5[offset];
		}

/****************************************************************************************************************************************************/
		float d = 0.5 + (max_val - min_val) / (1 - b);

		if (d > 255)
		{
			direct_out[offset] = 255;
		}
		else
		{
			direct_out[offset] = d;
		}

		float g = 0.5 + 2 * (min_val - max_val * b) / (1 - b * b);

		if (g < 0)
		{
			global_out[offset] = 0;
		}
		else
		{
			global_out[offset] = g;
		}

		printf("max:%f , min: %f ,d: %f , g: %f		",max_val,min_val,d,g);
		
/*****************************************************************************************************************************************************/

		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_4[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_5[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 3)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = CV_PI + std::atan2(a, b);
		}
  
		// confidence[offset] = std::sqrt(a*a + b*b);
		// d_out[offset] = DF_PI + std::atan2(a, b);
	}
}

__global__ void kernel_six_step_phase_shift_with_average(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence,unsigned char* const average,unsigned char* const brightness)
{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < width && idy < height)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];
		float r = std::sqrt(a * a + b * b);
  
		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_4[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_5[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 3)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = r;
			d_out[offset] = CV_PI + std::atan2(a, b);
		}


		float ave = (d_in_0[offset] +d_in_1[offset] +d_in_2[offset] +d_in_3[offset] +d_in_4[offset] +d_in_5[offset])/6.0;

		average[offset] = ave + 0.5;
		brightness[offset] = ave + 0.5+ r / 3.0;
  
		// confidence[offset] = std::sqrt(a*a + b*b);
		// d_out[offset] = DF_PI + std::atan2(a, b);
	}
}

__global__ void kernel_six_step_phase_shift(int width,int height,unsigned char * const d_in_0, unsigned char * const d_in_1, unsigned char * const d_in_2, unsigned char * const d_in_3,
unsigned char* const d_in_4,unsigned char* const d_in_5, float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < width && idy < height)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];
  
		int over_num = 0;
		if(d_in_0[offset]>= 255)
		{
			over_num++;
		}
		if (d_in_1[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_2[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_3[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_4[offset] >= 255)
		{
			over_num++;
		}
		if (d_in_5[offset] >= 255)
		{
			over_num++;
		}

		if(over_num> 3)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = CV_PI + std::atan2(a, b);
		}
  
		// confidence[offset] = std::sqrt(a*a + b*b);
		// d_out[offset] = DF_PI + std::atan2(a, b);
	}
} 


__global__ void kernel_unwrap_variable_phase(int width,int height,float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy * width + idx;

	if (idx < width && idy < height)
	{

		/*****************************************************************************/

		float temp = 0.5 + (rate * d_in_wrap_abs[offset] - d_in_wrap_high[offset]) / (2*CV_PI);
		int k = temp;
        
		float unwrap_value =  2*CV_PI*k + d_in_wrap_high[offset]; 
  
        float err = unwrap_value - (rate * d_in_wrap_abs[offset]);

		if(abs(err)> threshold)
		{
			d_out[offset] = -10.0; 
		}
		else
		{ 
			d_out[offset] = unwrap_value;
		}

		/******************************************************************/
	}
}


__global__ void kernel_unwrap_variable_phase_base_confidence(int width,int height,float * const d_in_wrap_abs, float * const d_in_wrap_high,float const rate,float threshold, float fisher_rate, float* const d_fisher_confidence_mask, float * const d_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	int offset = idy * width + idx;

	if (idx < width && idy < height)
	{

		/*****************************************************************************/

		float temp = 0.5 + (rate * d_in_wrap_abs[offset] - d_in_wrap_high[offset]) / (2*CV_PI);
		int k = temp;
        
		float unwrap_value =  2*CV_PI*k + d_in_wrap_high[offset]; 
  
        float err = unwrap_value - (rate * d_in_wrap_abs[offset]);

		d_fisher_confidence_mask[offset] = d_fisher_confidence_mask[offset] + (abs(err) * fisher_rate);

		if(abs(err)> threshold)
		{
			d_out[offset] = -10.0; 
		}
		else
		{ 
			d_out[offset] = unwrap_value;
		}

		/******************************************************************/
	}
}


__global__ void kernel_normalize_phase(int width,int height,float * const d_in_unwrap_map, float rate,  float * const d_out_normal_map)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	const unsigned int offset = idy*width + idx;
 
	if (idx < width && idy < height)
	{

		/*****************************************************************************/ 
		d_out_normal_map[offset] = d_in_unwrap_map[offset] /rate;  

		/******************************************************************/
	}
}


__global__ void kernel_merge_six_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,unsigned short* const d_in_4,unsigned short* const d_in_5,int repetition_count,
	uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;
	float s_0 =  0;
	float s_1 =  0.866025;
	float s_2 =  0.866025;
	float s_3 =  0;
	float s_4 =  -0.866025;
	float s_5 =  -0.866025;
	float c_0 =  1;
	float c_1 =  0.5;
	float c_2 =  -0.5;
	float c_3 =  -1;
	float c_4 =  -0.5;
	float c_5 =  0.5;
	
	if (idx < img_width && idy < img_height)
	{

		float a = c_0 *d_in_3[offset] + c_1 *d_in_4[offset] + c_2 *d_in_5[offset] + c_3* d_in_0[offset] +c_4*d_in_1[offset] + c_5*d_in_2[offset];
		float b = s_0 *d_in_3[offset] + s_1 *d_in_4[offset] + s_2 *d_in_5[offset] + s_3* d_in_0[offset] +s_4*d_in_1[offset] + s_5*d_in_2[offset];

  
		confidence[offset] = std::sqrt(a*a + b*b);
		d_out[offset] = CV_PI + std::atan2(a, b);
	}

	
}


__global__ void kernel_merge_four_step_phase_shift(unsigned short * const d_in_0, unsigned short * const d_in_1, unsigned short * const d_in_2, 
	unsigned short * const d_in_3,int repetition_count,uint32_t img_height, uint32_t img_width,float * const d_out, float * const confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	int max_pixel = 255*repetition_count;

	if (idx < img_width && idy < img_height)
	{

		float a = d_in_3[offset] - d_in_1[offset];
		float b = d_in_0[offset] - d_in_2[offset];

		int over_num = 0;
		if(d_in_0[offset]>= max_pixel)
		{
			over_num++;
		}
		if (d_in_1[offset] >= max_pixel)
		{
			over_num++;
		}
		if (d_in_2[offset] >= max_pixel)
		{
			over_num++;
		}
		if (d_in_3[offset] >= max_pixel)
		{
			over_num++;
		}

		if(over_num> 1)
		{
			confidence[offset] = 0;
			d_out[offset] = -1;
		}
		else
		{
			confidence[offset] = std::sqrt(a*a + b*b);
			d_out[offset] = CV_PI + std::atan2(a, b);
		}
  
	}
}

// 函数功能：实现卷积的核函数
__global__ void kernal_convolution_2D(int width,int height, unsigned char *input, unsigned char *output, float *mask, int masksize) 
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;
	if (idy < height && idy > masksize / 2 - 1 && idx > masksize / 2 - 1 && idx < width) 
	{
		float pixVal = 0;
		//start
		int startCol = idx - masksize / 2;
		int startRow = idy - masksize / 2;
		//caculate the res
		for (int i = 0; i < masksize; i++)
		{
			for (int j = 0; j < masksize; j++)
			{
				int curRow = startRow + i;
				int curCol = startCol + j;
				if (curRow > -1 && curRow<height && curCol>-1 && curCol < width)
				{
					pixVal += mask[i*masksize + j] * input[curRow*width + curCol];
				}
			}
		}
		output[offset] = pixVal;
		
	}
}

__global__ void kernal_convolution_2D_short(int width,int height, unsigned short *input, unsigned short *output, float *mask, int masksize)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;
	if (idy < height && idy > masksize / 2 - 1 && idx > masksize / 2 - 1 && idx < width) 
	{
		float pixVal = 0;
		//start
		int startCol = idx - masksize / 2;
		int startRow = idy - masksize / 2;
		//caculate the res
		for (int i = 0; i < masksize; i++)
		{
			for (int j = 0; j < masksize; j++)
			{
				int curRow = startRow + i;
				int curCol = startCol + j;
				if (curRow > -1 && curRow<height && curCol>-1 && curCol < width)
				{
					pixVal += mask[i*masksize + j] * input[curRow*width + curCol];
				}
			}
		}
		output[offset] = pixVal;
		
	}
}

// 函数功能：实现六步相移并且计算差值
__global__ void kernel_six_step_phase_rectify(int width,int height,float* computeWrapedPhase_good, float* computeWrapedPhase_bad, float* computeWrapedPhase_original)
{
    const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && computeWrapedPhase_bad[offset] > 0 && computeWrapedPhase_good[offset] > 0)
	{
		computeWrapedPhase_original[offset] = computeWrapedPhase_original[offset] - computeWrapedPhase_bad[offset] + computeWrapedPhase_good[offset];
		if (computeWrapedPhase_original[offset] > CV_2PI)
		{
			computeWrapedPhase_original[offset] = computeWrapedPhase_original[offset] - CV_2PI;
		}
		else if (computeWrapedPhase_original[offset] < 0)
		{
			computeWrapedPhase_original[offset] = computeWrapedPhase_original[offset] + CV_2PI;
		}
		
	}
}