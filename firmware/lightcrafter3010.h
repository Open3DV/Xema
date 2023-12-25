#pragma once
#include "i2c.h"
#include <stdio.h>
#include <string.h>

#define Write_Image_Freeze				0x1A
#define Write_Operating_Mode_Select		0x05
#define Read_Operating_Mode_Select		0x06
#define Write_Input_Image_Size			0x2E
#define Write_Image_Crop				0x10
#define Write_Checkerboard				0x0B
#define Write_Display_Size				0x12
#define Write_Rgb_Led_Enable			0x52
#define Write_Trigger_Out               0x92
#define Read_Trigger_Out                0x93
#define Write_Led_Current               0x54
#define Read_Led_Current                0x55
#define Write_Pattern_Order             0x98
#define Read_Pattern_Order              0x99

#define Write_Internal_Pattern_Control	0x9E
#define Write_Flash_Data_Type_Select	0xDE
#define Read_Flash_Update_Precheck		0xDD
#define Write_Flash_Erase				0xE0
#define Read_Short_Status				0xD0
#define Write_Flash_Data_Length			0xDF
#define Write_Flash_Start				0xE1
#define Write_Flash_Continue			0xE2
#define Read_Flash_Start 				0xE3
#define Read_Flash_Continue				0xE4
#define Write_Pattern_Order_Table_Entry	0x98
#define Read_Device_ID					0xD4

#define R_TABLE_NUM                     166
const float R_table[R_TABLE_NUM] = {
    4409.574,	//	-40
    4105.02	,    //	-39
    3823.541,	//	-38
    3563.24	,    //	-37
    3322.388,	//	-36
    3099.407,	//	-35
    2892.859,	//	-34
    2701.428,	//	-33
    2523.912,	//	-32
    2359.214,	//	-31
    2206.328,	//	-30
    2064.333,	//	-29
    1932.385,	//	-28
    1809.713,	//	-27
    1695.607,	//	-26
    1589.417,	//	-25
    1490.346,	//	-24
    1398.079,	//	-23
    1312.106,	//	-22
    1231.961,	//	-21
    1157.213,	//	-20
    1087.466,	//	-19
    1022.355,	//	-18
    961.545	,    //	-17
    904.727	,    //	-16
    851.614	,    //	-15
    801.942	,    //	-14
    755.47	,    //	-13
    711.972	,    //	-12
    671.24	,    //	-11
    633.083	,    //	-10
    597.27	,    //	-9
    563.699	,    //	-8
    532.217	,    //	-7
    502.681	,    //	-6
    474.96	,    //	-5
    448.964	,    //	-4
    424.543	,    //	-3
    401.592	,    //	-2
    380.014	,    //	-1
    359.719	,    //	0
    340.604	,    //	1
    322.615	,    //	2
    305.68	,    //	3
    289.731	,    //	4
    274.706	,    //	5
    260.545	,    //	6
    247.194	,    //	7
    234.602	,    //	8
    222.723	,    //	9
    211.512	,    //	10
    200.927	,    //	11
    190.931	,    //	12
    181.488	,    //	13
    172.563	,    //	14
    164.127	,    //	15
    156.145	,    //	16
    148.596	,    //	17
    141.454	,    //	18
    134.693	,    //	19
    128.293	,    //	20
    122.233	,    //	21
    116.491	,    //	22
    111.049	,    //	23
    105.891	,    //	24
    101	    ,    //	25
    96.45	,    //	26
    92.13	,    //	27
    88.025	,    //	28
    84.125	,    //	29
    80.418	,    //	30
    76.895	,    //	31
    73.543	,    //	32
    70.356	,    //	33
    67.323	,    //	34
    64.436	,    //	35
    61.689	,    //	36
    59.072	,    //	37
    56.58	,    //	38
    54.205	,    //	39
    51.942	,    //	40
    49.787	,    //	41
    47.732	,    //	42
    45.772	,    //	43
    43.903	,    //	44
    42.12	,    //	45
    40.416	,    //	46
    38.79	,    //	47
    37.237	,    //	48
    35.753	,    //	49
    34.337	,    //	50
    32.984	,    //	51
    31.692	,    //	52
    30.457	,    //	53
    29.276	,    //	54
    28.146	,    //	55
    27.066	,    //	56
    26.032	,    //	57
    25.043	,    //	58
    24.096	,    //	59
    23.189	,    //	60
    22.321	,    //	61
    21.49	,    //	62
    20.693	,    //	63
    19.93	,    //	64
    19.198	,    //	65
    18.498	,    //	66
    17.827	,    //	67
    17.184	,    //	68
    16.567	,    //	69
    15.975	,    //	70
    15.405	,    //	71
    14.859	,    //	72
    14.334	,    //	73
    13.83	,    //	74
    13.346	,    //	75
    12.883	,    //	76
    12.438	,    //	77
    12.01	,    //	78
    11.599	,    //	79
    11.203	,    //	80
    10.824	,    //	81
    10.458	,    //	82
    10.107	,    //	83
    9.769	,    //	84
    9.444	,    //	85
    9.131	,    //	86
    8.83	,    //	87
    8.54	,    //	88
    8.261	,    //	89
    7.992	,    //	90
    7.734	,    //	91
    7.486	,    //	92
    7.247	,    //	93
    7.016	,    //	94
    6.794	,    //	95
    6.579	,    //	96
    6.371	,    //	97
    6.172	,    //	98
    5.979	,    //	99
    5.793	,    //	100
    5.614	,    //	101
    5.442	,    //	102
    5.276	,    //	103
    5.115	,    //	104
    4.96	,    //	105
    4.811	,    //	106
    4.666	,    //	107
    4.527	,    //	108
    4.392	,    //	109
    4.262	,    //	110
    4.136	,    //	111
    4.015	,    //	112
    3.897	,    //	113
    3.784	,    //	114
    3.674	,    //	115
    3.568	,    //	116
    3.466	,    //	117
    3.368	,    //	118
    3.272	,    //	119
    3.18	,    //	120
    3.09	,    //	121
    3.003	,    //	122
    2.919	,    //	123
    2.837	,    //	124
    2.759	,    //	125
};

class LightCrafter3010
{
private:
	I2CDevice _device;
	I2CDevice _MCP3221;
	size_t read_with_param(char inner_addr,unsigned char param, void* buffer, size_t buffer_size);
	size_t read(char inner_addr, void* buffer, size_t buffer_size);
	size_t write(char inner_addr, void* buffer, size_t buffer_size);
	size_t read_mcp3221(void* buffer, size_t buffer_size);
	float lookup_table(float fRntc);

public:
	LightCrafter3010();

	~LightCrafter3010();

	int init(); 

    int read_patterns_sets_num(int &num);
    
    void set_trigger_out_delay(int delay_time);
		
	void start_pattern_sequence();

	void stop_pattern_sequence();
	
	void write_pattern_table(unsigned char* pattern_index, int len);
	
	int write_pattern_table(unsigned char* pattern_index, unsigned char* pattern_nums, int len,float camera_exposure);

	int write_pattern_table(unsigned char* pattern_index, unsigned char* pattern_nums,unsigned char* pattern_invert, int len,float camera_exposure);

	void pattern_mode01();
	void pattern_mode02();
	void pattern_mode03();
	int pattern_mode04();
	int pattern_mode05();
	int pattern_mode06();
	int pattern_mode08();
	
	void pattern_mode03_repetition(int repetition_count);
	int pattern_mode04_repetition(int repetition_count);

	void pattern_mode_brightness();

	void read_pattern_table(int i);
	void read_pattern_status();
	
	float get_temperature();
    int SetLedCurrent(unsigned short R, unsigned short G, unsigned short B);

	void enable_checkerboard();
	void disable_checkerboard();

	void enable_solid_field();
	void disable_solid_field();

	void set_internal_pattern_stop();
	void set_flash_data_type();
	bool set_flash_build_data_size(unsigned int data_size);
	void set_erase_flash();
	bool check_erase_flash_status();
	void set_flash_data_length(unsigned short dataLen);
	int write_data_into_the_flash(unsigned char writeFlashCmd, char *TxBuffer, unsigned short dataLen);
	void read_data_from_the_flash(unsigned char readFlashCmd, char *RxBuffer, unsigned short dataLen);
	void reload_pattern_order_table_from_flash();
	float get_projector_temperature();

	void set_camera_exposure(float exposure){
		camera_exposure_ = exposure;
	}

	void read_dmd_device_id(int& version);

    void set_camera_min_exposure(float min);

private:
	float camera_exposure_;
    float dlp_min_exposure_;
    float camera_min_exposure_;
};

