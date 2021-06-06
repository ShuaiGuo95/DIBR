#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdafx.h"
#include "math.h"
#include "time.h"

#define I2R_OK    1
#define I2R_ERR   0 
#define CV_PI       3.14159265358979323846
//#define DEPTH_DOWNSAMPLE //decide whether use depth down sample or not. Modified in Apr, 2020
#define IS_PINHOLE 1 // decide whether use pinhole model or not 
//#define BILATERAL_FILTER  //decide whether use bilteral filter or not. Modified in Aug, 2020
//#define USE_BACKGROUND // decide whether use background or not
#define BACKGROUND_THRE 10 // the thresold of using background
float krt_K[9];
float krt_K_inv[9];

#define FILE_NAME_SIZE 200

typedef struct
{
	int  CamNum;
	int  SourceWidth;
	int  SourceHeight;
	char Color_Path[FILE_NAME_SIZE];    //color files
	char Depth_Path[FILE_NAME_SIZE];    //depth files
	char Cam_Params_File[FILE_NAME_SIZE];   //total camera params, and mindepth  maxdepth
	char Color_Output_File[FILE_NAME_SIZE];
	char Depth_Output_File[FILE_NAME_SIZE];


	//parameters input for virtual camera 
	/*
	float krt_R[9];
	float krt_WorldPosition[3];
	int ltype;
	float krt_kc[3];
	float krt_cc[2];
	int vcam_width, vcam_height;
	float lens_fov;
	float fisheye_radius; // -- in pixel unit at original resolution
	*/
	double Vcam_krt_R_0;
	double Vcam_krt_R_1;
	double Vcam_krt_R_2;
	double Vcam_krt_R_3;
	double Vcam_krt_R_4;
	double Vcam_krt_R_5;
	double Vcam_krt_R_6;
	double Vcam_krt_R_7;
	double Vcam_krt_R_8;

	double Vcam_krt_WorldPosition_0;
	double Vcam_krt_WorldPosition_1;
	double Vcam_krt_WorldPosition_2;

	double Vcam_krt_kc_0;
	double Vcam_krt_kc_1;
	double Vcam_krt_kc_2;

	double Vcam_krt_cc_0;
	double Vcam_krt_cc_1;
	double lens_fov;
	double fisheye_radius;

	int Vcam_src_width;
	int Vcam_src_height;
	int Vcam_ltype;


} InputParameters;

InputParameters configinput;
InputParameters inputs, *input = &inputs;

#define I2R_ASSERT(expression)										\
{																	\
	if(!(expression)){												\
		printf( "Assertion fail: in %s() line %i;\n",		        \
						__FUNCTION__, __LINE__);					\
		return I2R_ERR;												\
	}																\
}


typedef enum {
	I2R_LENS_FULLFRAME_UNDIST,
	I2R_LENS_FULLFRAME_FISHEYE,
	I2R_LENS_CIRCULAR_FISHEYE
} I2R_LENS_TYPE;


#define MAX_SFM_CAMNUM (40) // default 200   -- maximum support sfm image number, 2K

#define SFM_DISTORTION_COEF_NUM (3) // -- distortion coefficients number

#define MAX_KRT_PAIRWISE_NUM (MAX_SFM_CAMNUM * (MAX_SFM_CAMNUM - 1) / 2)

// -- MAX_KRT_CAMNUM * (9(rot) + 2(cc) + 3(position) + kc + 2(srcw/srch) + 2(radius/fov)) + 1(CamNum)
#define SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT (MAX_SFM_CAMNUM * (9 + 2 + 3 + 3 + 2 + 2) + 1)


#define REF_CAM_N   2    //ref cameras or frames used to interpolate



float mindepth;              // 10.0;   50.0    100.0            //resolution change  20.0000
float maxdepth;              //900.0;       2000.0   3000.0000



typedef enum {

	novel_pixel_hole,

	novel_pixel_main = 128,

	novel_pixel_foreground_boundary = 255,

	novel_pixel_background_boundary = 50

} novel_pixel_label_t;


typedef struct krt_CamParam
{
	//gs begin add 原本没有krt_K[] 用于放旋转矩阵
	float krt_K[9];
	float mindepth, maxdepth;
	float fx, fy; // 用于存放两个焦距
	//gs end add
	//float krt_k_inv[9];
	float krt_R[9];
	float krt_WorldPosition[3];
	I2R_LENS_TYPE ltype;
	float krt_kc[3];
	float krt_cc[2]; // 应该是用来存放分辨率的一半的
	int src_width, src_height;
	float lens_fov;
	float fisheye_radius; // -- in pixel unit at original resolution
}krt_CamParam;



typedef struct cuRef_data_t
{
	unsigned char  *cuTex;  //cudaTextureObject_t cuTex;
	float* range_img; // -- data allocated by caller

}cuRef_data_t;

typedef struct cuCam_data_t
{
	float* refR, *refinvR, *reft;
	float* refCC, *refKc, reffradius, reffov;       //reffradius  == fisheye_radius
	float* refr2t_curve;  //arr_rtheta  look-up table  
	int refr2tl;          //rtheta_len
						  //float* refK;
						  //float* refK_inv;
	float* vR, *vinvR, *vt;
	float* vCC, *vKc, vfradius, vfov;
	float *vr2t_curve;
	int vr2tl;

}cuCam_data_t;


typedef struct novel_view_t
{
	float* mrange_img; // -- main/foreground disparity image  视差图
	novel_pixel_label_t *mlabel_img;  // 前景边缘标记图
	unsigned char *mnovel_view;
	float* mconf;
	float prefW;

}novel_view_t;

typedef struct algo_param_t {
	int winh, winv, winstep; // -- patch size and step when computing matching cost
	float mindisp, maxdisp;
	float mindepth, maxdepth;
	float fB; // -- focal * baseline
	int *selectedViews, sv_nbr;
} algo_param_t;

// x^2 + y^2 + z^2
__inline float norm(float *sphere)
{
	return sqrtf(sphere[0] * sphere[0] + sphere[1] * sphere[1] + sphere[2] * sphere[2]);
}

// if f < a, return a
// if f >= a, then
//      if f > z, return z
//      else, return f
static __inline float clip(float f, float a, float z) {
	return (f < a) ? a : (f > z) ? z : f;
}


int  CamNum;
FILE* log_readed_parameter_file;

krt_CamParam krt_camparam[MAX_SFM_CAMNUM];


int set_sfm_parameters(float* save_data)
{
	// -- this function init sfm solver by loaded data

	fprintf(log_readed_parameter_file, "\n*****************************************************************\n");
	fprintf(log_readed_parameter_file, "---> set sfm calibration information to sfm solver\n");
	for (int i = 0; i < MAX_SFM_CAMNUM; i++)
	{
		memcpy((float*)krt_camparam[i].krt_R, save_data, 9 * sizeof(float)); // -- copy rotation matrix
		save_data += 9;
		//float cc[2] = {480.00,270.00};
		memcpy(krt_camparam[i].krt_cc, save_data, 2 * sizeof(float));
		//memcpy(krt_camparam[i].krt_cc, save_data, 2 * sizeof(float)); // -- copy principal points coord
		save_data += 2;

		memcpy(krt_camparam[i].krt_WorldPosition, save_data, 3 * sizeof(float)); // -- copy position
		save_data += 3;

		memcpy(krt_camparam[i].krt_kc, save_data, 3 * sizeof(float)); // -- copy kc
																	  //krt_camparam[i].krt_kc[1] = 0; krt_camparam[i].krt_kc[2] = 0;
		save_data += 3;

		if (i < CamNum)
		{
			int* int_save_data = (int*)save_data;
			//int_save_data[0] = 960;
			//int_save_data[1] = 540;
			if ((krt_camparam[i].src_width != int_save_data[0]) || (krt_camparam[i].src_height != int_save_data[1]))
			{
				// printf("error: sfm calibration image resolution(%d x %d) is not compatible with the loaded "
				// 	"image(%d x %d) for camera %d;\n", int_save_data[0], int_save_data[1], krt_camparam[i].src_width, krt_camparam[i].src_height, i);
				//return I2R_ERR;
			}

		}
		save_data += 2;
		//gs begin change
		krt_camparam[i].fisheye_radius = save_data[0];
		krt_camparam[i].lens_fov = save_data[1];
		//krt_camparam[i].fisheye_radius = 2000;
		//krt_camparam[i].lens_fov = 10;
		//gs end change
		save_data += 2;
	}
	int* int_save_data = (int*)save_data;
	/*if(CamNum != int_save_data[0]){
	sprintf(err_str,"error: sfm calibration data is not compatible with the loaded image;\n");
	return ISS_ERR;
	}*/
	save_data++;
	save_data -= SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT;

	//gs begin add
	FILE *fp = NULL, *tp = NULL;
	fp = fopen("para_cameras_1004_diff_noresize.txt", "r");
	tp = fopen("./depth_1004_diff_noresize/minmaxdepth.txt", "r");
	char temps[100];
	int tempid;
	for (int m = 0; m < CamNum; m++)
	{
		fscanf(fp, "%s %d\n", temps, &tempid);

		fscanf(fp, "%s %d %d\n", temps, &krt_camparam[m].src_width, &krt_camparam[m].src_height);

		for (int tempi = 0; tempi < 9; tempi++)
			krt_camparam[m].krt_K[tempi] = 0;
		fscanf(fp, "%s %f %f %f %f\n", temps, &krt_camparam[m].krt_K[0], &krt_camparam[m].krt_K[4], &krt_camparam[m].krt_K[2], &krt_camparam[m].krt_K[5]);

		krt_camparam[m].krt_cc[0] = krt_camparam[m].krt_K[2];
		krt_camparam[m].krt_cc[1] = krt_camparam[m].krt_K[5];

		fscanf(fp, "%s", temps);
		for (int tempi = 0; tempi < 9; tempi++)
			fscanf(fp, " %f", &krt_camparam[m].krt_R[tempi]);
		fgetc(fp);

		fscanf(fp, "%s %f %f %f\n", temps, &krt_camparam[m].krt_WorldPosition[0], &krt_camparam[m].krt_WorldPosition[1], &krt_camparam[m].krt_WorldPosition[2]);

		fscanf(tp, "%f %f\n", &krt_camparam[m].mindepth, &krt_camparam[m].maxdepth);
	}
	fclose(fp);
	fclose(tp);
	//gs end add

	// -- printf sfm data
	fprintf(log_readed_parameter_file, "\n--->camera parameters: \n");
	for (int m = 0; m < CamNum; m++)
	{
		fprintf(log_readed_parameter_file, "---> camera id: %d\n", m);
		fprintf(log_readed_parameter_file, "   ---> resolution: %d x %d\n", krt_camparam[m].src_width, krt_camparam[m].src_height);
		fprintf(log_readed_parameter_file, "   ---> cc: [%.2f; %.2f];\n", krt_camparam[m].krt_cc[0], krt_camparam[m].krt_cc[1]);
		//float rotvec[3][3] = {0};
		//Rodrigues(krt_camparam[m].krt_R, rotvec);   //wkj
		//gs begin add
		fprintf(log_readed_parameter_file, "   ---> internal paras: [%.6f; %.6f; %.6f; %.6f];\n", krt_camparam[m].krt_K[0], krt_camparam[m].krt_K[4], krt_camparam[m].krt_K[2], krt_camparam[m].krt_K[5]);
		//gs end add
		fprintf(log_readed_parameter_file, "   ---> rotation matrix: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f \n",
			krt_camparam[m].krt_R[0], krt_camparam[m].krt_R[1], krt_camparam[m].krt_R[2],
			krt_camparam[m].krt_R[3], krt_camparam[m].krt_R[4], krt_camparam[m].krt_R[5],
			krt_camparam[m].krt_R[6], krt_camparam[m].krt_R[7], krt_camparam[m].krt_R[8]);
		//fprintf(log_readed_parameter_file, "   ---> rotation vector: [%.2f, %.2f, %.2f]\n", rotvec[0][0], rotvec[1][0], rotvec[2][0]);
		fprintf(log_readed_parameter_file, "   ---> world position: [%.2f, %.2f, %.2f]\n", krt_camparam[m].krt_WorldPosition[0],
			krt_camparam[m].krt_WorldPosition[1], krt_camparam[m].krt_WorldPosition[2]);

		// -- init lens fov
		//compute_theta_by_radius(krt_camparam[m].krt_kc, krt_camparam[m].lens_fov, krt_camparam[m].fisheye_radius);   // wkj
		krt_camparam[m].lens_fov = (float)configinput.lens_fov;//60.60 * CV_PI / 360;  72.06* CV_PI / 360

		fprintf(log_readed_parameter_file, "   ---> lens distortion: [%.2f; %.2f; %.2f]\n", krt_camparam[m].krt_kc[0], krt_camparam[m].krt_kc[1], krt_camparam[m].krt_kc[2]);
		fprintf(log_readed_parameter_file, "   ---> lens fov: %f \n", krt_camparam[m].lens_fov);
		fprintf(log_readed_parameter_file, "   ---> fisheye radius: %.2f pixels\n", krt_camparam[m].fisheye_radius);
	}
	fprintf(log_readed_parameter_file, "---> set sfm calibration information to sfm solver done\n");
	fprintf(log_readed_parameter_file, "*****************************************************************\n");
	return I2R_OK;
}


int init_with_sfm_file(char* filename)
{
	log_readed_parameter_file = fopen("log_readed_camera_parameters", "wt+");

	fprintf(log_readed_parameter_file, "---> start to load calibration data...\n");
	if (!filename) { printf("error: can not open sfm file %s\n", filename); return I2R_ERR; }

	// -- sizeof(int) == sizeof(float)
	float load_data_array[SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT];
	float *load_data = &load_data_array[0];
	FILE* file = fopen(filename, "rb"); // 某个sfm文件
	if (file) {
		fread(&mindepth, sizeof(float), 1, file);   //mindepth and maxdepth are placed at first two float space.
		fread(&maxdepth, sizeof(float), 1, file);
		size_t read_count = fread(load_data, sizeof(float), SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT, file);
		//gs begin add 
		//mindepth = 6;
		//maxdepth = 51;
		//gs end add
		//printf("mindepth=%f\n", mindepth);
		//printf("maxdepth=%f\n", maxdepth);

		fclose(file);
		if (read_count != SFM_CALIB_FILE_SIZE_IN_FLOAT_UNIT) {
			printf("error: sfm calibration file size not compatible\n");
			return I2R_ERR;
		}
	}
	else { printf("error: can not open sfm calibration file %s\n", filename); return I2R_ERR; }

	set_sfm_parameters(load_data); // -- set calibration data with the loaded data
	fprintf(log_readed_parameter_file, "---> load calibration data done\n");

	fclose(log_readed_parameter_file);
	return I2R_OK;
}


void label_foreground_boudary_pixels(float* range_img, novel_pixel_label_t* labels, const float dthresh, const float fB, const int width, const int height, const int edge_radius)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			float cval = fB / (range_img[y * width + x] + 1e-5f);

			int rx = min(x + 1, width - 1);
			int dy = min(y + 1, height - 1);

			float dval = fB / (range_img[dy * width + x] + 1e-5f);
			float rval = fB / (range_img[y * width + rx] + 1e-5f);

			float grad_x = cval - rval;
			float grad_y = cval - dval;

			if (fabs(grad_x) >= dthresh)
			{
				int sid = grad_x > 0.f ? -edge_radius : 1;
				int eid = grad_x > 0.f ? 0 : edge_radius;
				for (int i = sid; i <= eid; i++)
				{
					int imx = max(0, min(width - 1, i + x));
					labels[y * width + imx] = novel_pixel_foreground_boundary; //如果梯度大于一定值，就把周围一定范围内的labels值赋为novel_pixel_foreground_boundary，这个阈值是深度范围乘以一个系数得到的
				}
			} // -- process horizontal gradients
			if (fabs(grad_y) >= dthresh)
			{
				int sid = grad_y > 0.f ? -edge_radius : 1;
				int eid = grad_y > 0.f ? 0 : edge_radius;
				for (int i = sid; i <= eid; i++)
				{
					int imy = max(0, min(height - 1, i + y));
					labels[imy * width + x] = novel_pixel_foreground_boundary;
				}
			} // -- process vertical gradients
		}
}

void label_background_boudary_pixels(float* range_img, novel_pixel_label_t* labels, const float dthresh, const float fB, const int width, const int height, const int edge_radius, float mindepth, float maxdepth)
{
	int x, y;
	//FILE *temp = fopen("./results/temp.txt", "a+");
	for (y = 0; y < min(height, 1000); y++) // gs add height改为min(height, 1000) 或否
		for (x = 0; x < width; x++)
		{
			if (range_img[y * width + x] > 17) continue; // gs add 背景前景边界判断 稀疏方案

			float cval = fB / (range_img[y * width + x] + 1e-5f);

			int rx = min(x + 1, width - 1);
			int dy = min(y + 1, height - 1);

			float dval = fB / (range_img[dy * width + x] + 1e-5f);
			float rval = fB / (range_img[y * width + rx] + 1e-5f);

			float grad_x = cval - rval;
			float grad_y = cval - dval;

			if (fabs(grad_x) >= dthresh)
			{
				//if (range_img[y * width + rx] >= maxdepth - 0.05 &&  range_img[y * width + x] > 17) continue; // gs add 背景前景边界判断 疑似0的位置都跳过 稠密方案
				//if (range_img[y * width + x] >= maxdepth - 0.05 &&  range_img[y * width + rx] > 17) continue; // gs add 背景前景边界判断 疑似0的位置都跳过 稠密方案
				if (range_img[y * width + rx] >= maxdepth - 0.05) continue; // 稀疏方案

				int sid = grad_x > 0.f ? 1 : -edge_radius;
				int eid = grad_x > 0.f ? edge_radius : 0;
				for (int i = sid; i <= eid; i++)//如果梯度大于一定值，就把周围一定范围内的labels值赋为novel_pixel_background_boundary
				{
					int imx = max(0, min(width - 1, i + x));
					//if (range_img[y * width + imx] >= maxdepth - 0.05) continue; // 稀疏方案
					labels[y * width + imx] = novel_pixel_background_boundary;
					//fprintf(temp, "x, %d; %d %d, %d; %f %f\n", y, x, y, imx, dthresh, fabs(grad_x));
				}
			} // -- process horizontal gradients
			if (fabs(grad_y) >= dthresh)
			{
				//if (range_img[dy * width + x] >= maxdepth - 0.05 && range_img[y * width + x] > 17) continue; // gs add 背景前景边界判断 疑似0的位置都跳过 稠密方案
				//if (range_img[y * width + x] >= maxdepth - 0.05 && range_img[dy * width + x] > 17) continue; // gs add 背景前景边界判断 疑似0的位置都跳过 稠密方案
				if (range_img[dy * width + x] >= maxdepth - 0.05) continue; // 稀疏方案

				int sid = grad_y > 0.f ? 1 : -edge_radius;
				int eid = grad_y > 0.f ? edge_radius : 0;
				for (int i = sid; i <= eid; i++)
				{
					int imy = max(0, min(height - 1, i + y));
					//if (range_img[imy * width + x] >= maxdepth - 0.05) continue; // 稀疏方案
					labels[imy * width + x] = novel_pixel_background_boundary;
					//fprintf(temp, "y, %d; %d %d, %d; %f %f\n", y, x, imy, x, dthresh, fabs(grad_y));
				}
			} // -- process vertical gradients
		}
	//fclose(temp);
}
// 如果某一点梯度的绝对值大于dthresh且为正（距离相比于右侧点来说更近），则右侧和下侧edge_radius范围内的点的label标记为novel_pixel_background_boundary；左侧和上侧edge_radius范围内的点的label标记为novel_pixel_foreground_boundary；
// 如果某一点梯度的绝对值大于dthresh且为负（距离相比于右侧点来说更远），则右侧和下侧edge_radius范围内的点的label标记为novel_pixel_foreground_boundary；左侧和上侧edge_radius范围内的点的label标记为novel_pixel_background_boundary；


void set_main_layer_labels(novel_pixel_label_t* labels, const int width, const int height, novel_pixel_label_t tarL)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{

			if ((x >= width) || (y >= height))
			{
				continue;
			}

			if (labels[y * width + x] != tarL)   //wkj
			{
				labels[y * width + x] = novel_pixel_main;
			}
			//if (labels[y * width + x] == novel_pixel_hole) { labels[y * width + x] = novel_pixel_main; }
		}
}


int label_boundary_pixels2(float* range_img, novel_pixel_label_t* labels, novel_pixel_label_t tarL,
	float fB, float dthresh, const int edge_radius, int width, int height, float mindepth, float maxdepth)
{

	//set_main_layer_labels(labels, width, height, tarL);   //wkj  the same as putting it  below  label_background_boudary_pixels

	if (tarL == novel_pixel_foreground_boundary)
	{
		label_foreground_boudary_pixels(range_img, labels, dthresh, fB, width, height, edge_radius);
	}
	else if (tarL == novel_pixel_background_boundary)
	{
		label_background_boudary_pixels(range_img, labels, dthresh, fB, width, height, edge_radius, mindepth, maxdepth);
	}

	set_main_layer_labels(labels, width, height, tarL);

	return I2R_OK;
}


//Brute-Force Search the root for  five-degree polynomial equation, we do not need to use a complex fast algorithm
float  roots_one_real(float* coeff)
{
	int i;

	float root = 0;
	float min_diff = 100.0;

	for (i = 0; i < 100000; i = i + 20)   //1000 000  have been tested, the accuracy of 1000 00 is enough 
	{
		float r = ((float)i) * 0.00001f;
		float r3 = r * r*r;
		float r5 = r * r*r3;
		float result = coeff[5] * r5 + coeff[3] * r3 + coeff[1] * r + coeff[0];
		//double result = coeff[5] * powf(r, 5) + coeff[3] * powf(r, 3) + coeff[1] * r + coeff[0];

		if (fabs(result) < min_diff)
		{
			min_diff = fabs(result);
			root = r;

		}

	}
	return root;

}



int set_r2t_array_by_kc(float* arr, float* kc, int rtheta_length, float fisheye_radius)
{
	// rtheta_length = fisheye_radius*2
#define rows 6

	float coeff[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };//cv::Mat::zeros(6, 1, CV_32FC1);
	double roots[6][2];

	coeff[1] = kc[0];
	coeff[3] = kc[1];
	coeff[5] = kc[2];

	for (int i = 0; i < rtheta_length; i++)         // rtheta_length = r2t_len = 2*fisheye_radius 
	{
		float r = (float)(i) / (float)(rtheta_length)* fisheye_radius; // i / 2 // 这个相当于在fisheye_raduis亚精度的打表
																	   //float r = (float)(i) / 2.0f;
																	   // -- solve the polynomial funciton
		coeff[0] = -r;

		arr[i] = roots_one_real(coeff); // min_theta_val;       
										//这个表中得到的是投影长度（单位为像素）与角度theta的关系，i是投影长度，arr[i]是投影角度
	}


	return I2R_OK;
}
// bilteral filter added, modified in Aug, 2020
int set_ws_array(float* arr, float drange, int radius)
{
	const float sigs = (float)(radius * 1.2f);

	for (int i = 0; i <= radius * radius * 2; i++)
	{
		float dist = sqrtf((float)(i));
		float ws = expf(-dist / sigs);
		arr[i] = ws;
	}
	return I2R_OK;
}
// bilteral filter added, modified in Aug, 2020
int set_wd_array(float* arr, int wd_array_len, const int radius, float drange)
{
	const float sigd = (float)(radius * 3.6f);
	for (int i = 0; i < wd_array_len; i++)
	{
		float r = (float)(i) / (float)(wd_array_len)* drange;
		float wd = expf(-r / sigd);
		arr[i] = wd;
	}
	return I2R_OK;
}

#define I(_i, _j) ((_j)+3*(_i))


void matrix_mult_3x3_by_3x3(float *m, float *lhs, float *rhs)
{
	// m = rhs * lhs
	int i, j;
	float t[9];

	t[0] = lhs[I(0, 0)] * rhs[I(0, 0)] + lhs[I(0, 1)] * rhs[I(1, 0)] + lhs[I(0, 2)] * rhs[I(2, 0)];
	t[1] = lhs[I(0, 0)] * rhs[I(0, 1)] + lhs[I(0, 1)] * rhs[I(1, 1)] + lhs[I(0, 2)] * rhs[I(2, 1)];
	t[2] = lhs[I(0, 0)] * rhs[I(0, 2)] + lhs[I(0, 1)] * rhs[I(1, 2)] + lhs[I(0, 2)] * rhs[I(2, 2)];

	t[3] = lhs[I(1, 0)] * rhs[I(0, 0)] + lhs[I(1, 1)] * rhs[I(1, 0)] + lhs[I(1, 2)] * rhs[I(2, 0)];
	t[4] = lhs[I(1, 0)] * rhs[I(0, 1)] + lhs[I(1, 1)] * rhs[I(1, 1)] + lhs[I(1, 2)] * rhs[I(2, 1)];
	t[5] = lhs[I(1, 0)] * rhs[I(0, 2)] + lhs[I(1, 1)] * rhs[I(1, 2)] + lhs[I(1, 2)] * rhs[I(2, 2)];

	t[6] = lhs[I(2, 0)] * rhs[I(0, 0)] + lhs[I(2, 1)] * rhs[I(1, 0)] + lhs[I(2, 2)] * rhs[I(2, 0)];
	t[7] = lhs[I(2, 0)] * rhs[I(0, 1)] + lhs[I(2, 1)] * rhs[I(1, 1)] + lhs[I(2, 2)] * rhs[I(2, 1)];
	t[8] = lhs[I(2, 0)] * rhs[I(0, 2)] + lhs[I(2, 1)] * rhs[I(1, 2)] + lhs[I(2, 2)] * rhs[I(2, 2)];

	memcpy(m, t, sizeof(t));
}

void matrix_inverse_or_rotate_3x3(float *m, float *lhs)  // for rotate matrix, its inverse is just its rotate. 旋转矩阵的逆矩阵，就是其本身的转置
{
	// m = inverse(lhs)
	int i, j;
	float t[9];

	for (j = 0; j < 3; j++)
	{
		for (i = 0; i < 3; i++)
		{
			t[3 * j + i] = lhs[3 * i + j];
		}
	}
	memcpy(m, t, sizeof(t));
}

void shear_row(float mat[][3], int r1, int r2, float scalar)
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		mat[r1][i] += scalar * mat[r2][i];
	}
}
void scale_row(float mat[][3], int r, float scalar)
{
	int i;
	for (i = 0; i < 3; ++i)
	{
		mat[r][i] *= scalar;
	}
}

void swap_rows(float** mat, int r1, int r2)
{
	float* tmp;
	tmp = mat[r1];
	mat[r1] = mat[r2];
	mat[r2] = tmp;
}

void set_identity_matrix(float** mat)
{
	int i, j;
	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				mat[i][j] = 1.0f;
			}
			else
			{
				mat[i][j] = 0.0f;
			}
		}
	}
}

int matrix_inv3x3(float* output, float* input)
{
	int i, j, r;
	float input_s[3][3];
	float output_s[3][3];
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			input_s[i][j] = input[i * 3 + j];
		}
	}
	float scalar;
	float shear_needed;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				output_s[i][j] = 1.0f;
			}
			else
			{
				output_s[i][j] = 0.0f;
			}
		}
	}
	for (i = 0; i < 3; ++i)
	{
		if (input_s[i][i] == 0.0)
		{
			for (r = i + 1; r < 3; ++r)
			{
				if (input_s[r][i] != 0.0)
				{
					break;
				}
			}
			if (r == 3)
			{
				return 0;
			}
			swap_rows(input_s, i, r);
			swap_rows(output_s, i, r);
		}
		scalar = 1.0 / input_s[i][i];
		scale_row(input_s, i, scalar);
		scale_row(output_s, i, scalar);
		for (j = 0; j < 3; ++j)
		{
			if (i == j)
			{
				continue;
			}
			shear_needed = -input_s[j][i];
			shear_row(input_s, j, i, shear_needed);
			shear_row(output_s, j, i, shear_needed);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			output[i * 3 + j] = output_s[i][j];
		}
	}
	return 1;
}



void matrix_mult_3x3_by_3x1(float* dst, float* mat3x3, float* vec3x1)
{
	float t[3];

	t[0] = mat3x3[0] * vec3x1[0] + mat3x3[1] * vec3x1[1] + mat3x3[2] * vec3x1[2];
	t[1] = mat3x3[3] * vec3x1[0] + mat3x3[4] * vec3x1[1] + mat3x3[5] * vec3x1[2];
	t[2] = mat3x3[6] * vec3x1[0] + mat3x3[7] * vec3x1[1] + mat3x3[8] * vec3x1[2];
	memcpy(dst, t, sizeof(t));
}


void vec3x1_sub(float* dst, float* mat3x1, float* vec3x1)
{
	float t[3];
	t[0] = mat3x1[0] - vec3x1[0];
	t[1] = mat3x1[1] - vec3x1[1];
	t[2] = mat3x1[2] - vec3x1[2];
	memcpy(dst, t, sizeof(t));
}

void vec3x1_add(float* dst, float* mat3x1, float* vec3x1)
{
	float t[3];
	t[0] = mat3x1[0] + vec3x1[0];
	t[1] = mat3x1[1] + vec3x1[1];
	t[2] = mat3x1[2] + vec3x1[2];
	memcpy(dst, t, sizeof(t));
}

void euclidean_transform_point_dp(float* newP, float* R, float* P, float *t)
{
	float Rp[3];
	matrix_mult_3x3_by_3x1(Rp, R, P);
	newP[0] = Rp[0] + t[0];
	newP[1] = Rp[1] + t[1];
	newP[2] = Rp[2] + t[2];
}

float get_theta_by_r(float* arr_rtheta, int length_rtheta, float fisheye_radius, float r)
{
	int indx = r / fisheye_radius * length_rtheta; // length_rtheta = 2*fisheye_radius
	indx = indx < length_rtheta ? indx : length_rtheta - 1;
	return arr_rtheta[indx];
}


void my_max(unsigned int* old_value, unsigned int new_value)
{
	*old_value = (new_value >= *old_value) ? new_value : (*old_value);
}


int warp_range_to_novel_view(krt_CamParam* ccam, float* range_img, novel_pixel_label_t* labels, krt_CamParam *vcam,
	float* novel_range_img, novel_pixel_label_t tar_label, novel_pixel_label_t* novel_label_img, float* r2t_curve, int r2t_len, float fB, float dthresh, const int edge_radius, int ispinhole, int maxheight, int maxwidth)
{
	int width = ccam->src_width, height = ccam->src_height;
	//printf("!!!!!!!! width = %d height = %d krt_cc[0] = %d krt_cc[1] = %d\n", width, height, ccam->krt_cc[0], ccam->krt_cc[1]);
	float p3D[3] = { 0, 0, 0 };// Mat::zeros(3, 1, CV_32FC1);
	float krt_R[9];

	float temp_invR[9];
	matrix_inverse_or_rotate_3x3(temp_invR, ccam->krt_R);     //matrix_mult_3x3_by_3x3(temp_invR, temp_invR, ccam->krt_R);
															  //temp_invR是ccam->krt_R的转置，ccam->krt_R是当前参考视角的旋转矩阵。旋转矩阵的逆矩阵就是转置矩阵
	matrix_mult_3x3_by_3x3(krt_R, vcam->krt_R, temp_invR); // krt_R = vcam->krt_R * temp_invR -> Rvc


	float krt_t[3];
	// krt_t = 当前参考视角的世界坐标 - 虚拟视角的世界坐标
	//gs begin change
	vec3x1_sub(krt_t, ccam->krt_WorldPosition, vcam->krt_WorldPosition); // krt_t = ccam->krt_WorldPosition - vcam->krt_WorldPosition; 
	matrix_mult_3x3_by_3x1(krt_t, vcam->krt_R, krt_t); //没错, worlposition -> Tw.

	//matrix_mult_3x3_by_3x1(krt_t, krt_R, ccam->krt_WorldPosition);
	//vec3x1_sub(krt_t, krt_t, vcam->krt_WorldPosition);
	//gs end change

	// krt_t = 虚拟视角的旋转矩阵 * krt_t
	// 对于从camera到novel，xv = Rvc * xc + Tvc = Rvc * xc + Rvw*Twc - Rvw*Twv = Rvc *xc + Rvw*(Twc-Twv) ？？？
	//   krt_t = Rvw*(Twc-Twv)
	//   krt_R = Rvc

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{

			novel_pixel_label_t Lab = labels[i * width + j];

			if (Lab != tar_label) { continue; } // -- we project only the required label pixels         //wkj

			float xd_norm = (j - ccam->krt_cc[0]);
			float yd_norm = (i - ccam->krt_cc[1]);
			float radius = sqrtf(xd_norm * xd_norm + yd_norm * yd_norm);
			float theta = get_theta_by_r(r2t_curve, r2t_len, ccam->fisheye_radius, radius);     //step 1:  table look-up based on r 
																								// 这个之前已经有亚精度的打表的计算了，查找radius（投影长度，单位为像素）与角度theta的关系

			float phi = atan2(yd_norm, xd_norm);
			const float sphrad = range_img[i * width + j]; // depth
			if (sphrad <= 0.f) { continue; }


			p3D[2] = sphrad * cos(theta);
			//if(i < 5 && j < 5) printf("i=%d,j=%d,  %f   d= %f\n",i,j, sphrad, p3D[2]);
			float r_undistort = sqrtf(sphrad * sphrad - p3D[2] * p3D[2]);


			p3D[0] = r_undistort * cos(phi);
			p3D[1] = r_undistort * sin(phi);
			// 这个p3D我可以理解，p3D是当前点在参考相机坐标系下的3D位置，单位为m
			// 参考视角下：A(x-B)=E；虚拟视角下：C(x-D)=F，已知A B C D E, 求F
			// 则 F = C( inv(A)E + B - D)

			float np3D[3];// = krt_R * p3D + krt_t;                               //step 2:
			euclidean_transform_point_dp(np3D, krt_R, p3D, krt_t);

			float x_norm = np3D[0] / np3D[2];   //step 3:  check: FOV, image res,fisheye_radius   
			float y_norm = np3D[1] / np3D[2];   //map x,y,z on the plane z = 1
			phi = atan2(y_norm, x_norm);
			r_undistort = sqrtf(x_norm * x_norm + y_norm * y_norm); //(phi, theta) under virtual camera 
			theta = atan(r_undistort); //虚拟视角的投影角度theta，这个theta一定是0~pi/2范围内的值，因为r_undistort必为正



			if (np3D[2] < 0.f) { theta = CV_PI - theta; } // -- beyond 180 degree
			if (theta > vcam->lens_fov)
			{
				continue;
			} // -- out of the lens fov
			float tp2 = theta * theta;
			float tp3 = theta * tp2;
			float tp5 = tp3 * tp2;
			float r = vcam->krt_kc[0] * theta + vcam->krt_kc[1] * tp3 + vcam->krt_kc[2] * tp5; //投影长度（像素）与角度theta的关系
			int xp = (int)roundf(r * cos(phi) + vcam->krt_cc[0]);
			int yp = (int)roundf(r * sin(phi) + vcam->krt_cc[1]); //参考视角上的点，对应于虚拟视角上的点（xp, yp），xp yp都是像素
																  //if PINHOLE
			float newrange_pinhole;
			if (ispinhole)
			{
				float z_depth = sphrad;
				float fx = cos(phi) * r_undistort * r / (theta * x_norm); ///？？？r_undistort/theta是焦距，xnorm/(cons(phi)*r)是每个像素的长度
				float fy = sin(phi) * r_undistort * r / (theta * y_norm);
				//gs begin change
				/*krt_K[0] = fx; //krt_K是内参矩阵
				krt_K[1], krt_K[3], krt_K[6], krt_K[7] = 0;
				krt_K[2] = vcam->krt_cc[0];
				krt_K[4] = fy;
				krt_K[5] = vcam->krt_cc[1];
				krt_K[8] = 1;*/

				krt_K[0] = vcam->krt_K[0]; //krt_K是内参矩阵 使用colmap估计出的内参
				krt_K[1], krt_K[3], krt_K[6], krt_K[7] = 0;
				krt_K[2] = vcam->krt_K[2];
				krt_K[4] = vcam->krt_K[4];
				krt_K[5] = vcam->krt_K[5];
				krt_K[8] = 1;
				//gs end change
				float uvcz[3] = { z_depth * j, z_depth * i, z_depth }; // uvcz是参考视角下的像素坐标，但是为什么乘了z_depth
				float XYZcam[3];

				matrix_inv3x3(krt_K_inv, krt_K);
				matrix_mult_3x3_by_3x1(XYZcam, krt_K_inv, uvcz); // XYZcam = krt_K_inv * uvcz

				p3D[0] = XYZcam[0];
				p3D[1] = XYZcam[1];
				p3D[2] = z_depth; //p3D是参考相机3D坐标系下看到的点

				euclidean_transform_point_dp(np3D, krt_R, p3D, krt_t); // np3D = krt_R * p3D + krt_t	np3D是虚拟相机坐标系下的3D位置

				float v_xyz[3];
				matrix_mult_3x3_by_3x1(v_xyz, krt_K, np3D); //v_xyz = krt_K * np3D		v_xyz是虚拟相机下的像素坐标。虚拟相机和参考相机用同一内参？
				newrange_pinhole = np3D[2];
				xp = (int)roundf(v_xyz[0] / v_xyz[2]); //v_xyz[2]不一定等于z_depth呀
				yp = (int)roundf(v_xyz[1] / v_xyz[2]);
			}

			if (r < vcam->fisheye_radius && xp >= 1 && xp <= (maxwidth - 1) - 1 && yp >= 1 && yp <= (maxheight - 1) - 1)   //because 3x3 
			{
				float newrange;
				if (ispinhole)
				{
					newrange = newrange_pinhole;
				}
				else
				{
					newrange = norm(np3D);                    //step 4:	 虚拟相机坐标系下的深度值newrange
				}

				for (int m = yp - 1; m <= yp + 1; m++)
				{
					for (int n = xp - 1; n <= xp + 1; n++)
					{
						{
							float nrange = novel_range_img[m * maxwidth + n];
							if (nrange == 0.f)
							{
								novel_range_img[m * maxwidth + n] = newrange;
								novel_label_img[m * maxwidth + n] = tar_label;    //wkj
							}
							else if (newrange < nrange)
							{
								novel_range_img[m * maxwidth + n] = newrange; // (xp,yp)周围九宫格内的点，如果没有深度值就赋值为newrange，有深度值则与newrange比较，取较小的
								novel_label_img[m * maxwidth + n] = tar_label;
							}
						}
					}
				} // -- project 1 pixel to 9 pixels to avoid cracks
			}
		} // -- within valid image plane
	}

	return I2R_OK;
}


int alloc_cuRef(cuRef_data_t *ppp, unsigned char* bgra, float* range_img, int width, int height)
{

	ppp->cuTex = malloc(width * height * 3 * sizeof(unsigned char));
	memcpy(ppp->cuTex, bgra, width * height * 3 * sizeof(unsigned char));

	ppp->range_img = malloc(width * height * sizeof(float));
	memcpy(ppp->range_img, range_img, width * height * sizeof(float));

	return I2R_OK;
}

int alloc_camp(cuCam_data_t* cp, krt_CamParam* ccam, krt_CamParam* vcam)
{
	cp->vfradius = vcam->fisheye_radius;
	cp->reffradius = ccam->fisheye_radius;
	cp->vr2tl = (int)(vcam->fisheye_radius * 2.f);
	cp->refr2tl = (int)(ccam->fisheye_radius * 2.f);
	cp->reffov = ccam->lens_fov;
	cp->vfov = vcam->lens_fov;


	cp->vr2t_curve = (float*)malloc(cp->vr2tl * sizeof(float));
	cp->refr2t_curve = (float*)malloc(cp->refr2tl * sizeof(float));

	set_r2t_array_by_kc(cp->vr2t_curve, vcam->krt_kc, cp->vr2tl, cp->vfradius);
	set_r2t_array_by_kc(cp->refr2t_curve, ccam->krt_kc, cp->refr2tl, cp->reffradius);

	// -- prepare matrix data
	float krt_R[9];// = ccam->krt_R * vcam->krt_R.inv();
	float temp_invR[9];
	matrix_inverse_or_rotate_3x3(temp_invR, vcam->krt_R);     // matrix_mult_3x3_by_3x3(krt_R, temp_invR, vcam->krt_R);   //check whether inverse  is right

	matrix_mult_3x3_by_3x3(krt_R, ccam->krt_R, temp_invR); // krt_R用于把虚拟视角3D坐标投影到参考视角3D坐标

	float krt_invR[9];// = krt_R.inv();
	matrix_inverse_or_rotate_3x3(krt_invR, krt_R);       //matrix_mult_3x3_by_3x3(temp_invR, temp_invR, vcam->krt_R);

	float krt_WorldPosition[3];// = ccam->krt_WorldPosition - vcam->krt_WorldPosition;
	vec3x1_sub(krt_WorldPosition, vcam->krt_WorldPosition, ccam->krt_WorldPosition);  //Because there is a minus symbol in the following, so we exchange the place of vcam and ccam.
	float krt_t[3];// = -ccam->krt_R * krt_WorldPosition;
				   //gs begin change
	matrix_mult_3x3_by_3x1(krt_t, ccam->krt_R, krt_WorldPosition);

	//matrix_mult_3x3_by_3x1(krt_t, krt_R, vcam->krt_WorldPosition);
	//vec3x1_sub(krt_t, krt_t, ccam->krt_WorldPosition);
	//gs end change

	cp->refR = (float*)malloc(9 * sizeof(float)); //krt_R是参考相机R矩阵的逆，乘以虚拟相机的R矩阵
	memcpy(cp->refR, krt_R, 9 * sizeof(float));
	cp->refinvR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->refinvR, krt_invR, 9 * sizeof(float)); //krt_invR是krt_R的逆，用于把参考视角3D坐标投到虚拟视角3D坐标
	cp->reft = (float*)malloc(3 * sizeof(float));
	memcpy(cp->reft, krt_t, 3 * sizeof(float));

	float I[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };//Mat::eye(3, 3, CV_32FC1); // -- identity matrix

	cp->vR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->vR, I, 9 * sizeof(float));
	cp->vinvR = (float*)malloc(9 * sizeof(float));
	memcpy(cp->vinvR, I, 9 * sizeof(float));
	cp->vt = (float*)malloc(3 * sizeof(float));
	memset(cp->vt, 0, 3 * sizeof(float));


	cp->refCC = (float*)malloc(2 * sizeof(float));
	cp->refCC[0] = ccam->krt_cc[0], cp->refCC[1] = ccam->krt_cc[1];

	cp->refKc = (float*)malloc(3 * sizeof(float));
	cp->refKc[0] = ccam->krt_kc[0], cp->refKc[1] = ccam->krt_kc[1], cp->refKc[2] = ccam->krt_kc[2];


	cp->vCC = (float*)malloc(2 * sizeof(float));
	cp->vCC[0] = vcam->krt_cc[0], cp->vCC[1] = vcam->krt_cc[1];

	cp->vKc = (float*)malloc(3 * sizeof(float));
	cp->vKc[0] = vcam->krt_kc[0], cp->vKc[1] = vcam->krt_kc[1], cp->vKc[2] = vcam->krt_kc[2];
	return I2R_OK;
}

int destroy_camp(cuCam_data_t* cp)
{
	free(cp->refR);
	free(cp->refinvR);
	free(cp->reft);
	free(cp->refCC);
	free(cp->refKc);
	free(cp->refr2t_curve);

	free(cp->vR);
	free(cp->vinvR);
	free(cp->vt);
	free(cp->vCC);
	free(cp->vKc);
	free(cp->vr2t_curve);
	return I2R_OK;
}

float find_medium(float depth_window[], int N)
{
	int i, j;
	int length = N - 1;
	float total = 0;

	for (i = 0; i < length; i++)
	{

		//find max in each cycle
		for (j = 0; j < length - i; j++)
		{
			if (depth_window[j] > depth_window[j + 1])
			{
				//interchange positions
				float temp = depth_window[j];
				depth_window[j] = depth_window[j + 1];
				depth_window[j + 1] = temp;
			}
		}
	}
	for (i = 0; i < N; i++)  	total = total + depth_window[i];
	//return total/N;
	return depth_window[N / 2]; //先冒泡排序然后返回中位数

}


void median_filter_range_image(float* irange, float* orange, novel_pixel_label_t* ilabels, novel_pixel_label_t* olabels, int width, int height, int kradius, float fB, float mindp, float dprange)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			if (ilabels[y * width + x] == novel_pixel_hole)
			{
				olabels[y * width + x] = novel_pixel_hole;
				orange[y * width + x] = 0.f;
				continue;
			}

			const int range_level = 512 + 1; // -- evenly divide the range into multiple bins, +1 for invalid holes
											 //int hist[range_level];
			int hist[512 + 1];
			memset(hist, 0, range_level * sizeof(int)); // -- histogram
			float bin_len = dprange / (float)(range_level - 1);
			//int vnbr = 0;
			for (int i = -kradius; i <= kradius; i++)
			{
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = max(0, min(y + i, height - 1));  //make sure the index (x+j, y+i) is located in the image.
					int imx = max(0, min(x + j, width - 1)); // -- clamp
					if (ilabels[imy * width + imx] == novel_pixel_hole)
					{
						hist[0] ++;
						continue;
					} // -- invalid holes

					float ival = fB / irange[imy * width + imx]; // -- input range value
					int binID = max(0, min(range_level - 2, (int)floor((ival - mindp) / bin_len)));
					hist[binID + 1] ++;
					//vnbr++;
				}
			}
			const int tNbr = (2 * kradius + 1) * (2 * kradius + 1);
			int mID = 0; // -- median bin ID
			int aval = 0; // -- accumulated value
			for (int id = 0; id < range_level + 1; id++)
			{
				aval += hist[id];
				if (aval > tNbr / 2)
				{
					mID = id;
					break;
				}
			} // -- select the median value by accumulating the histogram
			if (mID == 0)
			{
				orange[y * width + x] = 0.f;
				olabels[y * width + x] = novel_pixel_hole;
			}
			else
			{
				float oval = fB / ((mID - 1.f + 0.5f) * bin_len + mindp);
				orange[y * width + x] = oval;
				olabels[y * width + x] = novel_pixel_main;
			}
		}
}

void median_filter_range_image__(float* irange, float* orange, novel_pixel_label_t* ilabels, novel_pixel_label_t* olabels, int width, int height, int kradius, float fB, float mindp, float dprange)
{

	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			orange[y * width + x] = irange[y * width + x];
			{
				float depth_window[9];
				float medium_value = 0;
				for (int i = -kradius; i <= kradius; i++)
				{
					for (int j = -kradius; j <= kradius; j++)
					{
						int imy = max(0, min(y + i, height - 1));  //make sure the index (x+j, y+i) is located in the image.
						int imx = max(0, min(x + j, width - 1)); // -- clamp
																 //depth_window[] = [imy * width + imx];
						depth_window[(i + kradius) * (2 * kradius + 1) + (j + kradius)] = irange[imy * width + imx];
					}
				}
				medium_value = find_medium(depth_window, 9);
				orange[y * width + x] = medium_value;
			}

		}
}


// add bilateral filter for range image. Modified in Aug, 2020
void bilateral_filter_range_image(float* irange, float* orange, novel_pixel_label_t* ilabels, novel_pixel_label_t* olabels, int width, int height, int kradius, float fB, float* ws_arr, float* wd_arr, int wd_arr_len, float drange)
{
	int x, y;

	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			if (ilabels[y * width + x] == novel_pixel_hole)
			{
				olabels[y * width + x] = novel_pixel_hole;
				orange[y * width + x] = 0.f;
				continue;
			}

			float range = fB / irange[y * width + x];
			float sumC = 0.f;
			float sumW = 0.f;
			const float sigs = (float)(kradius * 1.2f);
			const float sigd = (float)(kradius * 3.6f);
			for (int i = -kradius; i <= kradius; i++) {
				for (int j = -kradius; j <= kradius; j++)
				{

					int imy = max(0, min(y + i, height - 1));
					int imx = max(0, min(x + j, width - 1));

					if (ilabels[imy * width + imx] == novel_pixel_hole)
					{
						continue;
					}
					int dist = (i * i + j * j);
					float ws = ws_arr[dist];

					float disp = fB / irange[imy * width + imx];
					int indx = fabs(disp - range) / drange * wd_arr_len;
					indx = indx < wd_arr_len ? indx : wd_arr_len - 1;
					float wd = wd_arr[indx];
					float d = irange[imy * width + imx];

					sumC += (ws * wd * d);
					sumW += (ws * wd);

				}
			}


			if (sumW < 1e-5f) { continue; }

			orange[y * width + x] = sumC / sumW;
			olabels[y * width + x] = novel_pixel_main;
		}
}

int median_filtering_novel_range(float* range, novel_pixel_label_t* labels, float fB, float mindp, float dprange, int radius, int width, int height)
{
	float* orange = NULL;
	novel_pixel_label_t* olabels = NULL;
	orange = (float*)malloc(width * height * sizeof(float));
	olabels = (float*)malloc(width * height * sizeof(novel_pixel_label_t));
	memset(orange, 0, width * height * sizeof(float));
	memset(olabels, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));

	median_filter_range_image(range, orange, labels, olabels, width, height, radius, fB, mindp, dprange);

	memcpy(range, orange, width * height * sizeof(float));
	memcpy(labels, olabels, width * height * sizeof(novel_pixel_label_t));

	free(orange);
	free(olabels);
	return I2R_OK;
}
// add bilateral filter for range image. Modified in Aug, 2020
int bilateral_filtering_novel_range(float* range, novel_pixel_label_t* labels, float fB, int radius, int width, int height, float* ws_arr, float* wd_arr, int wd_arr_len, float drange)
{
	float* orange = NULL;
	novel_pixel_label_t* olabels = NULL;
	orange = (float*)malloc(width * height * sizeof(float));
	olabels = (float*)malloc(width * height * sizeof(novel_pixel_label_t));
	memset(orange, 0, width * height * sizeof(float));
	memset(olabels, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));

	bilateral_filter_range_image(range, orange, labels, olabels, width, height, radius, fB, ws_arr, wd_arr, wd_arr_len, drange);

	memcpy(range, orange, width * height * sizeof(float));
	memcpy(labels, olabels, width * height * sizeof(novel_pixel_label_t));

	free(orange);
	free(olabels);
	return I2R_OK;
}

float get_theta_by_r_dp(float* arr_rtheta, int length_rtheta, float fisheye_radius, float r)
{
	int indx = r / fisheye_radius * length_rtheta;
	indx = indx < length_rtheta ? indx : length_rtheta - 1;
	return arr_rtheta[indx];
}


void get_unit_view_vector_dp(float* rtheta_arr, int rtheta_len, float* cc, float fisheye_radius, float x, float y, float *unit_vv)
{
	float xd_norm = x - cc[0];
	float yd_norm = y - cc[1];
	float radius = sqrtf(xd_norm * xd_norm + yd_norm * yd_norm);

	float theta = get_theta_by_r_dp(rtheta_arr, rtheta_len, fisheye_radius, radius);  //resort to  look-up table
	float phi = atan2(yd_norm, xd_norm);

	const float sphrad = 1.f; // -- unit vector
	unit_vv[2] = sphrad * cos(theta);
	float r_undistort = sqrtf(sphrad - unit_vv[2] * unit_vv[2]);
	unit_vv[0] = r_undistort * cos(phi);
	unit_vv[1] = r_undistort * sin(phi);
}


int project_to_camera_dp(float* worldP, float* R, float *t, float* kc, float* cc, float fisheye_radius, float lens_fov, int* img_reso, float *xp, float *yp, int ispinhole)
{
	float newWorldP[3];
	euclidean_transform_point_dp(newWorldP, R, worldP, t); //把虚拟相机3D坐标系下的点worldP投影到参考相机下得到点newWorldP
	float x_norm = newWorldP[0] / newWorldP[2];
	float y_norm = newWorldP[1] / newWorldP[2];

	//float phi = atan2(y_norm, x_norm);
	float ryx = fabs(y_norm / (x_norm + 1e-10f)); // -- avoid /0 error
	float r_undistort = sqrtf(x_norm * x_norm + y_norm * y_norm);
	float theta = atan(r_undistort);

	if (newWorldP[2] < 0.f) { theta = CV_PI - theta; } // -- beyond 180 degree
	if (theta > lens_fov) { return I2R_ERR; } // -- out of the lens fov

	float tp2 = theta * theta;
	float tp3 = theta * tp2;
	float tp5 = tp3 * tp2;
	float r = kc[0] * theta + kc[1] * tp3 + kc[2] * tp5;
	float fabsx = sqrtf(r * r / (1.f + ryx * ryx));
	float fabsy = ryx * fabsx;
	*xp = (x_norm > 0.f ? fabsx : -fabsx) + cc[0];
	*yp = (y_norm > 0.f ? fabsy : -fabsy) + cc[1]; //参考视角下的点(xp, yp)

	if (ispinhole)
	{
		float uvz[3];
		matrix_mult_3x3_by_3x1(uvz, krt_K, newWorldP);
		uvz[0] = uvz[0] / uvz[2];
		uvz[1] = uvz[1] / uvz[2]; //针孔相机，把之前乘的那个深度值range除去了，但是问题是uvz[2]不一定等于range
		*xp = uvz[0];
		*yp = uvz[1];
	}

	if ((r > fisheye_radius) || (*xp > img_reso[0]) || (*xp < 0) || (*yp > img_reso[1]) || (*yp < 0))
	{
		return I2R_ERR;// I2R_ERR;
	} // -- out of the image plane
	else
	{
		return I2R_OK;
	}
}


#define LANCZOS_TAB_SIZE        3
#define LANCZOS_FAST_SCALE      100
#define LANCZOS_FAST_MAX_SIZE   4096
static double tbl_lanczos_coef[LANCZOS_FAST_MAX_SIZE * LANCZOS_FAST_SCALE];
#define lanczos_coef(x) tbl_lanczos_coef[(int)(fabs(x) * LANCZOS_FAST_SCALE + 0.5)]

// sinc function
static double sinc(double x)
{
	x *= CV_PI;
	if (x < 0.01 && x > -0.01) {
		double x2 = x * x;
		return 1.0f + x2 * (-1.0 / 6.0 + x2 / 120.0);
	}
	else {
		return sin(x) / x;
	}
}

void init_lanczos_filter()
{
	int i;
	for (i = 0; i < LANCZOS_FAST_MAX_SIZE*LANCZOS_FAST_SCALE; i++)
	{
		double x = (double)i / LANCZOS_FAST_SCALE;
		tbl_lanczos_coef[i] = sinc(x) * sinc(x / LANCZOS_TAB_SIZE);
	}
}

float filtre_lanczos(unsigned char * src, float j, float i, int input_width, int input_height, int i_src, int RGB_offset)
{
	double coef, sum = 0, res = 0;
	int m, n, idx_x, idx_y;
	float ret_val = 0;

	for (n = -LANCZOS_TAB_SIZE; n < LANCZOS_TAB_SIZE; n++)
	{
		for (m = -LANCZOS_TAB_SIZE; m < LANCZOS_TAB_SIZE; m++)
		{
			idx_x = (int)i + m + 1;
			idx_y = (int)j + n + 1;

			coef = lanczos_coef(i - idx_x) * lanczos_coef(j - idx_y);
			//coef = 1;
			// when the neib. pixel is outside the boundary, using the boundary pixels
			idx_x = (idx_x < 0) ? 0 : idx_x;
			idx_y = (idx_y < 0) ? 0 : idx_y;
			idx_x = (idx_x >= input_width) ? (input_width - 1) : idx_x;
			idx_y = (idx_y >= input_height) ? (input_height - 1) : idx_y;

			//res += src[idx_x + idx_y * i_src] * coef;
			res += src[(idx_x + idx_y * i_src) * 3 + RGB_offset] * coef;
			sum += coef;
		}
	}

	if (sum != 0) {
		ret_val = (float)(res / sum + 0.5);
		ret_val = clip(ret_val, 0.0f, 255.0f);
	}

	return ret_val;
}


void generate_novel_view(cuRef_data_t refData, float* novel_range_img, unsigned char* novel_view, novel_pixel_label_t* novel_ls, float* novel_conf,
	novel_pixel_label_t tarL, int width, int height, cuCam_data_t camp, const float prefW, float fB, int ispinhole, int maxheight, int maxwidth)
{	// refData是参考相机，camp中既有虚拟相机参数也有参考相机参数，其余参数都是虚拟相机的
	printf("prefW: %f\n", prefW);
	int x, y;
	for (y = 0; y < maxheight; y++)
		for (x = 0; x < maxwidth; x++)
		{
			float range = novel_range_img[y * maxwidth + x];

			if (range <= 0.f)
			{
				continue;       //wkj
			} // -- invalid range data

			float unitvv[3];
			//虚拟相机像素坐标系（x，y）到世界坐标系unitvv
			get_unit_view_vector_dp(camp.vr2t_curve, camp.vr2tl, camp.vCC, camp.vfradius, x, y, unitvv); //step1: 虚拟相机像素坐标系到虚拟相机下的3D坐标range image (x,y,range)  into  world (x',y',z')
			unitvv[0] *= range; unitvv[1] *= range; unitvv[2] *= range; //因为get_unit_view_vector_dp()函数把深度值当作1在算，所以这里要乘以深度值
			if (ispinhole)
			{
				float uvz[3] = { range * x, range * y, range }; //针孔相机从3D坐标系向像素坐标转换时，都乘了一个深度值range
				matrix_mult_3x3_by_3x1(unitvv, krt_K_inv, uvz);
				//uvz是虚拟视角下的像素坐标乘了个深度值range，krt_K_inv是虚拟相机内参的逆，unitvv是虚拟相机式视角下的3D坐标
			}

			int reso[] = { width, height };
			float xpf, ypf;
			//这个注释不是我写的：世界坐标系unitvv 投影到参考相机坐标（u，v）即（xpf，ypf）（可能不是整像素），然后lanczos滤波从（u，v）附近取值，unitvv虚拟相机世界坐标系
			if (project_to_camera_dp(unitvv, camp.refR, camp.reft, camp.refKc, camp.refCC, camp.reffradius, camp.reffov, reso, &xpf, &ypf, ispinhole)) //step2: (x',y',z')  into  (xpf, ypf)世界坐标系到参考相机像素坐标系
			{	//先把虚拟视角下的3D点unitvv投影到参考视角，得到像素坐标（xpr, ypf）
				//printf("xpf: %f, %f\n", x.pf, ypf);
				//float4 pix = tex2D<float4>(refData.cuTex, xpf + 0.5f, ypf + 0.5f);
				unsigned char  pix[3];
				int xxx = max(0, min((int)(round(xpf)), width - 1));
				int yyy = max(0, min((int)(round(ypf)), height - 1));
				pix[0] = filtre_lanczos(refData.cuTex, ypf + 0.5f, xpf + 0.5f, width, height, width, 0);    //use subpixel interpolation
				pix[1] = filtre_lanczos(refData.cuTex, ypf + 0.5f, xpf + 0.5f, width, height, width, 1);
				pix[2] = filtre_lanczos(refData.cuTex, ypf + 0.5f, xpf + 0.5f, width, height, width, 2);

				novel_view[y * maxwidth * 3 + x * 3 + 0] = (unsigned char)(pix[0]); //使用参考视角的RGB图像获得虚拟视角的RGB图像
				novel_view[y * maxwidth * 3 + x * 3 + 1] = (unsigned char)(pix[1]);
				novel_view[y * maxwidth * 3 + x * 3 + 2] = (unsigned char)(pix[2]);

				// -- also compute the back projection for confidence   底下都是计算conf，（xp，yp）参考相机图像坐标系坐标
				int xp = max(0, min((int)(round(xpf + 0.5f)), 1 * width - 1));
				int yp = max(0, min((int)(round(ypf + 0.5f)), 1 * height - 1));
				float bunitvv[3]; // -- unit view vector of reference camera
								  //参考视角下的像素坐标（xp，yp）投影到参考相机3D坐标系bunitvv
				get_unit_view_vector_dp(camp.refr2t_curve, camp.refr2tl, camp.refCC, camp.reffradius, xpf, ypf, bunitvv);

				float brange = refData.range_img[yp  * width / 1 + xp / 1];
				if (brange <= 0.f)
				{
					continue;
				}  // -- invalid holes
				bunitvv[0] *= brange; bunitvv[1] *= brange; bunitvv[2] *= brange; //get_unit_view_vector_dp是把深度值当作1去算参考相机视角下的D坐标的，这里乘以深度值得到真实3D坐标
				if (ispinhole)
				{
					float uvz[3] = { brange * x, brange * y, brange };
					matrix_mult_3x3_by_3x1(bunitvv, krt_K_inv, uvz);
				}
				// bunitvv是参考相机坐标系下的3D坐标
				// -- project back    
				float pmt[] = { bunitvv[0] - camp.reft[0], bunitvv[1] - camp.reft[1], bunitvv[2] - camp.reft[2] };
				float pback[3]; // -- 3D points under ref camera's coordinate system
								//pback是（xp，yp）在虚拟相机下的3D坐标
				matrix_mult_3x3_by_3x1(pback, camp.refinvR, pmt);

				float bxpf, bypf;
				//虚拟相机下的3D坐标pback，用I矩阵和0向量重新映射一遍，用来观测误差参数
				if (project_to_camera_dp(pback, camp.vR, camp.vt, camp.vKc, camp.vCC, camp.vfradius, camp.vfov, reso, &bxpf, &bypf, ispinhole))
				{
					//if (y == 429 && x == 954) getchar();
					float diffx = x - bxpf, diffy = y - bypf;             //wkj
					float dist = sqrtf(diffx * diffx + diffy * diffy);
					//printf("!!!!!!!! %f\n", prefW);
					novel_conf[y * maxwidth + x] = expf(-dist / 20.f) * prefW;          //mconf   权重用到该点的深度图，深度置信度
					//printf("%f\n", novel_conf[y * maxwidth + x]);
				}
			}

		}


}


int warp_image_to_novel_view(novel_view_t* nv, cuRef_data_t *curef, cuCam_data_t* cuCam, int width, int height, float fB, int ispinhole, int maxheight, int maxwidth)
{

	generate_novel_view(*curef, nv->mrange_img, nv->mnovel_view, nv->mlabel_img, nv->mconf, novel_pixel_main, width, height, *cuCam, nv->prefW, fB, ispinhole, maxheight, maxwidth);

	return I2R_OK;
}

int gen_novel_view(krt_CamParam* ccam, unsigned char* bgra, float* range_img, krt_CamParam *vcam, novel_view_t *nv, float fB, float mindepth, float maxdepth, int s, int pixel_h, int pixel_w, int maxheight, int maxwidth)
{	//ccam是参考视角的参数, bgra是当前参考视角的RGB图像, range_img是当前参考视角的D图像, vcam是虚拟相机的参数
	//nv是参考视角投影到虚拟视角的各种初始化过参数, fB,min,max是自定义参数, s是参考视角的序列号9,11
	//I2R_ASSERT(ccam->src_width == vcam->src_width);
	//I2R_ASSERT(ccam->src_height == vcam->src_height);

	//int width = vcam->src_width, height = vcam->src_height;
	int width = pixel_w, height = pixel_h;
	// -- allocate texture object for bgra data
	cuRef_data_t cuRef;
	alloc_cuRef(&cuRef, bgra, range_img, width, height); // rgb & depth for cuRef
	cuCam_data_t camp;
	alloc_camp(&camp, ccam, vcam); // camera parameter for camp

	//	--	step 0. label foreground/background pixels for later processing
	//				foreground pixels will be gaussian-smoothed after merging all the novel views
	//				background pixels will be used to perform a erosion to remove ghost coutours
	novel_pixel_label_t* labels = NULL;
	labels = malloc(width * height * sizeof(novel_pixel_label_t));
	//初始化全部label层都是空洞
	memset(labels, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));

	float maxdisp = fB / mindepth, mindisp = fB / maxdepth;
	float drange = maxdisp - mindisp + 1;
	const float dtr = 20*0.01666667f; // -- disparity threashold ratio  ???? gs add dtr 原本为 0.01666667f;
	const int edge_radius = 4; // -- the same as in the original paper   for 1080P is 4. 

	/////!!!!!!!!!!!!!!!!!!!!!!!!!!!!! label_boundary_pixels is random !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//标记label层后经边缘区域，其余区域标记为main layer
	label_boundary_pixels2(cuRef.range_img, labels, novel_pixel_background_boundary, fB, dtr * drange, edge_radius, width, height, mindepth, maxdepth);
	// 左右静态增长的前后景边缘
	//gs begin
	if (s == 0)
	{
		FILE *tempfile = fopen("./results/hlabel1.txt", "wb");
		for (int di = 0; di < height; di++)
		{
			for (int dj = 0; dj < width; dj++)
			{
				fprintf(tempfile, "%d ", labels[di*width + dj]);
			}
			fprintf(tempfile, "\n");
		}
		fclose(tempfile);

		FILE *tempfile2 = fopen("./results/hrange1.txt", "wb");
		for (int di = 0; di < height; di++)
		{
			for (int dj = 0; dj < width; dj++)
			{
				fprintf(tempfile2, "%lf ", range_img[di*width + dj]);
			}
			fprintf(tempfile2, "\n");
		}
		fclose(tempfile2);
	}
	if (s == 1)
	{
		FILE *tempfile = fopen("./results/hlabel3.txt", "wb");
		for (int di = 0; di < height; di++)
		{
			for (int dj = 0; dj < width; dj++)
			{
				fprintf(tempfile, "%d ", labels[di*width + dj]);
			}
			fprintf(tempfile, "\n");
		}
		fclose(tempfile);

		FILE *tempfile2 = fopen("./results/hrange3.txt", "wb");
		for (int di = 0; di < height; di++)
		{
			for (int dj = 0; dj < width; dj++)
			{
				fprintf(tempfile2, "%lf ", range_img[di*width + dj]);
			}
			fprintf(tempfile2, "\n");
		}
		fclose(tempfile2);
	}
	// gs end

	//	--	step 1. warp range image by forward projection, 
	int r2t_len = (int)(ccam->fisheye_radius * 2.f);
	float* r2t_curve;// = new float[r2t_len];
	r2t_curve = (float*)malloc(r2t_len * sizeof(float));
	memset(r2t_curve, 0, r2t_len * sizeof(float));
	set_r2t_array_by_kc(r2t_curve, ccam->krt_kc, r2t_len, ccam->fisheye_radius); //打表

	clock_t st;

	st = clock();
	// -- warp main/foreground layer sperately, 只投影novel_pixel_main的pixel
	// r2t_len = 参考相机的fisheye_radius*2, r2t_curve是一系列根
	warp_range_to_novel_view(ccam, range_img, labels, vcam, nv->mrange_img, novel_pixel_main, nv->mlabel_img, r2t_curve, r2t_len, fB, dtr * drange, edge_radius, IS_PINHOLE, maxheight, maxwidth);
	free(r2t_curve);

	printf("warp_range_to_novel_view time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);
	st = clock();
	//	--	step 2. median filtering the novel range image. Updated in Aug, 2020.
#ifdef BILATERAL_FILTER
	int radius = 3;
	float* ws_array;
	ws_array = (float*)malloc((radius * radius * 2 + 1) * sizeof(float));
	memset(ws_array, 0, (radius * radius * 2 + 1) * sizeof(float));
	set_ws_array(ws_array, drange, radius);

	int wd_array_len = (int)(drange * 2.f);
	float* wd_array;
	wd_array = (float*)malloc(wd_array_len * sizeof(float));
	memset(wd_array, 0, wd_array_len * sizeof(float));
	set_wd_array(wd_array, wd_array_len, radius, drange);

	bilateral_filtering_novel_range(nv->mrange_img, nv->mlabel_img, fB, radius, width, height, ws_array, wd_array, wd_array_len, drange);
	free(ws_array);
	free(wd_array);
	printf("bilateral_filtering_novel_range time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);
#else
	median_filtering_novel_range(nv->mrange_img, nv->mlabel_img, fB, mindisp, drange, 4, maxwidth, maxheight);

	printf("median_filtering_novel_range time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);
#endif // BILATERAL_FILTER
	st = clock();

	//	// add refine depth map Start
	//FILE* in_depth;
	//if (s == 0)
	//{
	//	in_depth = fopen("..//depth-9to10.yuv", "rb");

	//}
	//else if (s == 1)
	//{
	//	in_depth = fopen("..//depth-11to10.yuv", "rb");
	//}
	////in_depth = fopen("..//depth-10.yuv", "rb");
	//unsigned char* depth = malloc(width * height * 1 * sizeof(unsigned char));
	//fread(depth, 1, width * height *1, in_depth);
	//float* in_depth_float = malloc(width * height * sizeof(float));
	////float maxdisp = fB / mindepth;
	////float mindisp = fB / maxdepth;
	//for (int j = 0; j < height; j++)
	//{
	//	for (int i = 0; i < width; i++)
	//	{
	//		in_depth_float[j * width + i] = (float)((float)depth[j * width + i] / 255.f * (maxdisp - mindisp) + mindisp);
	//		in_depth_float[j * width + i] = (float)(fB / (float)in_depth_float[j * width + i]);
	//	}
	//}
	//memcpy(nv->mrange_img, in_depth_float, width * height * sizeof(float));


	//	--	step 3. generate layered novel view image based on the novel range image
	warp_image_to_novel_view(nv, &cuRef, &camp, width, height, fB, IS_PINHOLE, maxheight, maxwidth);
	//nv是新视角，cuRef中是参考视角的RGB与深度图，camp有参考视角相机的参数也有虚拟相机参数，
	printf("warp_image_to_novel_view time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);

	destroy_camp(&camp);
	free(cuRef.range_img);
	free(labels);


	return I2R_OK;
}


void merge_novel_views_mainlayer(novel_view_t* nvs, int nbr, const float fB, const float dthresh,
	int width, int height, unsigned char* mnv, novel_pixel_label_t* olabel, float* orange, float* dist, int* flag)
{
	int x, y;
	float minNum = 0;
	//add BG start
	//unsigned char* bg_color = malloc(width * height * 3 * sizeof(unsigned char));
	//unsigned char* bg_depth = malloc(width * height * 1 * sizeof(unsigned char));
	//FILE* in_bg_color = fopen("..//BG_10.yuv", "rb");
	//FILE* in_bg_depth = fopen("..//BG-depth-10.yuv", "rb");
	//fread(bg_color, 1, width * height * 3, in_bg_color);
	//fread(bg_depth, 1, width * height * 3, in_bg_depth);
	//float* in_img_depth_float = malloc(width * height * sizeof(float));
	////float fB = 32504.0;   //80000  90000  120000
	//float maxdisp = fB / mindepth;
	//float mindisp = fB / maxdepth;
	//for (int j = 0; j < height; j++)
	//{
	//	for (int i = 0; i < width; i++)
	//	{
	//		in_img_depth_float[j * width + i] = (float)((float)bg_depth[j * width + i] / 255.f * (maxdisp - mindisp) + mindisp);
	//		in_img_depth_float[j*width+i] = (float)(fB / (float)in_img_depth_float[j * width + i]);
	//	}
	//}
	//add End
	float dist_weight[2] = {
		expf(-dist[0] / 5.f),
		expf(-dist[1] / 5.f)
		//1.0/dist[0],1.0/dist[1]
		//dist[0]/(dist[0]+dist[1]),
		//dist[1]/(dist[0]+dist[1])	
	};
	if (flag == 1) { dist_weight[0] = 1, nbr = 1; }
	//dist_weight[0] = max(0.01, min(0.09, dist[0]));
	//dist_weight[1] = max(0.01, min(0.09, dist[1]));
	float rgbt[3];
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			float bgr[3] = { 0.f, 0.f, 0.f };
			float sumW = 0.f, sumR = 0.f, minR = 1e10f;
			int  is_hole = 1;
			//if (y == 429 && x == 954) getchar();

			//遍历两个参考视点
			for (int i = 0; i < nbr; i++)
			{	//如果一个纹理图该像素位置为空洞，则跳过该纹理图
				if (nvs[i].mlabel_img[y * width + x] != novel_pixel_main)////非深度图后景边缘区域的处理
				{
					continue;                             //wkj
				}
				/*if (nvs[i].mnovel_view[y * width * 3 + x * 3 + 0] < (10/255) && nvs[i].mnovel_view[y * width * 3 + x * 3 + 1] < (10 / 255) && nvs[i].mnovel_view[y * width * 3 + x * 3 + 2] < (10 / 255))
				{
				continue;
				}*/
				/*
				rgbt[0] = nvs[i].mnovel_view[y * width * 3 + x * 3 + 0];
				rgbt[1] = nvs[i].mnovel_view[y * width * 3 + x * 3 + 1];
				rgbt[2] = nvs[i].mnovel_view[y * width * 3 + x * 3 + 2];
				if (rgbt[0] >= minNum || rgbt[1] >= minNum || rgbt[2] >= minNum) {
				float range = nvs[i].mrange_img[y * width + x];
				//conf_t = max(0.01, min(0.99, nvs[i].mconf[y * width + x]));
				float conf = nvs[i].mconf[y * width + x] * dist_weight[i];
				sumW += conf;
				sumR += range;
				minR = range < minR ? range : minR;
				for (int chan = 0; chan < 3; chan++)
				{
				bgr[chan] += (conf * rgbt[chan]);
				}
				}*/


				float range = nvs[i].mrange_img[y * width + x];
				float conf = nvs[i].mconf[y * width + x] * dist_weight[i];
				sumW += conf;
				sumR += range;
				minR = range < minR ? range : minR;
				//权重乘以该参考视点的
				for (int chan = 0; chan < 3; chan++)
				{
					bgr[chan] += (conf * nvs[i].mnovel_view[y * width * 3 + x * 3 + chan]);
				}

			}
			//ori
			if (sumW < 1e-5f) { continue; }

			for (int chan = 0; chan < 3; chan++)
			{
				mnv[y * width * 3 + x * 3 + chan] = (unsigned char)(max(0.f, min(255.f, bgr[chan] / sumW)));
			}
			// add Start
			//if (sumW < 1e-5f)
			//{
			//	
			//	mnv[y * width * 3 + x * 3 + 0] = bg_color[y * width*3 + x * 3 + 0];
			//	mnv[y * width * 3 + x * 3 + 1] = bg_color[y * width*3 + x * 3 + 1];
			//	mnv[y * width * 3 + x * 3 + 2] = bg_color[y * width*3 + x * 3 + 2];
			//	olabel[y * width + x] = novel_pixel_main;
			//	orange[y * width + x] = in_img_depth_float[y * width + x];
			//	continue;
			//}
			// add End
			olabel[y * width + x] = novel_pixel_main;
			orange[y * width + x] = minR;// sumR / float(sumN);

		}
}


void bilateral_hole_filling(unsigned char* bgr, float* irange, float* orange, novel_pixel_label_t* ilabels, novel_pixel_label_t* olabels, int width, int height, int kradius, float fB, float sigd)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{

			if (ilabels[y * width + x] != novel_pixel_hole)
			{
				olabels[y * width + x] = ilabels[y * width + x];
				orange[y * width + x] = irange[y * width + x];

				continue;
			}

			float mindisp = 1e10f;
			int mindist = 0;
			// -- move along 8 directions to find the smallest disparity
			for (int movx = x + 1; movx < width; movx++)
			{
				if (ilabels[y * width + movx] != novel_pixel_hole)
				{
					float disp = fB / irange[y * width + movx];
					if (disp < mindisp)
					{
						mindisp = disp; mindist = abs(movx - x);
					}
					break;
				}
			} // -- move horizontally

			for (int movx = x - 1; movx >= 0; movx--)
			{
				if (ilabels[y * width + movx] != novel_pixel_hole)
				{
					float disp = fB / irange[y * width + movx];
					if (disp < mindisp)
					{
						mindisp = disp; mindist = abs(movx - x);
					}
					break;
				}
			} // -- move horizontally

			for (int movy = y + 1; movy < height; movy++)
			{
				if (ilabels[movy * width + x] != novel_pixel_hole)
				{
					float disp = fB / irange[movy * width + x];
					if (disp < mindisp)
					{
						mindisp = disp; mindist = abs(movy - y);
					}
					break;
				}
			} // -- move vertically

			for (int movy = y - 1; movy >= 0; movy--)
			{
				if (ilabels[movy * width + x] != novel_pixel_hole)
				{
					float disp = fB / irange[movy * width + x];
					if (disp < mindisp)
					{
						mindisp = disp; mindist = abs(movy - y);
					}
					break;
				}
			} // -- move vertically

			if (mindisp > 1e9f) { continue; } // -- some kind of exception ???

			float sumC[] = { 0.f, 0.f, 0.f };
			float sumW = 0.f;
			const float sigs = (float)(kradius * 1.2f);
			for (int i = -kradius; i <= kradius; i++) {
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = max(0, min(y + i, height - 1));
					int imx = max(0, min(x + j, width - 1));
					if (ilabels[imy * width + imx] == novel_pixel_hole)
					{
						continue;
					}
					float dist = sqrtf((float)(i * i + j * j));
					float ws = expf(-dist / sigs);

					float disp = fB / irange[imy * width + imx];
					float wd = expf(-fabs(disp - mindisp) / sigd);

					for (int c = 0; c < 3; c++)
					{
						float d = bgr[imy * width * 3 + imx * 3 + c];
						sumC[c] += (ws * wd * d);
					}
					sumW += (ws * wd);
				}
			} // -- disparity based joint bilateral filtering

			  //if (sumW < 1e-5f) { return; }
			if (sumW < 1e-5f) { continue; }
			for (int c = 0; c < 3; c++)
			{
				bgr[y * width * 3 + x * 3 + c] = sumC[c] / sumW;
			}
			orange[y * width + x] = fB / mindisp;
			olabels[y * width + x] = novel_pixel_foreground_boundary; // -- set to forground edge so that it will be smoothed later

		}
}

int fill_holes_with_background_color(unsigned char* img, novel_pixel_label_t* labels, novel_pixel_label_t* tlabels, float* range, float* trange,
	float fB, int width, int height, float sigd)

{


	int kradius = 30;     //30

	int base_param = 20;    //wkj  fixed parameter

	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			//如果该像素点是空洞，则取空洞附近kradius个像素点的深度值的加权和作为空洞点深度值
			if (labels[y * width + x] == novel_pixel_hole)
			{
				float sumC[] = { 0.f, 0.f, 0.f };
				float sumW = 0.f;
				const float sigs = (float)(kradius * 1.2f);
				for (int i = -kradius; i <= kradius; i++)
				{
					for (int j = -kradius; j <= kradius; j++)
					{
						int imy = max(0, min(y + i, height - 1));
						int imx = max(0, min(x + j, width - 1));
						if (labels[imy * width + imx] == novel_pixel_hole)
						{
							continue;
						}
						float dist = sqrtf((float)(i * i + j * j));
						float ws = expf(-dist / sigs);

						float disp = fB / range[imy * width + imx];
						float wd = expf(-fabs(disp - base_param) / sigd);
						//float wd = 1.0;

						for (int c = 0; c < 3; c++)
						{
							float d = img[imy * width * 3 + imx * 3 + c];
							sumC[c] += (ws * wd * d);
						}
						sumW += (ws * wd);
					}
				} // -- disparity based joint bilateral filtering   根据视差图双边滤波
				for (int c = 0; c < 3; c++)
				{
					img[y * width * 3 + x * 3 + c] = sumC[c] / sumW;
				}

			}
		}

	return I2R_OK;
}


int fill_holes_with_background_color__(unsigned char* img, novel_pixel_label_t* labels, novel_pixel_label_t* tlabels, float* range, float* trange,
	float fB, int width, int height, float sigd)
{

	bilateral_hole_filling(img, range, trange, labels, tlabels, width, height, 15, fB, sigd);
	memset(labels, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));
	memset(range, 0, width * height * sizeof(float));

	bilateral_hole_filling(img, trange, range, tlabels, labels, width, height, 15, fB, sigd);
	memset(tlabels, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));
	memset(trange, 0, width * height * sizeof(float));

	bilateral_hole_filling(img, range, trange, labels, tlabels, width, height, 15, fB, sigd);
	memset(labels, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));
	memset(range, 0, width * height * sizeof(float));

	bilateral_hole_filling(img, trange, range, tlabels, labels, width, height, 15, fB, sigd);


	return I2R_OK;
}


void expand_label_pixels(novel_pixel_label_t* ilabels, novel_pixel_label_t* olabels, novel_pixel_label_t tarL, int width, int height, int kradius)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{

			int nearFB = 0;
			for (int i = -kradius; i <= kradius; i++)
			{
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = max(0, min(y + i, height - 1));
					int imx = max(0, min(x + j, width - 1));
					if (ilabels[imy * width + imx] == tarL)
					{
						nearFB = 1;
						break;
					}
				}
			} // -- disparity based joint bilateral filtering
			if (nearFB)
			{
				olabels[y * width + x] = tarL;
			}
			else
			{
				olabels[y * width + x] = ilabels[y * width + x];
			}
		}

}


void smooth_foreground(unsigned char* bgr, unsigned char* obgr, novel_pixel_label_t* labels, int width, int height, int kradius, float sig)
{
	int x, y;
	for (y = 0; y < height; y++)
		for (x = 0; x < width; x++)
		{
			if (labels[y * width + x] != novel_pixel_foreground_boundary)
			{
				for (int c = 0; c < 3; c++)
				{
					obgr[y * width * 3 + x * 3 + c] = bgr[y * width * 3 + x * 3 + c];
				}

				continue;
			}

			float sumC[] = { 0.f, 0.f, 0.f };
			float sumW = 0.f;
			for (int i = -kradius; i <= kradius; i++)
			{
				for (int j = -kradius; j <= kradius; j++)
				{
					int imy = max(0, min(y + i, height - 1));
					int imx = max(0, min(x + j, width - 1));
					float dist = sqrtf((float)(i * i + j * j));
					float w = expf(-dist / sig);
					for (int c = 0; c < 3; c++)
					{
						float d = bgr[imy * width * 3 + imx * 3 + c];
						sumC[c] += (w * d);
					}
					sumW += w;
				}
			}
			sumW += 1e-5f; // -- avoid /0 error
			for (int c = 0; c < 3; c++)
			{
				obgr[y * width * 3 + x * 3 + c] = sumC[c] / sumW;
			}
		}
}

int merge_novel_views(novel_view_t* nvs, int nbr, int width, int height, const float fB,
	float mindepth, float maxdepth, unsigned char* mnv, float*dist, int flag)
{
	unsigned char* mnvdata = NULL, *onvdata = NULL;
	float* mrange = NULL, *orange = NULL;
	novel_pixel_label_t *mlabel = NULL, *expanded_label = NULL;

	mnvdata = (unsigned char*)malloc(width * height * 3 * sizeof(unsigned char));
	onvdata = (unsigned char*)malloc(width * height * 3 * sizeof(unsigned char));
	mrange = (float*)malloc(width * height * sizeof(float));
	orange = (float*)malloc(width * height * sizeof(float));
	mlabel = (novel_pixel_label_t*)malloc(width * height * sizeof(novel_pixel_label_t));
	expanded_label = (novel_pixel_label_t*)malloc(width * height * sizeof(novel_pixel_label_t));

	float maxdisp = fB / mindepth, mindisp = fB / maxdepth;
	float drange = maxdisp - mindisp + 1;
	const float dtr = 0.01666667f; // -- disparity threashold ratio


	memset(mnvdata, 0, width * height * 3 * sizeof(unsigned char));
	memset(mlabel, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));//初始化全是空洞点
	memset(mrange, 0, width * height * sizeof(float));
	//根据深度值对纹理图进行融合，权重加权
	merge_novel_views_mainlayer(nvs, nbr, fB, dtr * drange, width, height, mnvdata, mlabel, mrange, dist, flag);

	//add


	//memcpy(mnvdata, buf_color, width * height * 3 * sizeof(unsigned char));
	// -- post processing: filling holes/smoothing edges
	memset(expanded_label, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));
	memset(orange, 0, width * height * sizeof(float));

	//填补深度空洞
	// -- fill the holes   (1) merge color,  generate mlable and mrange
	//mnvdata 混合后的纹理图merge novel view，填补纹理图空洞
	//expanded_label和orange都没用到
	fill_holes_with_background_color__(mnvdata, mlabel, expanded_label, mrange, orange, fB, width, height, dtr * drange);

	//标记前景边缘
	//(2)lable process
	const int edge_radius = 1;
	memset(mlabel, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));
	label_boundary_pixels2(mrange, mlabel, novel_pixel_foreground_boundary, fB, dtr * drange, edge_radius, width, height, mindepth, maxdepth);
	memset(expanded_label, novel_pixel_hole, width * height * sizeof(novel_pixel_label_t));
	expand_label_pixels(mlabel, expanded_label, novel_pixel_foreground_boundary, width, height, 2); // -- expand foreground boundary
																									////对纹理图的前景边缘进行滤波
																									////(3) smooth
	memset(onvdata, 0, width * height * 3 * sizeof(unsigned char));
	smooth_foreground(mnvdata, onvdata, expanded_label, width, height, 2, 2.f); // -- smooth the foreground edges

																				//// -- copy back to CPU
	memcpy(mnv, mnvdata, width * height * 3 * sizeof(unsigned char));    //copy intermedia result before smooth
	memcpy(mnv, onvdata, width * height * 3 * sizeof(unsigned char));  //copy final result after smooth


	free(mnvdata);
	free(onvdata);
	free(mrange);
	free(orange);
	free(mlabel);
	free(expanded_label);

	return I2R_OK;
}

int blend_two_camera_parameters(krt_CamParam* camp1, krt_CamParam* camp2, const float w1, krt_CamParam *newcamp)
{

	newcamp->src_width = camp1->src_width;
	newcamp->src_height = camp1->src_height;

	const float w2 = 1.f - w1;
	newcamp->krt_WorldPosition[0] = camp1->krt_WorldPosition[0] + w2 * (camp2->krt_WorldPosition[0] - camp1->krt_WorldPosition[0]);
	newcamp->krt_WorldPosition[1] = camp1->krt_WorldPosition[1] + w2 * (camp2->krt_WorldPosition[1] - camp1->krt_WorldPosition[1]);
	newcamp->krt_WorldPosition[2] = camp1->krt_WorldPosition[2] + w2 * (camp2->krt_WorldPosition[2] - camp1->krt_WorldPosition[2]);

	newcamp->krt_kc[0] = w1 * camp1->krt_kc[0] + w2 * camp2->krt_kc[0];
	newcamp->krt_kc[1] = w1 * camp1->krt_kc[1] + w2 * camp2->krt_kc[1];
	newcamp->krt_kc[2] = w1 * camp1->krt_kc[2] + w2 * camp2->krt_kc[2];

	newcamp->krt_cc[0] = w1 * camp1->krt_cc[0] + w2 * camp2->krt_cc[0];
	newcamp->krt_cc[1] = w1 * camp1->krt_cc[1] + w2 * camp2->krt_cc[1];

	newcamp->lens_fov = w1 * camp1->lens_fov + w2 * camp2->lens_fov;
	newcamp->fisheye_radius = w1 * camp1->fisheye_radius + w2 * camp2->fisheye_radius;

	// -- blending for R
	/*
	Mat rotvec;
	Mat R1_to_2 = camp2->krt_R * camp1->krt_R.inv();
	Rodrigues(R1_to_2, rotvec);
	rotvec *= w2;
	Rodrigues(rotvec, R1_to_2);
	newcamp->krt_R = R1_to_2 * camp1->krt_R;
	*/
	int i = 0;
	for (i = 0; i < 9; i++)	 newcamp->krt_R[i] = camp1->krt_R[i];
	return I2R_OK;
}

#define MAX_ITEMS_TO_PARSE  10000
#define DEFAULTCONFIGFILENAME "DIBR.cfg"
#define assert(_Expression) (void)( (!!(_Expression)) || (_wassert(_CRT_WIDE(#_Expression), _CRT_WIDE(__FILE__), __LINE__), 0) )

#if defined WIN32
#include <io.h>
#define strcasecmp strcmpi
#else
#include <unistd.h>
#endif


typedef struct {
	char *TokenName;
	void *Place;
	int Type;
	double Default;
	int param_limits; //! 0: no limits, 1: both min and max, 2: only min (i.e. no negatives), 3: specialcase for QPs since min needs bitdepth_qp_scale
	double min_limit;
	double max_limit;  //we do not use param_limits, min_limit and max_limit.//wkj
} Mapping;


Mapping Map[] = {
	{ "CamNum",         &configinput.CamNum, 0, 30, 0, 0.0, 0.0 },
	{ "SourceWidth",    &configinput.SourceWidth,  0, 1920,         0, 0.0, 0.0 },
	{ "SourceHeight",   &configinput.SourceHeight, 0, 1080,         0, 0.0, 0.0 },

	{ "Color_Path",        &configinput.Color_Path,        1, 0.0,     0, 0.0, 0.0 },
	{ "Depth_Path",        &configinput.Depth_Path,        1, 0.0,     0, 0.0, 0.0 },
	{ "Cam_Params_File",   &configinput.Cam_Params_File,   1, 0.0,     0, 0.0, 0.0 },
	{ "Color_Output_File", &configinput.Color_Output_File, 1, 0.0,     0, 0.0, 0.0 },
	{ "Depth_Output_File", &configinput.Depth_Output_File, 1, 0.0,     0, 0.0, 0.0 },

	{ "Vcam_krt_R_0", &configinput.Vcam_krt_R_0, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_1", &configinput.Vcam_krt_R_1, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_2", &configinput.Vcam_krt_R_2, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_3", &configinput.Vcam_krt_R_3, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_4", &configinput.Vcam_krt_R_4, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_5", &configinput.Vcam_krt_R_5, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_6", &configinput.Vcam_krt_R_6, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_7", &configinput.Vcam_krt_R_7, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_R_8", &configinput.Vcam_krt_R_8, 2, 0.0, 0, 0.0, 0.0 },

	{ "Vcam_krt_WorldPosition_0", &configinput.Vcam_krt_WorldPosition_0, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_WorldPosition_1", &configinput.Vcam_krt_WorldPosition_1, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_WorldPosition_2", &configinput.Vcam_krt_WorldPosition_2, 2, 0.0, 0, 0.0, 0.0 },

	{ "Vcam_krt_kc_0", &configinput.Vcam_krt_kc_0, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_kc_1", &configinput.Vcam_krt_kc_1, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_kc_2", &configinput.Vcam_krt_kc_2, 2, 0.0, 0, 0.0, 0.0 },

	{ "Vcam_krt_cc_0", &configinput.Vcam_krt_cc_0, 2, 0.0, 0, 0.0, 0.0 },
	{ "Vcam_krt_cc_1", &configinput.Vcam_krt_cc_1, 2, 0.0, 0, 0.0, 0.0 },
	{ "lens_fov", &configinput.lens_fov, 2, 0.0, 0, 0.0, 0.0 },
	{ "fisheye_radius", &configinput.fisheye_radius, 2, 0.0, 0, 0.0, 0.0 },

	{ "Vcam_src_width", &configinput.Vcam_src_width, 0, 0, 0, 0.0, 0.0 },
	{ "Vcam_src_height", &configinput.Vcam_src_height, 0, 0, 0, 0.0, 0.0 },
	{ "Vcam_ltype", &configinput.Vcam_ltype, 0, 0, 0, 0.0, 0.0 },
	{ NULL, NULL, -1, 0.0, 0, 0.0, 0.0 }
};


static int ParameterNameToMapIndex(char *s)
{
	int i = 0;

	while (Map[i].TokenName != NULL)
		if (0 == strcasecmp(Map[i].TokenName, s))
			return i;
		else
			i++;
	return -1;
};


void ParseContent(char *buf, int bufsize)
{

	char *items[MAX_ITEMS_TO_PARSE];
	int MapIdx;
	int item = 0;
	int InString = 0, InItem = 0;
	char *p = buf;
	char *bufend = &buf[bufsize];
	int IntContent;
	double DoubleContent;
	int i;

	// Stage one: Generate an argc/argv-type list in items[], without comments and whitespace.
	// This is context insensitive and could be done most easily with lex(1).

	while (p < bufend)
	{
		switch (*p)
		{
		case 13: //回车
			p++;
			break;
		case '#':                 // Found comment
			*p = '\0';              // Replace '#' with '\0' in case of comment immediately following integer or string
			while (*p != '\n' && p < bufend)  // Skip till EOL or EOF, whichever comes first
				p++;
			InString = 0;
			InItem = 0;
			break;
		case '\n':
			InItem = 0;
			InString = 0;
			*p++ = '\0';
			break;
		case ' ':
		case '\t':              // Skip whitespace, leave state unchanged
			if (InString)
				p++;
			else
			{                     // Terminate non-strings once whitespace is found
				*p++ = '\0';
				InItem = 0;
			}
			break;

		case '"':               // Begin/End of String
			*p++ = '\0';
			if (!InString)
			{
				items[item++] = p;
				InItem = ~InItem;
			}
			else
				InItem = 0;
			InString = ~InString; // Toggle
			break;

		default:
			if (!InItem)
			{
				items[item++] = p;
				InItem = ~InItem;
			}
			p++;
		}
	}

	item--;

	for (i = 0; i < item; i += 3)
	{
		if (0 >(MapIdx = ParameterNameToMapIndex(items[i])))
		{
			//snprintf(errortext, ET_SIZE, " Parsing error in config file: Parameter Name '%s' not recognized.", items[i]);
			//error(errortext, 300);
		}
		if (strcasecmp("=", items[i + 1]))
		{
			//snprintf(errortext, ET_SIZE, " Parsing error in config file: '=' expected as the second token in each line.");
			//error(errortext, 300);
		}

		// Now interpret the Value, context sensitive...

		switch (Map[MapIdx].Type)
		{
		case 0:           // Numerical
			if (1 != sscanf(items[i + 2], "%d", &IntContent))
			{
				//snprintf(errortext, ET_SIZE, " Parsing error: Expected numerical value for Parameter of %s, found '%s'.", items[i], items[i + 2]);
				//error(errortext, 300);
			}
			*(int *)(Map[MapIdx].Place) = IntContent;
			printf(".");
			break;
		case 1:
			strncpy((char *)Map[MapIdx].Place, items[i + 2], FILE_NAME_SIZE);
			printf(".");
			break;
		case 2:           // Numerical double
			if (1 != sscanf(items[i + 2], "%lf", &DoubleContent))
			{
				//snprintf(errortext, ET_SIZE, " Parsing error: Expected numerical value for Parameter of %s, found '%s'.", items[i], items[i + 2]);
				//error(errortext, 300);
			}
			*(double *)(Map[MapIdx].Place) = DoubleContent;
			printf(".");
			break;
		default:
			assert("Unknown value type in the map definition of configfile.h");
		}
	}
	memcpy(input, &configinput, sizeof(InputParameters));
}

void no_mem_exit(char *where)
{
	printf("Could not allocate memory: %s", where);
	//error(errortext, 100);
}

char *GetConfigFileContent(char *Filename)
{
	long FileSize;
	FILE *f;
	char *buf;

	if (NULL == (f = fopen(Filename, "r")))
	{
		printf("Cannot open configuration file %s.");
		return NULL;
	}

	if (0 != fseek(f, 0, SEEK_END))
	{
		printf("Cannot fseek in configuration file %s.");
		return NULL;
	}

	FileSize = ftell(f);
	if (FileSize < 0 || FileSize > 60000)
	{
		printf("Unreasonable Filesize %ld reported by ftell for configuration file %s.");
		return NULL;
	}
	if (0 != fseek(f, 0, SEEK_SET))
	{
		printf("Cannot fseek in configuration file %s.");
		return NULL;
	}

	if ((buf = malloc(FileSize + 1)) == NULL) no_mem_exit("GetConfigFileContent: buf");

	// Note that ftell() gives us the file size as the file system sees it.  The actual file size,
	// as reported by fread() below will be often smaller due to CR/LF to CR conversion and/or
	// control characters after the dos EOF marker in the file.

	FileSize = fread(buf, 1, FileSize, f);
	buf[FileSize] = '\0';


	fclose(f);
	return buf;
}

void Configure(int ac, char *av[])
{
	char *content;
	int CLcount, ContentLen, NumberParams;
	char *filename = DEFAULTCONFIGFILENAME;

	memset(&configinput, 0, sizeof(InputParameters));
	//Set default parameters.
	printf("Setting Default Parameters...\n");
	//InitEncoderParams();  //no default parameters, wkj

	// Process default config file
	CLcount = 1;

	if (ac == 2)
	{
		if (0 == strncmp(av[1], "-h", 2))
		{
			//JMHelpExit();
		}
	}

	if (ac >= 3)
	{
		if (0 == strncmp(av[1], "-d", 2))
		{
			// filename = av[2];
			filename = "./cfg/v10.cfg"; //郭帅
			CLcount = 3;
		}
		if (0 == strncmp(av[1], "-h", 2))
		{
			//JMHelpExit();
		}
	}
	printf("Parsing Configfile %s", filename);
	content = GetConfigFileContent(filename);
	//if (NULL == content)
	//error(errortext, 300);
	ParseContent(content, strlen(content));
	printf("\n");
	free(content);
}


void setVirCamParam(krt_CamParam* vcam, InputParameters* input, int width, int height)
{
	vcam->krt_R[0] = input->Vcam_krt_R_0;
	vcam->krt_R[1] = input->Vcam_krt_R_1;
	vcam->krt_R[2] = input->Vcam_krt_R_2;
	vcam->krt_R[3] = input->Vcam_krt_R_3;
	vcam->krt_R[4] = input->Vcam_krt_R_4;
	vcam->krt_R[5] = input->Vcam_krt_R_5;
	vcam->krt_R[6] = input->Vcam_krt_R_6;
	vcam->krt_R[7] = input->Vcam_krt_R_7;
	vcam->krt_R[8] = input->Vcam_krt_R_8;

	vcam->krt_WorldPosition[0] = input->Vcam_krt_WorldPosition_0;
	vcam->krt_WorldPosition[1] = input->Vcam_krt_WorldPosition_1;
	vcam->krt_WorldPosition[2] = input->Vcam_krt_WorldPosition_2;

	vcam->krt_kc[0] = input->Vcam_krt_kc_0;
	vcam->krt_kc[1] = input->Vcam_krt_kc_1;
	vcam->krt_kc[2] = input->Vcam_krt_kc_2;

	vcam->krt_cc[0] = input->Vcam_krt_cc_0;
	vcam->krt_cc[1] = input->Vcam_krt_cc_1;

	vcam->lens_fov = input->lens_fov;
	vcam->fisheye_radius = input->fisheye_radius;
	//vcam->src_width = input->Vcam_src_width;
	vcam->src_width = width;
	//vcam->src_height = input->Vcam_src_height;
	vcam->src_height = height;
	vcam->ltype = input->Vcam_ltype;
	//-------------------

	//gs begin add
	//vcam->lens_fov = 10;
	//vcam->fisheye_radius = 2000;
	FILE *fp = NULL;
	fp = fopen("para_vcamera_1004_diff_noresize.txt", "r");
	char temps[100];
	int tempid;

	fscanf(fp, "%s %d\n", temps, &tempid);

	fscanf(fp, "%s %d %d\n", temps, &vcam->src_width, &vcam->src_height);

	for (int tempi = 0; tempi < 9; tempi++)
		vcam->krt_K[tempi] = 0;
	fscanf(fp, "%s %f %f %f %f\n", temps, &vcam->krt_K[0], &vcam->krt_K[4], &vcam->krt_K[2], &vcam->krt_K[5]);//K

	vcam->krt_cc[0] = vcam->krt_K[2];
	vcam->krt_cc[1] = vcam->krt_K[5];

	fscanf(fp, "%s", temps);
	for (int tempi = 0; tempi < 9; tempi++)
		fscanf(fp, " %f", &vcam->krt_R[tempi]);//R
	fgetc(fp);

	fscanf(fp, "%s %f %f %f\n", temps, &vcam->krt_WorldPosition[0], &vcam->krt_WorldPosition[1], &vcam->krt_WorldPosition[2]);//T
	fclose(fp);
	//gs end add

}
// New algorithm, updated in Aug, 2020.
void selectTwoView_new(krt_CamParam* vcam, int* camID, float* dist)
{
	int cam_1_number = -1;
	int cam_2_number = -1;

	float dist1 = 0, dist2 = 0;


	float min_dist = 1000000000000.f;

	for (int i = 0; i < CamNum; i++)  //select first camera
	{
		if (i == 10) continue;  //for debug, should be deleted, since for debug we use camera 1 as vitual camera  
		float pos_diff_[3] =
		{
			vcam->krt_WorldPosition[0] - krt_camparam[i].krt_WorldPosition[0],
			vcam->krt_WorldPosition[1] - krt_camparam[i].krt_WorldPosition[1],
			vcam->krt_WorldPosition[2] - krt_camparam[i].krt_WorldPosition[2]
		};
		dist1 = norm(pos_diff_);
		//printf("wp0: %f, %f, %f\n", vcam->krt_WorldPosition[0], vcam->krt_WorldPosition[1], vcam->krt_WorldPosition[2]);
		//printf("krt: %f, %f, %f\n", krt_camparam[i].krt_WorldPosition[0], krt_camparam[i].krt_WorldPosition[1], krt_camparam[i].krt_WorldPosition[2]);
		//printf("dist %d -> %f\n",i,dist);
		if (dist1 <= min_dist)
		{
			min_dist = dist1;
			cam_1_number = i;
		}
	}
	dist1 = min_dist;
	min_dist = 1000000000000.f;

	for (int i = 0; i < CamNum; i++)  //select second camera
	{
		if (i == 10) continue;  //for debug, should delete, since for debug we use camera 1 as vitual camera
		if (i == cam_1_number) continue;
		float vector_VcamRcam1[3] =
		{
			vcam->krt_WorldPosition[0] - krt_camparam[cam_1_number].krt_WorldPosition[0],
			vcam->krt_WorldPosition[1] - krt_camparam[cam_1_number].krt_WorldPosition[1],
			vcam->krt_WorldPosition[2] - krt_camparam[cam_1_number].krt_WorldPosition[2]
		};
		float vector_Rcam2Rcam1[3] =
		{
			krt_camparam[i].krt_WorldPosition[0] - krt_camparam[cam_1_number].krt_WorldPosition[0],
			krt_camparam[i].krt_WorldPosition[1] - krt_camparam[cam_1_number].krt_WorldPosition[1],
			krt_camparam[i].krt_WorldPosition[2] - krt_camparam[cam_1_number].krt_WorldPosition[2]
		};
		float product1 = vector_VcamRcam1[0] * vector_Rcam2Rcam1[0] + vector_VcamRcam1[1] * vector_Rcam2Rcam1[1] + vector_VcamRcam1[2] * vector_Rcam2Rcam1[2];
		if (product1 < 0) continue;
		float vector_Rcam2Vcam[3] =
		{
			krt_camparam[i].krt_WorldPosition[0] - vcam->krt_WorldPosition[0],
			krt_camparam[i].krt_WorldPosition[1] - vcam->krt_WorldPosition[1],
			krt_camparam[i].krt_WorldPosition[2] - vcam->krt_WorldPosition[2]
		};
		float product2 = vector_Rcam2Vcam[0] * vector_Rcam2Rcam1[0] + vector_Rcam2Vcam[1] * vector_Rcam2Rcam1[1] + vector_Rcam2Vcam[2] * vector_Rcam2Rcam1[2];
		if (product2 < 0) continue;
		dist2 = norm(vector_Rcam2Vcam);
		if (dist2 < min_dist)
		{
			min_dist = dist2;
			cam_2_number = i;
		}
		if (cam_2_number == -1 && i == CamNum - 1)
		{
			cam_2_number = cam_1_number;
			dist2 = dist1;
		}
	}
	dist2 = min_dist;

	dist[0] = dist1;

	dist[1] = dist2;

	camID[0] = cam_1_number;

	camID[1] = cam_2_number;
}

void selectTwoView(krt_CamParam* vcam, int* camID, float* dist, float*test, int *flag)
{
	int cam_1_number = -1;
	int cam_2_number = -1;

	float min_dist = 1000000000000.f;

	for (int i = 0; i < CamNum; i++)  //select first camera
	{
		//if (i == 7 || i == 8 || i==6 || i==9 ) continue;  //for debug, should be deleted, since for debug we use camera 1 as vitual camera  
		float pos_diff_[3] =
		{
			vcam->krt_WorldPosition[0] - krt_camparam[i].krt_WorldPosition[0],
			vcam->krt_WorldPosition[1] - krt_camparam[i].krt_WorldPosition[1],
			vcam->krt_WorldPosition[2] - krt_camparam[i].krt_WorldPosition[2]
		};
		float dist = norm(pos_diff_);
		//printf("wp0: %f, %f, %f\n", vcam->krt_WorldPosition[0], vcam->krt_WorldPosition[1], vcam->krt_WorldPosition[2]);
		//printf("krt: %f, %f, %f\n", krt_camparam[i].krt_WorldPosition[0], krt_camparam[i].krt_WorldPosition[1], krt_camparam[i].krt_WorldPosition[2]);
		//printf("dist %d -> %f\n",i,dist);
		if (dist <= min_dist)
		{
			min_dist = dist;
			cam_1_number = i;
		}
	}
	float d1 = min_dist;

	min_dist = 1000000000000.f;
	for (int i = 0; i < CamNum; i++)  //select second camera
	{
		//if (i == 6 || i == 8 || i==9 ||  i==7 ) continue;  //for debug, should delete, since for debug we use camera 1 as vitual camera
		if (i == cam_1_number) continue;
		float pos_diff_[3] =
		{
			vcam->krt_WorldPosition[0] - krt_camparam[i].krt_WorldPosition[0],
			vcam->krt_WorldPosition[1] - krt_camparam[i].krt_WorldPosition[1],
			vcam->krt_WorldPosition[2] - krt_camparam[i].krt_WorldPosition[2]
		};
		float dist = norm(pos_diff_);

		float vector_2_1[3] = {
			krt_camparam[i].krt_WorldPosition[0] - krt_camparam[cam_1_number].krt_WorldPosition[0],
			krt_camparam[i].krt_WorldPosition[1] - krt_camparam[cam_1_number].krt_WorldPosition[1],
			krt_camparam[i].krt_WorldPosition[2] - krt_camparam[cam_1_number].krt_WorldPosition[2]
		};
		float dist_2 = norm(vector_2_1);

		float vector_0_1[3] = {
			vcam->krt_WorldPosition[0] - krt_camparam[cam_1_number].krt_WorldPosition[0],
			vcam->krt_WorldPosition[1] - krt_camparam[cam_1_number].krt_WorldPosition[1],
			vcam->krt_WorldPosition[2] - krt_camparam[cam_1_number].krt_WorldPosition[2]
		};
		float dot_mul = vector_2_1[0] * vector_0_1[0] + vector_2_1[1] * vector_0_1[1] + vector_2_1[2] * vector_0_1[2];
		float intersection_point[3];
		intersection_point[0] = dot_mul / (dist_2 * dist_2) * vector_2_1[0] + krt_camparam[cam_1_number].krt_WorldPosition[0];
		intersection_point[1] = dot_mul / (dist_2 * dist_2) * vector_2_1[1] + krt_camparam[cam_1_number].krt_WorldPosition[1];
		intersection_point[2] = dot_mul / (dist_2 * dist_2) * vector_2_1[2] + krt_camparam[cam_1_number].krt_WorldPosition[2];

		float pos_diff_0[3] = {
			intersection_point[0] - krt_camparam[cam_1_number].krt_WorldPosition[0],
			intersection_point[1] - krt_camparam[cam_1_number].krt_WorldPosition[1],
			intersection_point[2] - krt_camparam[cam_1_number].krt_WorldPosition[2]
		};
		float dist_0 = norm(pos_diff_0);

		float pos_diff_1[3] = {
			intersection_point[0] - krt_camparam[i].krt_WorldPosition[0],
			intersection_point[1] - krt_camparam[i].krt_WorldPosition[1],
			intersection_point[2] - krt_camparam[i].krt_WorldPosition[2]
		};

		float dist_1 = norm(pos_diff_1);

		test[0] = dist_0; test[1] = dist_1; test[2] = dist_2;//test

		if (fabs(dist_0 + dist_1 - dist_2) < 0.00001f) {
			if (dist <= min_dist) {
				min_dist = dist;
				cam_2_number = i;
				//break;
			}

		}
		else if (i == CamNum - 1) {
			*flag = 1;
			cam_2_number = 0;
		}
	}

	float d2 = min_dist;

	camID[0] = cam_1_number;
	camID[1] = cam_2_number;
	dist[0] = d1; dist[1] = d2;

}