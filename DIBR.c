/*****************************************************************************

*This software is developed within IEEE 1857.9 subgroup for the reference test model of Immersive Video Content Coding.
*Contributor: kaijin wei, kaijin.wkj@alibaba-inc.com,  Alibaba Group.

*****************************************************************************/

/***

单个投影的加速
原来：单个投影 15秒，合成  1.6秒
需要加速到：单个投影  0.15秒，合成0.016秒
加速100倍


多个投影的并行

***/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stdafx.h"
#include "math.h"
#include "dibr.h"
#include "time.h"

int main(int argc, char** argv)
{
	int pixel_w = 0, pixel_h = 0;

	FILE *color_file_input;
	FILE *depth_file_input;

	novel_view_t  nvs[REF_CAM_N];

	Configure(argc, argv);   //get patameters from configure file

	printf("Input image width = %d, height = %d\n", input->SourceWidth, input->SourceHeight);
	//----------initialize input parameter----------
	// gs begin change
	FILE *kp = NULL;
	kp = fopen("./depth_1004_diff_noresize/resolutions.txt", "r");
	int i, j;
	for (i = 0; i < 12; i++)
	{
		// krt_camparam[i].src_width  = input->SourceWidth ;
		// krt_camparam[i].src_height = input->SourceHeight;
		fscanf(kp, "%d %d\n", &krt_camparam[i].src_width, &krt_camparam[i].src_height);
	}
	pixel_w = input->SourceWidth;
	pixel_h = input->SourceHeight;
	CamNum = input->CamNum;  //should be placed before init_with_sfm_file, since init_with_sfm_file will use CamNum

	init_with_sfm_file(input->Cam_Params_File); //v10.cfg是虚拟视角的参数，camps-max-min-depth.sfm是所有相机的参数
	FILE *depth_file_output = fopen(input->Depth_Output_File, "wb");
	FILE *color_file_output = fopen(input->Color_Output_File, "wb");
	printf("%s %s\n", input->Depth_Output_File, input->Color_Output_File);


	printf("output rgb >>> %s \n", input->Color_Output_File);
	float prefW[2] = { 1.0, 1.0 };
	float fB = 32504.0;   //80000  90000  120000
	float maxdisp = fB / mindepth;
	float mindisp = fB / maxdepth;

	//--------------------------------initialize virtual camera-------------------------
	krt_CamParam krt_camparam_temp;
	krt_CamParam* vcam = &krt_camparam_temp;
	//	krt_CamParam* vcam = &krt_camparam[1];//for debug

	//---------------------------------------------------------
	// do_view_selection(camps, X, MAX_MVS_CAMNUM, &algop);  //select two nearest camera according to virtual camera, we only use two for interpolation
	int cam12[2] = { 0, 0 };
	// find two nearest camera. In fact, this selecting approach cannot work well when the input cameras are not evenly placed on a circle.
	float dist[2] = { 0.f,0.f };
	float test[3] = { 0.f,0.f,0.f };
	int* flag = 0;
	//selectTwoView(vcam, &cam12[0],&dist[0],&test[0],&flag);
	//selectTwoView_new(vcam, &cam12[0], &dist[0]);
	// 可以手动修改值
	cam12[0] = 5;
	cam12[1] = 7;

	int maxwidth = max(krt_camparam[cam12[0]].src_width, krt_camparam[cam12[1]].src_width);
	int maxheight = max(krt_camparam[cam12[0]].src_height, krt_camparam[cam12[1]].src_height);

	// 将input中读入的虚拟视点的参数导入到vcam中
	setVirCamParam(vcam, input, maxwidth, maxheight); //vcam是虚拟视角

	printf("cam1 = %d\n", cam12[0]);
	printf("cam2 = %d\n", cam12[1]);
	printf("dist1 = %.3f\n", dist[0]);
	printf("dist2 = %.3f\n", dist[1]);
	printf("%.6f %.6f %.6f\n", test[0], test[1], test[2]);//test

	int cam_1_number = cam12[0];
	int cam_2_number = cam12[1];

	//---------------------------------------------------------
	//scam_1_number = 0;
	//cam_2_number = 2; // debug;

	//allocate input color and depth buffer
	// unsigned char *in_buf_color = malloc(pixel_w * pixel_h * 3 * sizeof(unsigned char)); // gs begin 挪到后面了
	// revise w h to w/2 h/2
#ifdef DEPTH_DOWNSAMPLE
	unsigned char* in_buf_depth = malloc(pixel_w / 2 * pixel_h / 2 * 1 * sizeof(unsigned char));
	//allocate intermediate depth buffer
	float * in_depth_float = malloc(pixel_w * pixel_h * sizeof(float));
#else
	// unsigned char *in_buf_depth = malloc(pixel_w * pixel_h * 1 * sizeof(unsigned char)); // gs begin 挪到后面了
	// float* in_img_depth_float = malloc(pixel_w * pixel_h * sizeof(float)); // gs begin  挪到后面了
#endif // DEPTH_DOWNSAMPLE




	//allocate output buffer
	unsigned char *out_buf_color = malloc(maxwidth * maxheight * 3 * sizeof(unsigned char));
	unsigned char *out_buf_depth = malloc(maxwidth * maxheight * 1 * sizeof(unsigned char));


	init_lanczos_filter(); // ??????   tbl_lanczos_coef,一个比较好的插值算法
#ifdef USE_BACKGROUND
						   // consider camera 10 as vitural viewpoint
	FILE* read_bg_color1 = fopen("./bg/color/9.yuv", "rb");
	FILE* read_bg_color2 = fopen("./bg/color/11.yuv", "rb");
	FILE* read_bg_depth1 = fopen("./bg/depth/range-cam9-mmdp-20.00-3000.00-maxsvnbr-7-cam9.yuv", "rb");
	FILE* read_bg_depth2 = fopen("./bg/depth/range-cam11-mmdp-20.00-3000.00-maxsvnbr-7-cam11.yuv", "rb");
	unsigned char* bg_color1 = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char) * 3);
	unsigned char* bg_color2 = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char) * 3);
	unsigned char* bg_depth1 = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));
	unsigned char* bg_depth2 = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));

	fread(bg_color1, 1, pixel_w * pixel_h * 3, read_bg_color1);
	fread(bg_color2, 1, pixel_w * pixel_h * 3, read_bg_color2);
	fread(bg_depth1, 1, pixel_w * pixel_h, read_bg_depth1);
	fread(bg_depth2, 1, pixel_w * pixel_h, read_bg_depth2);
#endif

	printf("--------------begin to process DIBR -----------------------------------  \n");
	//int sss_[2] = {0,1};
	for (int sss = 0; sss < REF_CAM_N; sss++)
	{
		// 得到当前邻域图像在全部输入中的真实的编号，比如5-6， 8-9这种
		int re_index_input = cam12[sss];
		pixel_w = krt_camparam[re_index_input].src_width;
		pixel_h = krt_camparam[re_index_input].src_height;

		unsigned char *in_buf_color = malloc(pixel_w * pixel_h * 3 * sizeof(unsigned char));
		unsigned char *in_buf_depth = malloc(pixel_w * pixel_h * 1 * sizeof(unsigned char));
		float* in_img_depth_float = malloc(pixel_w * pixel_h * sizeof(float));

		clock_t start, finish;
		start = clock();
		//int sss = sss_[sssi];
		//allocate various buffer
		// 我姑且认为，这个是邻域图投影在中间图的缓冲
		novel_view_t *nv = &nvs[sss];
		nv->mrange_img = malloc(maxwidth * maxheight * sizeof(float));                 // depth  深度图
		nv->mlabel_img = malloc(maxwidth * maxheight * sizeof(novel_pixel_label_t));   // mask?? 前景边缘标记图
		nv->mnovel_view = malloc(maxwidth * maxheight * 3 * sizeof(unsigned char));    // rgb   纹理图
		nv->mconf = malloc(maxwidth * maxheight * sizeof(float));

		//initialize various buffer
		memset(nv->mrange_img, 0, maxwidth * maxheight * sizeof(float)); // 初始化值全部为0
		memset(nv->mlabel_img, 0, maxwidth * maxheight * sizeof(novel_pixel_label_t));
		memset(nv->mnovel_view, novel_pixel_hole, maxwidth * maxheight * 3 * sizeof(unsigned char));
		memset(nv->mconf, 0, maxwidth * maxheight * sizeof(float));
		//可以预先设置每个参考相机的权重比例，此时每个参考相机的权重设置为1
		nv->prefW = prefW[sss]; // ??? s

		char color_input_file_name[FILE_NAME_SIZE];
		char depth_input_file_name[FILE_NAME_SIZE];
		sprintf(color_input_file_name, "%s\\%d.yuv", input->Color_Path, re_index_input);
		//sprintf(depth_input_file_name, "%s\\range-cam%d-mmdp-20.00-3000.00-maxsvnbr-7-cam%d.yuv", input->Depth_Path, re_index_input, re_index_input);
		sprintf(depth_input_file_name, "%s\\%d.yuv", input->Depth_Path, re_index_input, re_index_input);
		printf("%s, %s\n", color_input_file_name, depth_input_file_name);
		//read color from file
		if ((color_file_input = fopen(color_input_file_name, "rb")) == NULL) { printf("cannot ---open this file\n");	return -1; }
		if (fread(in_buf_color, 1, pixel_w*pixel_h * 3, color_file_input) != pixel_w*pixel_h * 3)
		{
			fseek(color_file_input, 0, SEEK_SET);
			fread(in_buf_color, 1, pixel_w*pixel_h * 3, color_file_input);
		}// Loop  reading




#ifdef DEPTH_DOWNSAMPLE
		 //color to grey
		unsigned char* greycolor = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));
		for (j = 0; j < pixel_h; j++)
		{
			for (i = 0; i < pixel_w; i++)
			{
				int r = in_buf_color[j * pixel_w * 3 + i * 3 + 0];
				int g = in_buf_color[j * pixel_w * 3 + i * 3 + 1];
				int b = in_buf_color[j * pixel_w * 3 + i * 3 + 2];
				float grey = 0.2989 * r + 0.5870 * g + 0.1140 * b;
				greycolor[j * pixel_w + i] = (unsigned char)grey;
			}
		}

		//read depth from file
		if ((depth_file_input = fopen(depth_input_file_name, "rb")) == NULL) { printf("cannot open this file\n");	return -1; }
		if (fread(in_buf_depth, 1, pixel_w / 2 * pixel_h / 2, depth_file_input) != pixel_w / 2 * pixel_h / 2)
		{
			fseek(depth_file_input, 0, SEEK_SET);
			fread(in_buf_depth, 1, pixel_w / 2 * pixel_h / 2, depth_file_input);
		}
		unsigned char* depth2x = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));
		int THR = 2;
		for (j = 0; j < pixel_h; j++)
		{
			for (i = 0; i < pixel_w; i++)
			{
				if (j % 2 == 0 && i % 2 == 0)
				{
					int grey = in_buf_depth[j / 2 * pixel_w / 2 + i / 2];
					depth2x[j * pixel_w + i] = grey;
				}
				else if (j % 2 == 0 && i % 2 != 0)
				{
					int depthL = in_buf_depth[j / 2 * pixel_w / 2 + i / 2];
					int depthR = in_buf_depth[j / 2 * pixel_w / 2 + min(959, i / 2 + 1)];
					int pixC = greycolor[j * pixel_w + i];
					int pixL = greycolor[j * pixel_w + max(0, i - 1)];
					int pixR = greycolor[j * pixel_w + min(1919, i + 1)];
					if (abs(pixC - pixR) < abs(pixC - pixL) / THR)
						depth2x[j * pixel_w + i] = depthR;
					else if (abs(pixC - pixL) < abs(pixC - pixR) / THR)
						depth2x[j * pixel_w + i] = depthL;
					else
						depth2x[j * pixel_w + i] = max(depthL, depthR);
				}

			}
		}
		for (j = 0; j < pixel_h; j++)
		{
			for (i = 0; i < pixel_w; i++)
			{
				if (j % 2 != 0)
				{
					int depthU = depth2x[max(0, j - 1) * pixel_w + i];
					int depthD = depth2x[min(1079, j + 1) * pixel_w + i];
					int pixC = greycolor[j * pixel_w + i];
					int pixU = greycolor[max(0, j - 1) * pixel_w + i];
					int pixD = greycolor[min(1079, j + 1) * pixel_w + i];
					if (abs(pixC - pixD) < abs(pixC - pixU) / THR)
						depth2x[j * pixel_w + i] = depthD;
					else if (abs(pixC - pixU) < abs(pixC - pixD) / THR)
						depth2x[j * pixel_w + i] = depthU;
					else
						depth2x[j * pixel_w + i] = max(depthU, depthD);
				}
			}
		}
		for (j = 0; j < pixel_h; j++)
		{
			for (i = 0; i < pixel_w; i++)
			{
				in_depth_float[j * pixel_w + i] = (float)((float)depth2x[j * pixel_w + i] / 255.f * (maxdisp - mindisp) + mindisp);
				in_depth_float[j * pixel_w + i] = (float)(fB / (float)in_depth_float[j * pixel_w + i]);
			}
		}
		float* in_img_depth_float = (float*)malloc(pixel_w * pixel_h * sizeof(float));
		int x, y;
		//median filter
		int kradius = 2;
		for (y = 0; y < pixel_h; y++)
			for (x = 0; x < pixel_w; x++)
			{
				in_img_depth_float[y * pixel_w + x] = in_depth_float[y * pixel_w + x];
				{
					float depth_window[25];
					float medium_value = 0;
					for (int i = -kradius; i <= kradius; i++)
					{
						for (int j = -kradius; j <= kradius; j++)
						{
							int imy = max(0, min(y + i, pixel_h - 1));  //make sure the index (x+j, y+i) is located in the image.
							int imx = max(0, min(x + j, pixel_w - 1)); // -- clamp
																	   //depth_window[] = [imy * width + imx];
							depth_window[(i + kradius) * (2 * kradius + 1) + (j + kradius)] = in_depth_float[imy * pixel_w + imx];
						}
					}
					medium_value = find_medium(depth_window, 9);
					in_img_depth_float[y * pixel_w + x] = medium_value;
				}
			}
#else
		if ((depth_file_input = fopen(depth_input_file_name, "rb")) == NULL) { printf("cannot open this file\n");	return -1; }
		if (fread(in_buf_depth, 1, pixel_w*pixel_h, depth_file_input) != pixel_w*pixel_h)
		{
			fseek(depth_file_input, 0, SEEK_SET);
			fread(in_buf_depth, 1, pixel_w*pixel_h, depth_file_input);
		}// Loop  reading
#ifdef USE_BACKGROUND
		unsigned char* mask = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));
		memset(mask, 0, sizeof(unsigned char)* pixel_w* pixel_h);
		if (sss == 0)
		{
			for (j = 0; j < pixel_h; j++)
			{
				for (i = 0; i < pixel_w; i++)
				{
					if (abs(bg_color1[j * pixel_w * 3 + i * 3 + 0] - in_buf_color[j * pixel_w * 3 + i * 3 + 0]) >= BACKGROUND_THRE || abs(bg_color1[j * pixel_w * 3 + i * 3 + 1] - in_buf_color[j * pixel_w * 3 + i * 3 + 1]) >= BACKGROUND_THRE || abs(bg_color1[j * pixel_w * 3 + i * 3 + 2] - in_buf_color[j * pixel_w * 3 + i * 3 + 2]) >= BACKGROUND_THRE)
					{
						mask[j * pixel_w + i] = 255;
					}
				}
			}
			unsigned char* range_out = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));
			memset(range_out, 0, sizeof(unsigned char) * pixel_w * pixel_h);
			for (j = 0; j < pixel_h; j++)
			{
				for (i = 0; i < pixel_w; i++)
				{
					if (mask[j * pixel_w + i] == 255)
					{
						range_out[j * pixel_w + i] = in_buf_depth[j * pixel_w + i];
					}
					else
					{
						range_out[j * pixel_w + i] = bg_depth1[j * pixel_w + i];
					}
				}
			}
			memcpy(in_buf_depth, range_out, sizeof(unsigned char)* pixel_w* pixel_h);
			// choose whether output refined depth or not. Modified in Aug 2020.
			FILE* output_depth_1 = fopen("./bg/depth_revise/depth-revise9.yuv", "wb");
			fwrite(range_out, 1, pixel_w * pixel_h, output_depth_1);
		}
		if (sss == 1)
		{
			memset(mask, 0, sizeof(unsigned char) * pixel_w * pixel_h);
			for (j = 0; j < pixel_h; j++)
			{
				for (i = 0; i < pixel_w; i++)
				{
					if (abs(bg_color2[j * pixel_w * 3 + i * 3 + 0] - in_buf_color[j * pixel_w * 3 + i * 3 + 0]) >= BACKGROUND_THRE || abs(bg_color2[j * pixel_w * 3 + i * 3 + 1] - in_buf_color[j * pixel_w * 3 + i * 3 + 1]) >= BACKGROUND_THRE || abs(bg_color2[j * pixel_w * 3 + i * 3 + 2] - in_buf_color[j * pixel_w * 3 + i * 3 + 2]) >= BACKGROUND_THRE)
					{
						mask[j * pixel_w + i] = 255;
					}
				}
			}
			unsigned char* range_out2 = (unsigned char*)malloc(pixel_w * pixel_h * sizeof(unsigned char));
			memset(range_out2, 0, sizeof(unsigned char) * pixel_w * pixel_h);
			for (j = 0; j < pixel_h; j++)
			{
				for (i = 0; i < pixel_w; i++)
				{
					if (mask[j * pixel_w + i] == 255)
					{
						range_out2[j * pixel_w + i] = in_buf_depth[j * pixel_w + i];
					}
					else
					{
						range_out2[j * pixel_w + i] = bg_depth2[j * pixel_w + i];
					}
				}
			}
			memcpy(in_buf_depth, range_out2, sizeof(unsigned char)* pixel_w* pixel_h);
			// choose whether output refined depth or not. Modified in Aug 2020.
			FILE* output_depth_2 = fopen("./bg/depth_revise/depth-revise11.yuv", "wb");
			fwrite(range_out2, 1, sizeof(unsigned char) * pixel_w * pixel_h, output_depth_2);
		}



#endif // 
		printf("maxdisp=%f, mindisp=%f, fB=%f, mindepth=%f, maxdepth=%f\n", maxdisp, mindisp, fB, mindepth, maxdepth);
		// maxdisp=1625.199951, mindisp=32.504002, fB=32504.000000, mindepth=20.000000, maxdepth=1000.000000
		// 这一块可以并行，可能没有必要
		//gs begin
		maxdisp = fB / krt_camparam[re_index_input].maxdepth;
		mindisp = fB / krt_camparam[re_index_input].mindepth;
		printf("maxdepth = %f, mindepth = %f\n", krt_camparam[re_index_input].maxdepth, krt_camparam[re_index_input].mindepth);
		//gs end
		for (j = 0; j < pixel_h; j++)
		{
			for (i = 0; i < pixel_w; i++)
			{
				in_img_depth_float[j*pixel_w + i] = (float)((float)in_buf_depth[j*pixel_w + i] / 255.f * (maxdisp - mindisp) + mindisp);
				in_img_depth_float[j*pixel_w + i] = (float)(fB / (float)in_img_depth_float[j*pixel_w + i]);
				// in_img_depth_float[j*pixel_w + i] = (float)in_buf_depth[j*pixel_w + i];
			}
		}
#endif // DEPTH_DOWNSAMPLE
		//上面的loop是为了确保读到一张图


		//convert depth from fixed-point disparity into float-point range 
		//---------range image and range image have inverse relationship.----------------



		//initialize ，上面已经初始化过了
		//memset(nv->mrange_img, 0, pixel_w * pixel_h * sizeof(float));
		//memset(nv->mlabel_img, novel_pixel_hole, pixel_w * pixel_h * sizeof(novel_pixel_label_t));

		finish = clock();
		double totaltime = (double)(finish - start) / CLOCKS_PER_SEC;

		printf("One pre time = %f\n", totaltime);

		start = clock();
		// krt_camparam是各个相机的参数, re_index_input是参考视角序列号, vcam是虚拟相机参数
		gen_novel_view(&krt_camparam[re_index_input], in_buf_color, in_img_depth_float, vcam, nv, fB, mindepth, maxdepth, sss);     //output is nv->mrange_img and nv->mlabel_img 
		if (sss == 0)
		{
			FILE *color_file_output = fopen("./results/warp1.yuv", "wb", re_index_input);
			fwrite(nv->mnovel_view, 1, pixel_w * pixel_h * 3, color_file_output);
		}
		if (sss == 1)
		{
			FILE* color_file_output = fopen("./results/warp3.yuv", "wb", re_index_input);
			fwrite(nv->mnovel_view, 1, pixel_w * pixel_h * 3, color_file_output);
		}


		finish = clock();
		totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
		printf("One warp time = %f\n", totaltime);
		printf("*****************\n");


	}

	clock_t st, fi;
	st = clock();
	//nvs新视点的指针，ref-参考视点的个数=2
	//merge的时候会对前景边缘进行smooth_foreground，对前后左右半径内的bgr取平均
	merge_novel_views(nvs, REF_CAM_N, maxwidth, maxheight, fB, mindepth, maxdepth, out_buf_color, dist, flag);
	fi = clock();
	printf("merge time = %f\n", (double)(fi - st) / CLOCKS_PER_SEC);

	int select_cam = 0;  //select  1  as output depth
	maxdisp = fB / mindepth;
	mindisp = fB / maxdepth;
	//printf("%f %f !!!!!!!!!!!!!!!\n", mindepth, maxdepth);
	for (j = 0; j < maxheight; j++)    //convert float-point range depth into fixed-point disparity depth
		for (i = 0; i < maxwidth; i++)
		{
			nvs[select_cam].mrange_img[j*maxwidth + i] = (float)(fB / nvs[select_cam].mrange_img[j*maxwidth + i]);
			out_buf_depth[j*maxwidth + i] = (unsigned char)((nvs[select_cam].mrange_img[j*maxwidth + i] - mindisp) * 255.f / (maxdisp - mindisp));

		}
	fwrite(out_buf_depth, 1, maxwidth * maxheight, depth_file_output);
	fwrite(out_buf_color, 1, maxwidth * maxheight * 3, color_file_output);

	fclose(depth_file_input);
	fclose(color_file_input);

	fclose(depth_file_output);
	fclose(color_file_output);

	char command[100] = "ffmpeg -s ";
	char cwidth[10], cheight[10], name[10];
	itoa(maxwidth, cwidth, 10);
	itoa(maxheight, cheight, 10);
	itoa(cam12[0] + 1, name, 10);

	strcat(command, cwidth);
	strcat(command, "x");
	strcat(command, cheight);
	strcat(command, " -pix_fmt rgb24 -i ./results/color_100.yuv ./results/");
	strcat(command, name);
	strcat(command, ".png -y");
	printf("%s\n", command);
	system(command);

	printf("--------------processing  is  completed-----------------------------  \n");
	printf("%d %d\n", maxwidth, maxheight);
}

