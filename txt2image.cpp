#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <queue>
#include <iostream>
#include <cstdlib>
#include <math.h>
#include <ctime>

#pragma warning (disable: 4996)

using namespace std;
using namespace cv;

int main()
{
	int height, width, x;
	char tempchar;
	int height1 = 1082, height3 = 1082;
	int width1 = 1938, width3 = 1938;
	char inputname[20][50] = { "hlabel1.txt", "hlabel3.txt", "hrange1.txt", "hrange3.txt", "mdepth1.txt", "mdepth3.txt", "mlabel1.txt", "mlabel3.txt", "out_buf_depth.txt", "warp1.txt", "warp3.txt"};
	char outputname[20][50] = { "hlabel1.png", "hlabel3.png", "hrange1.png", "hrange3.png", "mdepth1.png", "mdepth3.png", "mlabel1.png", "mlabel3.png", "out_buf_depth.png", "warp1.png", "warp3.png"};
	//int heights[10] = { 1080, 1080, 1080, 1080, 1080, 1080, 1080, 1080, 1080 };
	//int widths[10] = { 1920, 1920, 1920, 1920, 1920, 1920, 1920, 1920, 1920 };
	int heights[20] = { height1, height3, height1, height3, max(height1, height3), max(height1, height3), max(height1, height3), max(height1, height3), max(height1, height3), max(height1, height3), max(height1, height3) };
	int widths[20] = { width1, width3, width1, width3, max(width1, width3), max(width1, width3), max(width1, width3), max(width1, width3), max(width1, width3), max(width1, width3), max(width1, width3) };

	for (int i = 0; i < 9; i++)
	{
		printf("%d\n", i);
		height = heights[i];
		width = widths[i];
		Mat label_output(height, width, CV_8UC1);
		FILE *tempfile = fopen(inputname[i], "r");
		float temp;

		for (int di = 0; di < height; di++)
		{
			for (int dj = 0; dj < width; dj++)
			{
				uchar* uc_pixel = label_output.data + di*label_output.step + dj;
				fscanf(tempfile, "%f ", &temp);
				uc_pixel[0] = (int)temp;
			}
			fscanf(tempfile, "\c", &tempchar);
		}
		imwrite(outputname[i], label_output);
	}

	for (int i = 9; i < 11; i++)
	{
		printf("%d\n", i);
		height = heights[i];
		width = widths[i];
		Mat label_output(height, width, CV_8UC3);
		FILE *tempfile = fopen(inputname[i], "r");
		for (int di = 0; di < height; di++)
		{
			for (int dj = 0; dj < width; dj++)
			{
				uchar* uc_pixel = label_output.data + di*label_output.step + dj * 3;
				fscanf(tempfile, "%d %d %d ", &uc_pixel[2], &uc_pixel[0], &uc_pixel[1]);
			}
			fscanf(tempfile, "\c", &tempchar);
		}
		imwrite(outputname[i], label_output);
	}

	return 0;
}