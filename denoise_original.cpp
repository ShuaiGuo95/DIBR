#include <iostream>
#include <stdio.h>
#include <queue>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#pragma warning(disable : 4996)

using namespace std;
using namespace cv;

struct mynode
{
	int nodei, nodej;
};
bool flag[2000][1200];

uchar BFS(Mat color_input, Mat depth_input, Mat depth_output, int h, int w, int i, int j)
{
	mynode nodetemp, nodetop;
	queue<mynode> queuen;
	
	bool ok = false;

	memset(flag, 0, sizeof(flag));

	while (!queuen.empty()) queuen.pop();
	nodetemp.nodei = i;
	nodetemp.nodej = j;
	queuen.push(nodetemp);
	flag[i][j] = true;

	while (!queuen.empty())
	{
		nodetop = queuen.front();
		queuen.pop();

		for (int tempi = -1; tempi <= 1; tempi++)
		{
			for (int tempj = -1; tempj <= 1; tempj++)
			{
				if (tempi == 0 && tempj == 0) continue;

				int reali = max(0, nodetop.nodei + tempi), realj = max(0, nodetop.nodej + tempj);
				reali = min(reali, h-1);
				realj = min(realj, w-1);

				if (flag[reali][realj]) continue;

				uchar* pixel_temp = depth_input.data + reali*depth_input.step + realj;
				nodetemp.nodei = reali;
				nodetemp.nodej = realj;
				queuen.push(nodetemp);
				flag[reali][realj] = true;

				if (pixel_temp[0] == 0)
					continue;
				else
				{
					int distance; 
					uchar r1, r2, g1, g2, b1, b2;
					uchar* color1 = color_input.data + i*color_input.step + j*3;
					uchar* color2 = color_input.data + reali*color_input.step + realj*3;
					r1 = color1[0]; g1 = color1[1]; b1 = color1[2];
					r2 = color2[0]; g2 = color2[1]; b2 = color2[2];
					distance = (r1 - r2)*(r1 - r2) + (g1 - g2)*(g1 - g2) + (b1 - b2)*(b1 - b2);

					if (distance < 1200)
					{
						return pixel_temp[0];
					}
				}
			}
		}
	}

	return 0;
}

uchar background_search(Mat color_input, Mat depth_input, int h, int w, int i, int j)
{
	int uprecord[50], upi = i, upave = 0;
	int dwrecord[50], dwi = i, dwave = 0;
	int b, g, r;
	int ans = 0, ansup = 0, ansdw = 0, numcount = 20;
	uchar* pixel_depth;
	uchar* pixel_color;
	bool flagup = false, flagdw = false;

	memset(uprecord, 0, sizeof(uprecord));
	memset(dwrecord, 0, sizeof(dwrecord));

	for (int upnode = i, tempj = 0; upnode>= numcount; upnode--)
	{
		upi = upnode;
		//cout << upnode << endl;
		for (tempj = 0; tempj < numcount; tempj++)
		{
			pixel_depth = depth_input.data + (upnode-tempj)*depth_input.step + j;
			uprecord[tempj] = pixel_depth[0];
			pixel_color = color_input.data + (upnode-tempj)*color_input.step + j*3;
			b = pixel_color[0];
			g = pixel_color[1];
			r = pixel_color[2];
			if (!uprecord[tempj] || uprecord[tempj]>160 || fabs(b - g) > 20 || fabs(g - r) > 20 || fabs(b - r) > 20)
			{
				//cout << "break 1 " << tempj << ' ' << (int)uprecord[tempj] << ' ' << b << ' ' << g << ' ' << r << endl;
				//cout << upnode - tempj << ' ' << j << endl;
				break;
			}
		}
		if (tempj == numcount)
		{
			//cout << "break 1 true " << tempj << ' ' << numcount << endl;
			flagup = true;
			break;
		}
	}
	
	for (int dwnode = i, tempj = 0; dwnode < 870-numcount; dwnode++)
	{
		dwi = dwnode;
		for (tempj = 0; tempj < numcount; tempj++)
		{
			pixel_depth = depth_input.data + (dwnode + tempj)*depth_input.step + j;
			dwrecord[tempj] = pixel_depth[0];
			pixel_color = color_input.data + (dwnode + tempj)*color_input.step + j*3;
			b = pixel_color[0];
			g = pixel_color[1];
			r = pixel_color[2];
			if (!dwrecord[tempj] || dwrecord[tempj]>160 || fabs(b - g) > 20 || fabs(g - r) > 20 || fabs(b - r) > 20)
			{
				//cout << "break 2 " << tempj << ' ' << b << ' ' << g << ' ' << r << endl;
				//cout << dwnode + tempj << ' ' << j << endl;
				break;
			}
		}
		if (tempj == numcount)
		{
			flagdw = true;
			break;
		}
	}

	for (int tempi = 0; tempi < numcount; tempi++)
	{
		ansup += uprecord[tempi];
		ansdw += dwrecord[tempi];
	}
	ansup = ansup / numcount;
	//cout << ansup << endl;
	ansup = ansup + (i - upi) / 32;
	ansdw = ansdw / numcount;
	//cout << ansdw << endl;
	ansdw = ansdw - (dwi - i) / 32;
	//cout << i << ' ' << upi << ' ' << dwi << ' ' << j << endl;
	//cout << "ansup=" << ansup << ' ' << "ansdw=" << ansdw << endl;
	//cout << flagup << ' ' << flagdw << endl;
	//getchar();

	if (!flagup && !flagdw)
		return (uchar)(0);
	else if (!flagup)
		return  (uchar)ansdw;
	else if (!flagdw)
		return (uchar)ansup;

	if (fabs(ansdw - ansup) > 50)
		return (uchar)(min(ansup, ansdw));
	else
		return (uchar)((ansup + ansdw) / 2);
}

void denoise(Mat color_input, Mat depth_input, Mat depth_output, int index)
{
	int w = color_input.cols, h = color_input.rows;
	if (w != depth_input.cols || h != depth_input.rows || w != depth_output.cols || h != depth_output.rows)
	{
		printf("Color frame and depth frame have different size");
		return;
	}

	int i, j;
	int leftj = 400 + index * 50, rightj = 1200 + index * 60; // leftj = 400 + index*50 rightj = 1200 + index*60, min(1200 + index * 110, w)
	int b, g, r;
	bool isbackground;
	uchar tempixel;
	for (i = 400; i < 900; i++)
	{
		uchar* pixel_depth = depth_output.data + i*depth_output.step;
		uchar* pixel_color = color_input.data + i*color_input.step;
		//printf("%d\n", i);
		for (j = leftj, pixel_depth += j, pixel_color += 3*j; j < rightj; j++)
		{
			b = pixel_color[0];
			g = pixel_color[1];
			r = pixel_color[2];
			if (pixel_depth[0] == 0)
			{
				//tempixel = BFS(color_input, depth_input, depth_output, h, w, i, j);

				if (fabs(b - g) <= 13 && fabs(g - r) <= 13 && fabs(b - r) <= 13 && (r+g+b)>150)
				{
					pixel_depth[0] = background_search(color_input, depth_input, h, w, i, j);
					//cout << i << ' ' << "BFS " << (int)(pixel_depth[0]) << endl;
				}
				else
				{
					pixel_depth[0] = BFS(color_input, depth_input, depth_output, h, w, i, j);
					//cout << i << ' ' << "search " << (int)(pixel_depth[0]) << endl;
				}

			}
			pixel_depth += 1;
			pixel_color += 3;
		}
		//cout << endl;
	}
	return;
}

int main()
{
	char color_input_dir[100] = ".\\color_1004_diff_noresize\\";
	char depth_input_dir[100] = ".\\depth_1004_diff_noresize\\";
	char color_input_file_name[100];
	char depth_input_file_name[100];

	char color_output_dir[100] = ".\\color_test\\";
	char depth_output_dir[100] = ".\\depth_test\\";
	char color_output_file_name[100];
	char depth_output_file_name[100];

	char wh_file_input[200];
	char wh_file_output[200];

	FILE *wh_input, *wh_output;
	int pixel_w, pixel_h;

	Mat color_input, depth_input;
	Mat color_output, depth_output;

	sprintf(wh_file_input, "%sresolutions.txt", depth_input_dir);
	if ((wh_input = fopen(wh_file_input, "rb")) == NULL) { printf("cannot open file %s\n", wh_file_input); }
	sprintf(wh_file_output, "%sresolutions.txt", depth_output_dir);
	if ((wh_output = fopen(wh_file_output, "w")) == NULL) { printf("cannot open file %s\n", wh_file_output); }

	for (int i = 0; i < 12; i++)
	{
		// 读入
		printf("%d\n", i);
		fscanf(wh_input, "%d %d\n", &pixel_w, &pixel_h);
		fprintf(wh_output, "%d %d\n", pixel_w, pixel_h);
		sprintf(color_input_file_name, "%s%d.png", color_input_dir, i);
		sprintf(depth_input_file_name, "%s%d.png", depth_input_dir, i);
		color_input = imread(color_input_file_name, 1);
		depth_input = imread(depth_input_file_name, 0);
		color_output = color_input.clone();
		depth_output = depth_input.clone();

		//draw_color(color_output);
		//draw_depth(depth_output);
		//est_color(color_output);
		//test_depth(depth_output);
		denoise(color_input, depth_input, depth_output, i);

		// 输出
		sprintf(color_output_file_name, "%s%d.png", color_output_dir, i);
		sprintf(depth_output_file_name, "%s%d.png", depth_output_dir, i);
		//imwrite(color_output_file_name, color_output);
		imwrite(depth_output_file_name, depth_output);
	}

	fclose(wh_input);
	fclose(wh_output);

	return 0;
}