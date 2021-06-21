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

struct node
{
	int nodex, nodey;
}tempnode;
bool visited[2000][2000];
bool forground_mark[2000][2000];

int edgebegin[12] = {};
int edgeend[12] = {};

int * getRandom(int N)
{
	static int  r[10];
	int *arr = (int*)malloc(N * sizeof(int));
	int count = 0;
	memset(arr, 0, N * sizeof(int));

	while (count<N)
	{
		int val = rand() % N;
		if (!arr[val])
		{
			arr[val] = 1;
			r[count] = val;
			++count;
		}
	}
	return r;
}

void background_replace(Mat color_input, Mat depth_input, Mat background_color, Mat background_depth)
{
	int h = depth_input.rows, w = depth_input.cols;
	memset(forground_mark, 1, sizeof(forground_mark));
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			uchar* input_depth = depth_input.data + i*depth_input.step + j;
			uchar* input_color = color_input.data + i*color_input.step + j * 3;
			uchar* bg_depth = background_depth.data + i*background_depth.step + j;
			uchar* bg_color = background_color.data + i*background_color.step + j * 3;
			int in_b = input_color[0], in_g = input_color[1], in_r = input_color[2];
			int bg_b = bg_color[0], bg_g = bg_color[1], bg_r = bg_color[2];

			if ((in_b - bg_b)*(in_b - bg_b) + (in_g - bg_g)*(in_g - bg_g) + (in_r - bg_r)*(in_r - bg_r) < 300 && (in_b - bg_b)<20 && (in_g - bg_g)<20 && (in_r - bg_r)<20)
			{
				// 当前像素是背景，用模板替换掉
				input_depth[0] = 125; //bg_depth[0]
				forground_mark[i][j] = false;
			}
		}
	}
}

void BFS(Mat color_input, Mat depth_input, Mat depth_output, int locate_x, int locate_y)
{
	int h = depth_input.rows, w = depth_input.cols;
	uchar* depth_pixel_locate = depth_output.data + locate_x*depth_output.step + locate_y;
	int *stepnow;
	int stepx[10] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	int stepy[10] = { -1, 0, 1, -1, 1, -1, 0, 1 };

	memset(visited, 0, sizeof(visited));
	queue<node> queue;
	while (!queue.empty()) queue.pop();

	tempnode.nodex = locate_x;
	tempnode.nodey = locate_y;
	queue.push(tempnode);
	visited[locate_x][locate_y] = true;
	while (!queue.empty())
	{
		tempnode = queue.front();
		queue.pop();
		//printf("\ntop.nodex=%d, top.nodey=%d\n", tempnode.nodex, tempnode.nodey);

		stepnow = getRandom(8);
		/*for (int i = 0; i < 8; i++)
		printf("%d ", stepnow[i]);
		printf("\n");*/
		for (int di = 0; di < 8; di++)
		{
			int dx = stepx[stepnow[di]];
			int dy = stepy[stepnow[di]];
			//printf("dx=%d, dy=%d\n", dx, dy);

			int nowx = max(0, tempnode.nodex + dx), nowy = max(0, tempnode.nodey + dy);
			nowx = min(nowx, h - 1);
			nowy = min(nowy, w - 1);
			//printf("nowx=%d, nowy=%d\n", nowx, nowy);

			if (visited[nowx][nowy] || !forground_mark[nowx][nowy])
			{
				//if (visited[nowx][nowy]) printf("visited\n");
				//else printf("not forground\n");
				continue;
			}

			uchar* color_input_pixel = color_input.data + nowx*color_input.step + 3 * nowy;
			int b = color_input_pixel[0], g = color_input_pixel[1], r = color_input_pixel[2];
			if (fabs(b - g) <= 20 && fabs(g - r) <= 20 && fabs(b - r) <= 20 && (b + g + r) > 150) continue;

			uchar* depth_pixel_now = depth_input.data + nowx*depth_input.step + nowy;
			if (depth_pixel_now[0] < 40)
			{
				//printf("pushback %d %d\n", nowx, nowy);
				visited[nowx][nowy] = 1;
				node nownode = { nowx, nowy };
				queue.push(nownode);
			}
			else
			{
				//printf("return %d\n", depth_pixel_now[0]);
				depth_pixel_locate[0] = depth_pixel_now[0];
				//getchar();
				return;
			}
		}
	}
	return;
}

uchar background_search(Mat color_input, Mat depth_input, int h, int w, int i, int j)
{
	int uprecord[50], upi = i, upave = 0;
	int dwrecord[50], dwi = i, dwave = 0;
	int b, g, r;
	int ans = 0, ansup = 0, ansdw = 0, numcount = 10;
	uchar* pixel_depth;
	uchar* pixel_color;
	bool flagup = false, flagdw = false;

	memset(uprecord, 0, sizeof(uprecord));
	memset(dwrecord, 0, sizeof(dwrecord));

	for (int upnode = i, tempj = 0; upnode >= numcount; upnode--)
	{
		upi = upnode;
		//cout << upnode << endl;
		for (tempj = 0; tempj < numcount; tempj++)
		{
			pixel_depth = depth_input.data + (upnode - tempj)*depth_input.step + j;
			uprecord[tempj] = pixel_depth[0];
			pixel_color = color_input.data + (upnode - tempj)*color_input.step + j * 3;
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

	for (int dwnode = i, tempj = 0; dwnode < 870 - numcount; dwnode++)
	{
		dwi = dwnode;
		for (tempj = 0; tempj < numcount; tempj++)
		{
			pixel_depth = depth_input.data + (dwnode + tempj)*depth_input.step + j;
			dwrecord[tempj] = pixel_depth[0];
			pixel_color = color_input.data + (dwnode + tempj)*color_input.step + j * 3;
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
	ansup = ansup + (i - upi) / 5;
	ansdw = ansdw / numcount;
	ansdw = ansdw - (dwi - i) / 5;


	if (!flagup && !flagdw)
		return (uchar)(0);
	else if (!flagup)
		return  (uchar)ansdw;
	else if (!flagdw)
		return (uchar)ansup;

	if (fabs(ansdw - ansup) > 30)
		return (uchar)(min(ansup, ansdw));
	else
		return (uchar)((ansup + ansdw) / 2);
}

uchar down_search(Mat depth_input, int h, int w, int i, int j, int edge)
{
	for (int di = i; di < h; di++)
	{
		uchar* pixel_depth = depth_input.data + di*depth_input.step + j;
		if (pixel_depth[0] > edge) return (uchar)(pixel_depth[0]);
	}
	return (uchar)0;
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
	int leftj = 0 + index * 80, rightj = min(1400 + index * 80, w); // leftj = 400 + index*50 rightj = 1200 + index*60, min(1200 + index * 110, w)
	int b, g, r;
	bool isbackground;
	uchar tempixel;
	for (i = 0; i < h; i++) // 350-900
	{
		//printf("%d\n", i);
		for (j = 0; j < w; j++)
		{
			//if (j == 1299 && i == 492) getchar();
			uchar* pixel_depth = depth_output.data + i*depth_output.step + j;
			if (forground_mark[i][j])
			{
				if (pixel_depth[0] < 40)
				{
					BFS(color_input, depth_input, depth_output, i, j);
					//getchar();
					//if (j == 751 && i == 453)  cout << (int)pixel_depth[0] << endl;
				}
				if (pixel_depth[0] < 40)
				{
					pixel_depth[0] = down_search(depth_input, h, w, i, j, 40);
				}
			}
			else if (!pixel_depth[0]) //如果经过BFS还为0，或者背景为0，则从上下方向找代替吧，没辙了
			{
				pixel_depth[0] = down_search(depth_input, h, w, i, j, 0);
			}
			//if (j == 947 && i == 609) getchar();
			/*uchar* pixel_depth = depth_output.data + i*depth_output.step + j;
			uchar* pixel_color = color_input.data + i*color_input.step + j * 3;
			b = pixel_color[0];
			g = pixel_color[1];
			r = pixel_color[2];
			if (pixel_depth[0] == 0)
			{
				//tempixel = BFS(color_input, depth_input, depth_output, h, w, i, j);

				if (fabs(b - g) <= 15 && fabs(g - r) <= 15 && fabs(b - r) <= 15 && (r + g + b)>150)
				{
					//continue;
					pixel_depth[0] = background_search(color_input, depth_input, h, w, i, j);
					//cout << i << ' ' << "BFS " << (int)(pixel_depth[0]) << endl;
				}
				else
				{
					//BFS(color_input, depth_input, depth_output, i, j);
					//cout << i << ' ' << j << " search " << (int)(pixel_depth[0]) << endl;
				}
			}
			if (fabs(b - g) > 15 || fabs(g - r) > 15 || fabs(b - r) > 15 || (r + g + b)<=150) //去除人物
			{
				pixel_depth[0] = background_search(color_input, depth_input, h, w, i, j);
			}*/
		}
		//cout << endl;
	}
	return;
}

int main()
{
	char color_background_dir[100] = ".\\background_color\\";
	char depth_background_dir[100] = ".\\background_depth\\";

	char color_input_dir[100] = ".\\color_100_0_200_same\\";
	char depth_input_dir[100] = ".\\depth_100_0_200_same\\";
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

	Mat color_input, depth_input, background_color, background_depth;
	Mat color_output, depth_output;

	//sprintf(wh_file_input, "%sresolutions.txt", depth_input_dir);
	//if ((wh_input = fopen(wh_file_input, "rb")) == NULL) { printf("cannot open file %s\n", wh_file_input); }
	sprintf(wh_file_output, "%sresolutions.txt", depth_output_dir);
	if ((wh_output = fopen(wh_file_output, "w")) == NULL) { printf("cannot open file %s\n", wh_file_output); }

	for (int i = 0; i < 12; i++)
	{
		// 读入
		printf("%d\n", i);
		//fscanf(wh_input, "%d %d\n", &pixel_w, &pixel_h);

		sprintf(color_input_file_name, "%s%d.png", color_input_dir, i);
		sprintf(depth_input_file_name, "%s%d.png", depth_input_dir, i);
		color_input = imread(color_input_file_name, 1);
		depth_input = imread(depth_input_file_name, 0);

		sprintf(color_input_file_name, "%s%d.png", color_background_dir, i);
		sprintf(depth_input_file_name, "%s%d.png", depth_background_dir, i);
		background_color = imread(color_input_file_name, 1);
		background_depth = imread(depth_input_file_name, 0);

		fprintf(wh_output, "%d %d\n", depth_input.cols, depth_input.rows);

		background_replace(color_input, depth_input, background_color, background_depth);

		color_output = color_input.clone();
		depth_output = depth_input.clone();
		denoise(color_input, depth_input, depth_output, i);

		// 输出
		//sprintf(color_output_file_name, "%s%d.png", color_output_dir, i);
		sprintf(depth_output_file_name, "%s%d.png", depth_output_dir, i);
		//imwrite(color_output_file_name, color_output);
		imwrite(depth_output_file_name, depth_output);
		//getchar();
	}

	//fclose(wh_input);
	fclose(wh_output);

	return 0;
}