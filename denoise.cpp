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

struct node
{
	int nodex, nodey;
}tempnode;
bool visited[2000][1200];

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
			nowx = min(nowx, h-1);
			nowy = min(nowy, w-1);
			//printf("nowx=%d, nowy=%d\n", nowx, nowy);

			if (visited[nowx][nowy])
			{
				//printf("visited!\n");
				continue;
			}

			uchar* color_input_pixel = color_input.data + nowx*color_input.step + 3 * nowy;
			int b = color_input_pixel[0], g = color_input_pixel[1], r = color_input_pixel[2];
			if (fabs(b - g) <= 20 && fabs(g - r) <= 20 && fabs(b - r) <= 20 && (b + g + r) > 150) continue;

			uchar* depth_pixel_now = depth_input.data + nowx*depth_input.step + nowy;
			if (depth_pixel_now[0] <= 40)
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

void denoise(Mat color_input, Mat depth_input, Mat depth_output)
{
	int w = depth_input.cols, h = depth_input.rows;
	if (w != color_input.cols || h != depth_input.rows)
	{
		printf("Color input size and depth input size don't match");
		return;
	}

	for (int i = 260; i < 990; i++)
	{
		//printf("%d\n", i);
		for (int j = 800; j < 1270; j++)
		{
			uchar* depth_input_pixel = depth_input.data + i*depth_input.step + j;
			uchar* depth_output_pixel = depth_output.data + i*depth_output.step + j;
			uchar* color_input_pixel = color_input.data + i*color_input.step + 3 * j;
			int b = color_input_pixel[0], g = color_input_pixel[1], r = color_input_pixel[2];

			if (fabs(b - g) <= 20 && fabs(g - r) <= 20 && fabs(b - r) <= 20 && (b + g + r) > 150) continue;
			if (depth_output_pixel[0] <= 40)
			{
				//cout << i << ' ' << j << endl;
				BFS(color_input, depth_input, depth_output, i, j);
			}
		}
	}

	return;
}

int main()
{
	Mat color_input, depth_input, depth_output;
	char color_input_file[100], depth_input_file[100], depth_output_file[100];

	srand(time(NULL));
	for (int i = 0; i < 3; i++)
	{
		printf("%d\n", i);
		sprintf(color_input_file, "%s\\%d.png", "color_100_0_123", i);
		sprintf(depth_input_file, "%s\\%d.png", "depth_100_0_123", i);
		sprintf(depth_output_file, "%s\\%d.png", "depth_test", i);

		color_input = imread(color_input_file);
		depth_input = imread(depth_input_file, 0);
		depth_input.copyTo(depth_output);

		denoise(color_input, depth_input, depth_output);

		imwrite(depth_output_file, depth_output);
	}

	return 0;
}