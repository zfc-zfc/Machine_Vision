#include <stdlib.h>
#include <cv.h>
#include <math.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#define LINEAR_X 0
using namespace cv;
using namespace std;
const double PI = acos(-1.0);//反三角函数

//机器视觉作业：圆测量算法实现
//最小二乘法拟合圆
void LeastSquareFittingCircle(vector<Point2f> temp_coordinates)//利用opencv solve函数的高斯消元算法（LU分解）解方程组
{
	float x1 = 0;
	float x2 = 0;
	float x3 = 0;
	float y1 = 0;
	float y2 = 0;
	float y3 = 0;
	float x1y1 = 0;
	float x1y2 = 0;
	float x2y1 = 0;
	int num;
	vector<Point2f>::iterator k;
	Point3f tempcircle;
	num = temp_coordinates.size();
	for (k = temp_coordinates.begin(); k != temp_coordinates.end(); k++)
	{
 
		x1 = x1 + (*k).x;
		x2 = x2 + (*k).x * (*k).x;
		x3 = x3 + (*k).x * (*k).x * (*k).x;
		y1 = y1 + (*k).y;
		y2 = y2 + (*k).y * (*k).y;
		y3 = y3 + (*k).y * (*k).y * (*k).y;
		x1y1 = x1y1 + (*k).x * (*k).y;
		x1y2 = x1y2 + (*k).x * (*k).y * (*k).y;
		x2y1 = x2y1 + (*k).x * (*k).x * (*k).y;
	}
	Mat left_matrix = (Mat_<float>(3,3) << x2, x1y1, x1, x1y1, y2, y1, x1, y1, num);
	//cout << "left_matrix=" << left_matrix << endl;
	Mat right_matrix = (Mat_<float>(3, 1) << -(x3 + x1y2), -(x2y1 + y3), -(x2 + y2));
	//cout << "right_matrix=" << right_matrix << endl;
	Mat solution(3,1,CV_32F);
	solve(left_matrix, right_matrix, solution,CV_LU);
	//cout << "solution=" << solution << endl;
	float a, b, c;
	a = solution.at<float>(0);
	b = solution.at<float>(1);
	c = solution.at<float>(2);
	tempcircle.x = -a / 2; //圆心x坐标
	tempcircle.y = -b / 2;//圆心y坐标
	tempcircle.z = sqrt(a*a + b*b - 4 * c) / 2;//圆心半径
	printf("拟合圆心row：%f,拟合圆心column：%f,拟合圆心半径：%f\n\n\n",tempcircle.y,tempcircle.x,tempcircle.z);
}
int main(int argc, char **argv)
{

    VideoCapture capture;
        capture.open(0);//打开 zed 相机

	ROS_WARN("*****START");
	ros::init(argc,argv,"trafficLaneTrack");
        ros::NodeHandle n;

    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开\n");
        return 0;
    }
	waitKey(1000);
    Mat frame;
	while (ros::ok())
	{
        clock_t begin,finish;
        begin=clock();
        Mat frGray = imread("/home/fangcheng/Library/brake_disk_part.png",0);
        imshow("frGray",frGray);
        Mat frThreshold=frGray.clone();
        cv::threshold(frGray, frThreshold, 250, 255, THRESH_BINARY);//第三个参数为阈值
        imshow("frThreshold",frThreshold);

//1.大致确定圆心位置找轮廓
	    vector<vector<Point> > contours;
	    vector<Vec4i> hireachy;
        Mat result_img = Mat::zeros(frThreshold.size(), CV_8UC3);     // 创建与原图同大小的黑色背景
        Point circle_center;              //定义圆心坐标
        float radius;
	    findContours(frThreshold, contours, hireachy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point());
	    for (int t = 0; t < contours.size(); ++t)
	    {
		    // 面积过滤
		    double area = contourArea(contours[t]);     //计算点集所围区域的面积
		    if (area < 10000||area>250000)            //选出轮廓面积大于10000的轮廓
				continue;
			
		    Rect rect = boundingRect(contours[t]);            // 求点集的最小直立外接矩形
			drawContours(result_img, contours, t, Scalar(0, 0, 255), -1, 8, Mat(), 0, Point());    //画出圆
			int x = rect.x + rect.width / 2;
		    int y = rect.y + rect.height / 2;
			circle_center = Point(x, y);          //得到圆心坐标
			printf("Area of the circle: %f\n", area);  //面积
            printf("Coordinates of the center point：row %d, column %d \n",circle_center.y,circle_center.x);
			circle(result_img, circle_center, 2, Scalar(0, 255, 255), 2, 8, 0);  
            radius=sqrt(area/PI);
            printf("Radius computed with area : %f\n \n", radius);//由面积计算半径
	    }
	    imshow("result_img", result_img);
		Mat frCircle = Mat::zeros(result_img.size(), CV_8UC3);
		cvtColor(result_img, frCircle, CV_RGB2GRAY);//RGB彩色图转换成Gray灰度图
		cv::threshold(frCircle, frCircle, 15, 255, THRESH_BINARY);//第三个参数为阈值
		imshow("frCircle", frCircle);

//2.利用大概圆心的位置生成一个圆环（内外圆半径可以手动设置）
        Mat frRing=result_img.clone();
        cv::circle(frRing, circle_center, (int)radius+8, cv::Scalar(0, 255, 0));
        cv::circle(frRing, circle_center, (int)radius-8, cv::Scalar(255, 0, 0));
        imshow("frRing",frRing);

//3.在圆环的内外圆之间每隔一定的角度（可以自己设定）生成扫描线
		Point Inside[365];//内环2通道数组用于保存扫描线起点坐标
		Point Outside[365];//外环2通道数组用于保存扫描线终点坐标
		for(int i=0;i<frRing.rows;i++){
        	for(int j=0;j<frRing.cols;j++){
				Scalar color = frRing.at<Vec3b>(i, j); //Blue,Green,Red顺序
				if (color(0)>250 && color(1)<10 && color(2)<10 && j!=circle_center.x){ //内环,蓝色
					float theta_inside=atan2((i-circle_center.y),(j-circle_center.x))/PI*180;
					if (abs(theta_inside-(int)theta_inside)<0.2){
						Inside[(int)theta_inside+180]=Point(j,i);
						//printf("内环坐标Inside[%d]=%d %d, 角度%f\n",(int)theta_inside+180,i,j,theta_inside);
					}
				}
				if (color(0)<10 && color(1)>250 && color(2)<10 && j!=circle_center.x){ //外环,绿色
					float theta_outside=atan2((i-circle_center.y),(j-circle_center.x))/PI*180;
					if (abs(theta_outside-(int)theta_outside)<0.2){
						Outside[(int)theta_outside+180]=Point(j,i);
						//printf("外环坐标Outside[%d]=%d %d, 角度%f\n ",(int)theta_outside+180,i,j,theta_outside);
					}
				}
			}
		}
		Mat frScan=frRing.clone();
		for(int k=0;k<360;k++)
		{
			if(Inside[k].x>1 && Inside[k].y>1 && Outside[k].x>1 && Outside[k].y>1)
				cv::line(frScan, Inside[k], Outside[k], cv::Scalar(0, 180, 180));
		}
		imshow("frScan",frScan);
		//扫描线上各点的非整数坐标
		float Line_x[360][20]={0.0};//最多360根扫描线,一根最多包含20个点，保存各点横坐标（列数)
		float Line_y[360][20]={0.0};//最多360根扫描线,一根最多包含20个点，保存各点横坐标（行数）
		float BiLinear[360][20]={0.0};
		for(int k=0;k<360;k++)
		{
			if(Inside[k].x>1 && Inside[k].y>1 && Outside[k].x>1 && Outside[k].y>1){
				for(int t=0;t<16;t++){
					Line_x[k][t]=Inside[k].x+t*cos(atan2((Inside[k].y-circle_center.y),(Inside[k].x-circle_center.x)));
					Line_y[k][t]=Inside[k].y+t*sin(atan2((Inside[k].y-circle_center.y),(Inside[k].x-circle_center.x)));
					//printf("第 %d 条扫描线，第 %d 个点的非整数坐标：row:%f,column:%f\n",k,t,Line_y[k][t],Line_x[k][t]);
					//计算扫描线上各点灰度值（双线性插值）
					//x方向插值
					float temp_x1=((int)Line_x[k][t]+1-Line_x[k][t])*frGray.at<uchar>((int)Line_y[k][t]+1,(int)Line_x[k][t])+(Line_x[k][t]-(int)Line_x[k][t])*frGray.at<uchar>((int)Line_y[k][t]+1,(int)Line_x[k][t]+1);
					float temp_x2=((int)Line_x[k][t]+1-Line_x[k][t])*frGray.at<uchar>((int)Line_y[k][t],(int)Line_x[k][t])+(Line_x[k][t]-(int)Line_x[k][t])*frGray.at<uchar>((int)Line_y[k][t],(int)Line_x[k][t]+1);
					//y方向插值,得到扫描线上各非整数坐标点的灰度值
					BiLinear[k][t]=(Line_y[k][t]-(int)Line_y[k][t])*temp_x1+((int)Line_y[k][t]+1-Line_y[k][t])*temp_x2;
					//printf("第 %d 条扫描线，第 %d 个点,非整数坐标为%f,%f　的灰度值 : %f\n",k,t,Line_x[k][t],Line_y[k][t],BiLinear[k][t]);
				}
			}
		}
		imshow("frScan",frScan);


//4.在扫描线上计算亚像素边缘点坐标
		vector<Point2f> eage(360);
		float derivative[360][20]={0.000};//扫描线上各点一阶倒数值
		for(int k=0;k<360;k++)
		{
			float max=0.0;
			float eage_position_x;
			float eage_position_y;
			if(Inside[k].x>1 && Inside[k].y>1 && Outside[k].x>1 && Outside[k].y>1){
				for(int t=1;t<15;t++){
					derivative[k][t]=0.5*(BiLinear[k][t-1]-BiLinear[k][t+1]);
					if(abs(derivative[k][t])>max){
						max=abs(derivative[k][t]);
						eage_position_x=Line_x[k][t];
						eage_position_y=Line_y[k][t];
					}
				}
				//printf("第 %d 条扫描线，亚像素精度边缘点坐标为　row:%f,column:%f　\n",k,eage_position_y, eage_position_x);
				eage[k] = Point2f((float)eage_position_y, (float)eage_position_x);
			}
		}
		printf("======================================　\n");

//5.利用圆拟合方法拟合出圆方程，给出圆的圆心位置和直径
		LeastSquareFittingCircle(eage);
		ros::spinOnce();
		waitKey(1000);
	}
	return 0;
}