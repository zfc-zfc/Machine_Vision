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

//机器视觉作业：单层NCC模板匹配
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
	waitKey(100);
    Mat frame;
	while (ros::ok())
	{
        clock_t begin,finish;
        begin=clock();
        Mat Temp = imread("/home/fangcheng/Library/pattern.bmp",0);
        Mat Img = imread("/home/fangcheng/Library/IMAGEB34.bmp",0);
        imshow("Temp",Temp);
        imshow("Img",Img);
        double mt=0.0000;
        double st_square=0.0000;
        double sum_mt=0.0000;
        double sum_st=0.0000;
        int n=Temp.rows*Temp.cols;
        //模板平均灰度
        for(int i=0;i<Temp.rows;i++){
            for(int j=0;j<Temp.cols;j++){
                sum_mt+=Temp.at<uchar>(i,j);
            }
        }
        mt=(double)sum_mt/n;
        //模板方差
        for(int i=0;i<Temp.rows;i++){
            for(int j=0;j<Temp.cols;j++){
                sum_st+=(Temp.at<uchar>(i,j)-mt)*(Temp.at<uchar>(i,j)-mt);
            }
        }
        st_square=(double)sum_st/n;

        float NCC[Img.rows][Img.cols]={0.0};
        double mf = 0.0;
        double sf_square = 0.0;
        double sum_mf = 0.000000;
        double sum_sf = 0.000000;
        double sum = 0.000000;
        float max_NCC=0.00;
        int X=0;//左上角点横坐标
        int Y=0;//左上角点纵坐标
        for(int i=0;i<Img.rows-Temp.rows+1;i++){
            for(int j=0;j<Img.cols-Temp.cols+1;j++){
                sum_mf = 0.000000;
                for (int k = 0; k <Temp.rows; k++){
                    for (int l = 0; l < Temp.cols; l++){
                        if((i+k) <Img.rows && (j+l)<Img.cols){
                            sum_mf+=Img.at<uchar>(i+k,j+l);
                        }
                    }
                }
                mf=sum_mf/n;

                sum_sf = 0.000000;
                for (int k = 0; k <Temp.rows; k++){
                    for (int l = 0; l < Temp.cols; l++){
                        if((i+k)<Img.rows && (j+l)<Img.cols)    
                        sum_sf+=(Img.at<uchar>(i+k,j+l)-mf)*(Img.at<uchar>(i+k,j+l)-mf);
                    }
                }
                sf_square=sum_sf/n;     

                sum = 0.000000;
                for (int k = 0; k <Temp.rows; k++){
                    for (int l = 0; l < Temp.cols; l++){
                        if((i+k)<Img.rows && (j+l)<Img.cols)
                        sum+=((Temp.at<uchar>(k,l)-mt)/sqrt(st_square))*((Img.at<uchar>(i+k,j+l)-mf)/sqrt(sf_square));
                    }
                }
                NCC[i][j]=(float)sum/n;
                if (abs(NCC[i][j])>max_NCC)//记录NCC最大值及相应的位置
                {
                    max_NCC=NCC[i][j];
                    X=i;
                    Y=j;
                }
            }
        }
        printf("最大NCC绝对值=%f\n\n",max_NCC);
        Mat result = Mat::zeros(Img.rows, Img.cols,CV_8U);
        for (int k = 0; k <Temp.rows; k++){
            for (int l = 0; l < Temp.cols; l++){
                if((X+k) <Img.rows && (Y+l)<Img.cols){
                    result.at<uchar>(X+k,Y+l)=Img.at<uchar>(X+k,Y+l);
                }
            }
        }
        imshow("result",result);
        int coordinate[]={0,0};
        coordinate[0]=X+Temp.rows/2;
        coordinate[1]=Y+Temp.cols/2;
        /*
        //坐标计算
        int start_position[]={0,0};
        int end_position[]={0,0};
        for (int i = 0;i < result.rows;i++)
        {
	        for (int j = 0;j < result.cols;j++)
	        {
		        if (result.at<uchar>(i , j )>0){
                    if (start_position[0]==0 && start_position[1]==0){
                        start_position[0]=i;
                        start_position[1]=j;
                    }
                    end_position[0]=(i>end_position[0])?i:end_position[0];
                    end_position[1]=(j>end_position[1])?j:end_position[1];
		        }
	        }
        }
        int coordinate[]={0,0};
        coordinate[0]=(start_position[0]+end_position[0])/2;
        coordinate[1]=(start_position[1]+end_position[1])/2;
        */
        printf("中心坐标=（%d,%d)\n\n", coordinate[0], coordinate[1]);
        finish=clock();
        printf("耗时%fs\n\n\n\n\n\n",(double)(finish-begin)/CLOCKS_PER_SEC);
		ros::spinOnce();
		waitKey(10000);
	}
	return 0;
}