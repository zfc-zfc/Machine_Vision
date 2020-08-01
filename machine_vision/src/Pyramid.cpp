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
void Bottom_Generation(Mat input, Mat &output);
void Zoomout(Mat temp_in, Mat img_in, Mat &temp_out, Mat &img_out);
void ScoreCalculation(Mat Temp, Mat Img, int input_i, int input_j, int &Position_i, int &Position_j);
void pyramid(Mat Temp, Mat Img);
//机器视觉课程设计：8层NCC模板匹配
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
        Mat Template1 = imread("/home/fangcheng/Library/project/pattern.bmp",0);
        Mat Image1 = imread("/home/fangcheng/Library/project/F4Y32962DL8FC5VAK_3_110418.bmp",0);
        imshow("Template1",Template1);
        imshow("Image1",Image1);
        pyramid(Template1, Image1);
        finish=clock();
        printf("耗时%fs\n\n\n\n\n\n",(double)(finish-begin)/CLOCKS_PER_SEC);
		ros::spinOnce();
		waitKey(1000);
	}
	return 0;
}
/*******************************************************/
//产生底层的图像Img和模板Temp
//输入为原始的Image&Template，输出为底层的Img和Temp
/*******************************************************/
void Bottom_Generation(Mat input, Mat &output)
{
    for (int i = 0; i < output.rows; i++) {
		for (int j = 0; j < output.cols; j++) {
			if (i < input.rows && j < input.cols) {
				output.at<uchar>(i, j) = input.at<uchar>(i, j);
			}
            else
                output.at<uchar>(i, j) = 0;
		}
	}
}
/*******************************************************/
//金字塔下采样，金字塔上层图像宽、高均变为下层的一半
/*******************************************************/
void Zoomout(Mat temp_in, Mat img_in, Mat &temp_out, Mat &img_out)
{
    pyrDown(temp_in, temp_out, Size(temp_in.cols / 2, temp_in.rows / 2));
    pyrDown(img_in, img_out, Size(img_out.cols / 2, img_out.rows / 2));
    imshow("zoomout of temp",temp_out);
    imshow("zoomout of image",img_out);
}
/*******************************************************/
//NCC分值计算，同时输出匹配结果
//参数1,2:该层对应的模板和图像
//参数3,4:由上层计算出的NCC值得到的“在该层模板最有可能的位置坐标”
//参数5,6:在该层计算NCC，得到最大NCC对应的位置坐标，为下一次迭代提供参数
/*******************************************************/
void ScoreCalculation(Mat Temp, Mat Img, int input_i, int input_j, int &Position_i, int &Position_j)
{
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

        
        float NCC=0.0;
        double mf = 0.0;
        double sf_square = 0.0;
        double sum_mf = 0.000000;
        double sum_sf = 0.000000;
        double sum = 0.000000;
        float max_NCC=0.00;
        int X=0;//左上角点横坐标
        int Y=0;//左上角点纵坐标
        int start_i = 0;
        int end_i = Img.rows-Temp.rows;
        int start_j = 0;
        int end_j = Img.cols-Temp.cols;
        //不是(0,0)时才改变
        //使搜索范围局限于一个5x5的邻域，减小运算量，提高速度
        if (input_i>0 || input_j>0)
        {
            if((input_i-2) >= 0)
                start_i=input_i-2;
            
            if((input_i+2) <= (Img.rows-Temp.rows))
                end_i=input_i+2;
            
            if((input_j-2) >= 0)
                start_j=input_j-2;
            
            if((input_j+2) <= (Img.cols-Temp.cols))
                end_j=input_j+2;
        }
        for(int i=start_i;i<end_i+1;i++){
            for(int j=start_j;j<end_j+1;j++){
                sum_mf = 0.000000;
                for (int k = 0; k <Temp.rows; k++){
                    for (int l = 0; l < Temp.cols; l++){
                        if((i+k) <Img.rows && (j+l)<Img.cols){
                            sum_mf+=Img.at<uchar>(i+k,j+l);
                        }
                    }
                }
                mf=sum_mf/n;//灰度平均值
                sum_sf = 0.000000;
                for (int k = 0; k <Temp.rows; k++){
                    for (int l = 0; l < Temp.cols; l++){
                        if((i+k)<Img.rows && (j+l)<Img.cols)    
                        sum_sf+=(Img.at<uchar>(i+k,j+l)-mf)*(Img.at<uchar>(i+k,j+l)-mf);
                    }
                }
                sf_square=sum_sf/n;//方差

                sum = 0.000000;
                for (int k = 0; k <Temp.rows; k++){
                    for (int l = 0; l < Temp.cols; l++){
                        if((i+k)<Img.rows && (j+l)<Img.cols)
                        sum+=((Temp.at<uchar>(k,l)-mt)/sqrt(st_square))*((Img.at<uchar>(i+k,j+l)-mf)/sqrt(sf_square));
                    }
                }
                NCC=(float)sum/n;
                if (abs(NCC)>max_NCC)//记录NCC最大值及相应的位置
                {
                    max_NCC=NCC;
                    X=i;
                    Y=j;
                }
            }
        }
        Position_i=X;
        Position_j=Y;
        printf("最大NCC绝对值=%f\n\n",max_NCC);
        //在Img中框处模板对应的区域
        Mat result = Img.clone();
        cvtColor(result,result,COLOR_GRAY2RGB);
        Point rect[4]; //矩形的四个顶点
        rect[0]=Point(Y,X);
        rect[1]=Point(Y+Temp.cols,X);
        rect[2]=Point(Y+Temp.cols,X+Temp.rows);
        rect[3]=Point(Y,X+Temp.rows);
        cv::line(result, rect[0], rect[1], cv::Scalar(0,0,255),3);
        cv::line(result, rect[1], rect[2], cv::Scalar(0,0,255),3);
        cv::line(result, rect[2], rect[3], cv::Scalar(0,0,255),3);
        cv::line(result, rect[3], rect[0], cv::Scalar(0,0,255),3);
        imshow("result",result);
        
        //计算中心坐标
        int coordinate[]={0,0};
        coordinate[0]=X+Temp.rows/2;
        coordinate[1]=Y+Temp.cols/2;
        printf("中心坐标=（%d,%d)\n\n", coordinate[0], coordinate[1]);
}

/*******************************************************/
//金字塔，主要计算的函数
//参数1,2:用户给定的模板图像和待检测图像
/*******************************************************/
void pyramid(Mat Temp, Mat Img)
{
        //产生底层图像，修改宽、高为2^7的倍数（8层），已知Img为2588x1940,Temp为1774x1106，故选取最邻近的整数
        Mat Img1 = Mat(2048,2688,CV_8UC1); //第一个参数为rows行数，即高度
        Bottom_Generation(Img,Img1);
        Mat Temp1 = Mat(1152,1792,CV_8UC1);
        Bottom_Generation(Temp,Temp1);
        imshow("Temp1",Temp1);
        imshow("Img1",Img1);
        
        //下采样，尺寸缩小，长宽均为上一层的0.5
        Mat Temp2,Img2,Temp3,Img3,Temp4,Img4,Temp5,Img5,Temp6,Img6,Temp7,Img7,Temp8,Img8;
        Zoomout(Temp1, Img1, Temp2, Img2);
        Zoomout(Temp2, Img2, Temp3, Img3);
        Zoomout(Temp3, Img3, Temp4, Img4);
        Zoomout(Temp4, Img4, Temp5, Img5);
        Zoomout(Temp5, Img5, Temp6, Img6);
        Zoomout(Temp6, Img6, Temp7, Img7);
        Zoomout(Temp7, Img7, Temp8, Img8);
        //在第8层搜索
        int out_position_i=0;
        int out_position_j=0;
        int in_position_i=0;
        int in_position_j=0;
        ScoreCalculation(Temp8, Img8, in_position_i, in_position_j, out_position_i, out_position_j);
        
        //在第7层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp7, Img7, in_position_i, in_position_j, out_position_i, out_position_j);
        //在第6层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp6, Img6, in_position_i, in_position_j, out_position_i, out_position_j);
        //在第5层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp5, Img5, in_position_i, in_position_j, out_position_i, out_position_j);
        //在第4层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp4, Img4, in_position_i, in_position_j, out_position_i, out_position_j);
        //在第3层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp3, Img3, in_position_i, in_position_j, out_position_i, out_position_j);
        //在第2层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp2, Img2, in_position_i, in_position_j, out_position_i, out_position_j);
        //在第1层搜索
        in_position_i=out_position_i*2;
        in_position_j=out_position_j*2;
        ScoreCalculation(Temp1, Img1, in_position_i, in_position_j, out_position_i, out_position_j);
        
        printf("匹配区域左上角在该层的坐标=（%d,%d)\n\n", out_position_i, out_position_j);
        
        
        
}