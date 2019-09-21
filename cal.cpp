#include <iostream>
#include <fstream> 
#include <string.h>
#include <algorithm>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#define pi 3.1415926
using namespace cv;
using namespace std;
using namespace pcl::console;
int MaxIndex(int num[],int length)
{
	int i,max,max_sign;
	max=num[0];
	for(i=1;i<length;i++)
	{
      	  if(num[i]>max)
      	  {
        	 max=num[i];
       	    	 max_sign=i;
      	  }
    	}
    return max_sign;
}

float Histogram(vector<float>& array,int bin,int size)
{
	int *num= new int[bin];
	float result;
	for(int i=0;i<bin;i++)
		num[i]=0;
	float maxValue,minValue,distance;
	maxValue = *max_element(array.begin(), array.end());
	minValue = *min_element(array.begin(), array.end());
        distance = (maxValue - minValue)/bin; 
	//cout<<"minValue ="<<minValue<<endl<<"maxValue ="<<maxValue<<endl<<"distance ="<<distance<<endl;
	for(int i=0;i<size;i++)
	{
		for(int j=0;j<bin;j++)
			if(array[i] >= (minValue+j*distance) && array[i] < (minValue+(j+1)*distance))
				{num[j]++;break;}	
	}//统计各分段的频数
	num[bin-1]++;
 	//for(int i=0;i<bin;i++) cout<<num[i]<<" "; 
	result=minValue+(MaxIndex(num,bin)+0.5)*distance;//计算直方图最高处的中点值
	return result;			
}

float DirecAngle(float y,float x)
{
   if(y<0&&x>0) return atan(y/x)+2*pi;
   if(y>0&&x>0) return atan(y/x);
   return atan(y/x)+pi;
}

int    default_k = 0,default_iter=1,default_bin=20;
double x=0,y=0,z=0;
int main(int argc,char** argv)
{
/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("car0.pcd", *cloud)==-1)
    {
        PCL_ERROR("Could not read file\n");
    }
*/
    int k = default_k,iter=default_iter,bin=default_bin;
    parse_argument (argc, argv, "-k", k);
    parse_argument (argc, argv, "-i", iter);
	parse_argument (argc, argv, "-b", bin);
	parse_argument (argc, argv, "-x", x);
	parse_argument (argc, argv, "-y", y);
        parse_argument (argc, argv, "-z", z);
    vector<int> p_file_indices;
    p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
    if (p_file_indices.size () != 2)
    {
      print_error ("Need one input PCD file and one output PCD file to continue.\n");
      return (-1);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[0]], *cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);

    //点云法向计算时，搜索近邻点数量
    n.setKSearch(k);

    n.compute(*normals);
    int sizeCloud=cloud->points.size();
	vector<Eigen::Vector2d> XOY, YOZ, ZOX;
	vector<float> Rz, Rx, Ry;
	XOY.resize(sizeCloud);
	YOZ.resize(sizeCloud);
	ZOX.resize(sizeCloud);
	Rz.resize(sizeCloud);
	Rx.resize(sizeCloud);
	Ry.resize(sizeCloud);
	
        for(size_t i=0;i < normals->points.size (); ++i)
	{
		
			XOY[i](0)=normals->points[i].normal_x;
		 	XOY[i](1)= normals->points[i].normal_y;

			YOZ[i](0) = normals->points[i].normal_y;
		 	YOZ[i](1) = normals->points[i].normal_z;

			ZOX[i](0)= normals->points[i].normal_z;
		 	ZOX[i](1)= normals->points[i].normal_x;

	}
   for(size_t i=0;i < normals->points.size (); ++i)
   	{
		Rz[i]= DirecAngle(XOY[i](1),XOY[i](0));
		Rx[i]= DirecAngle(YOZ[i](1),YOZ[i](0));
   		Ry[i]= DirecAngle(ZOX[i](1),ZOX[i](0));
	}

   int size=normals->points.size ();
	float rz,ry,rx;

// cout<<"size:"<<size<<endl;
   	rz=Histogram(Rz,bin,size);
	rx=Histogram(Rx,bin,size);
	ry=Histogram(Ry,bin,size);
//	cout<<rz<<" "<<ry<<" "<<rx<<endl<<endl;

//**********RANSAC提取地面计算正视滚转角

//×××××
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	//分割方法：随机采样法
	seg.setMethodType(pcl::SAC_RANSAC);
	//设置阈值
	seg.setDistanceThreshold(3.5);
	//输入点云
	seg.setInputCloud(cloud);
	//分割点云，获得平面和法向量
	seg.segment(*inliers, *coefficients);

//×××××

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);

	pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);

	pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg1;

	seg1.setOptimizeCoefficients(true);

	seg1.setModelType(pcl::SACMODEL_PLANE);

	seg1.setMethodType(pcl::SAC_RANSAC);

	seg1.setDistanceThreshold(3.5);

	seg1.setInputCloud(cloud1);

	seg1.segment(*inliers1, *coefficients1);
//********计算法向量的旋转

	float new_ry,new_ry1,new_rx,new_rx1;

	new_ry=DirecAngle(coefficients->values[2],coefficients->values[0]);	
	new_rx=DirecAngle(coefficients->values[2],coefficients->values[1]);	

//	new_ry1=DirecAngle(coefficients1->values[2],coefficients1->values[0]);	
//	new_rx1=DirecAngle(coefficients1->values[2],coefficients1->values[1]);
	cout<<rz<<" "<<new_ry<<" "<<new_rx<<endl<<endl;

//	cout<<rz<<" "<<new_ry1<<" "<<new_rx1<<endl;
//对第二张图片提取法向量

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n1;
	pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);


	Eigen::Matrix4d RZ= Eigen::Matrix4d::Identity();
	Eigen::Matrix4d RY= Eigen::Matrix4d::Identity();
	Eigen::Matrix4d RX= Eigen::Matrix4d::Identity();
	Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
 


	int sizeCloud1=cloud1->points.size();

	vector<Eigen::Vector2f> XOY1, YOZ1, ZOX1;
	vector<float> Rz1, Rx1, Ry1;
	XOY1.resize(sizeCloud);
	YOZ1.resize(sizeCloud);
	ZOX1.resize(sizeCloud);
	Rz1.resize(sizeCloud);
	Rx1.resize(sizeCloud);
	Ry1.resize(sizeCloud);

	float rz1,ry1,rx1;
	float yaw,pitch,roll;

	int I=0;

while(I<iter)
{
	if(I==0)
	{		
	//正视旋转(只做一次)
		new_ry1=DirecAngle(coefficients1->values[2],coefficients1->values[0]);	

		if(new_ry>new_ry1) pitch=new_ry-new_ry1;
			else pitch=2*pi-(new_ry1-new_ry);

		if(abs(new_ry1-new_ry)<0.05) pitch=0;

//		pitch=new_ry-new_ry1;

		RY(0,0)=cos(pitch);
		RY(0,2)=sin(pitch);
		RY(2,0)=-sin(pitch);
		RY(2,2)=cos(pitch);

		R = RY*R;

		pcl::transformPointCloud(*cloud1, *cloud1, RY);
	//侧视旋转只做一次
		seg1.setInputCloud(cloud1);

		seg1.segment(*inliers1, *coefficients1);

		new_rx1=DirecAngle(coefficients1->values[2],coefficients1->values[1]);	

		if(new_rx>new_rx1) roll=new_rx-new_rx1;
		else roll=2*pi-(new_rx1-new_rx);

	
		if(abs(new_rx1-new_rx)<0.05) roll=0;

		roll=new_rx-new_rx1;
		
		RX(1,1)=cos(roll);
		RX(1,2)=-sin(roll);
		RX(2,1)=sin(roll);
		RX(2,2)=cos(roll);

		R = RX*R;
		
		pcl::transformPointCloud(*cloud1, *cloud1, RX);

	}

	
//俯视旋转
    tree1->setInputCloud(cloud1);
    n1.setInputCloud(cloud1);
    n1.setSearchMethod(tree1);
    n1.setKSearch(k);
    n1.compute(*normals1);

    for(size_t i=0;i < normals1->points.size (); ++i)
	{
			
				XOY1[i](0)=normals1->points[i].normal_x;
				XOY1[i](1)= normals1->points[i].normal_y;

	}
    for(size_t i=0;i < normals1->points.size (); ++i)
   	{
			Rz1[i]= DirecAngle(XOY1[i](1),XOY1[i](0));
	}
	rz1=Histogram(Rz1,bin,sizeCloud1);

	if(rz>rz1) yaw=rz-rz1;
		else yaw=2*pi-(rz1-rz);

//	yaw=rz-rz1;
	if(abs(rz1-rz)<0.05) yaw=0;

	RZ(0,0)=cos(yaw);
	RZ(0,1)=-sin(yaw);
	RZ(1,0)=sin(yaw);
	RZ(1,1)=cos(yaw);

	R = RZ*R;
 	
	pcl::transformPointCloud(*cloud1, *cloud1, RZ);
	

	//cout<<"No."<<I+1<<":"<<endl<<rz1<<" "<<rx1<<" "<<ry1<<endl<<endl;

	cout<<rz1<<" "<<new_ry1<<" "<<new_rx1<<endl;

	I++;
}




//*********projection部分
	Mat dst1, dst2,dst11,dst22,dst111,dst222;
	int shiftX=0,shiftY=0;

//**********第一张图fushi
	vector<int> vecDataX(size,0),vecDataY(size,0);

        for (int i = 0; i < size; i++)
        {
//              vecDataX.push_back(round(cloud->points[i].x));
//		vecDataY.push_back(round(cloud->points[i].y));
		vecDataX[i]=round(cloud->points[i].x);
		vecDataY[i]=round(cloud->points[i].y);
        }
        double maxElementX = *(std::max_element(vecDataX.begin(), vecDataX.end()));
        double minElementX = *(std::min_element(vecDataX.begin(), vecDataX.end()));
        double maxElementY = *(std::max_element(vecDataY.begin(), vecDataY.end()));
        double minElementY = *(std::min_element(vecDataY.begin(), vecDataY.end()));


	if(-minElementX>shiftX)
                shiftX=-minElementX;
	if(-minElementY>shiftY)
                shiftY=-minElementY;
//	imshow("image", image);


//**********第二张图fushi
	int size1=cloud1->points.size();
	vector<int> vecDataX1(size1,0),vecDataY1(size1,0);
	

        for (int i = 0; i < size1; i++)
        {
//                vecDataX1.push_back(round(cloud1->points[i].x));
//		vecDataY1.push_back(round(cloud1->points[i].y));
		vecDataX1[i]=round(cloud1->points[i].x);
		vecDataY1[i]=round(cloud1->points[i].y);
        }
        double maxElementX1 = *(std::max_element(vecDataX1.begin(), vecDataX1.end()));
        double minElementX1 = *(std::min_element(vecDataX1.begin(), vecDataX1.end()));
        double maxElementY1 = *(std::max_element(vecDataY1.begin(), vecDataY1.end()));
        double minElementY1 = *(std::min_element(vecDataY1.begin(), vecDataY1.end()));

	if(-minElementX1>shiftX)
                shiftX=-minElementX1;
	if(-minElementY1>shiftY)
                shiftY=-minElementY1;


	int Xmax=max(maxElementX,maxElementX1);
	int Ymax=max(maxElementY,maxElementY1);
	int Xmin=min(minElementX,minElementX1);
	int Ymin=min(minElementY,minElementY1);
//********开始绘制第一张图的俯视投影
	Mat image1(Ymax-Ymin,Xmax-Xmin,CV_8UC1,Scalar(0));
        for (int i = 0; i < size; i++)
	{
		vecDataX[i]+=shiftX;
		vecDataY[i]+=shiftY;
	}
        for (int i = 0; i < size; i++)
	{
		image1.at<uchar>(abs(Ymax-Ymin-vecDataY[i]),vecDataX[i]) = 255;	
	}
	cout<<"rows:"<<image1.rows<<endl;
	cout<<"cols:"<<image1.cols<<endl;

   	vector<int> compression_params;
   	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION); //PNG格式图片的压缩级别  
        compression_params.push_back(9);  //这里设置保存的图像质量级别
 
        imwrite("new1_fushi.png", image1,compression_params);

//	imshow("image", image);
//********开始绘制第二张图的俯视投影
//	Mat image1(maxElementY1-minElementY1,maxElementX1-minElementX1,CV_8UC1,Scalar(0));

	Mat image2(Ymax-Ymin,Xmax-Xmin,CV_8UC1,Scalar(0));
        for (int i = 0; i < size1;i++)
	{
		vecDataX1[i]+=shiftX;
		vecDataY1[i]+=shiftY;
	}
        for (int i = 0; i < size1;i++)
	{
		image2.at<uchar>(abs(Ymax-Ymin-vecDataY1[i]),vecDataX1[i]) = 255;			
	}
	cout<<"rows:"<<image2.rows<<endl;
	cout<<"cols:"<<image2.cols<<endl;

 
        imwrite("new2_fushi.png", image2,compression_params);
//	imshow("image1", image1);


//	cvtColor( image,image1, CV_BGR2GRAY );     //转换为灰度图像
        image1.convertTo(dst1,CV_32FC1);       //转换为32位浮点型
//	cvtColor( src2, src2, CV_BGR2GRAY );
        image2.convertTo(dst2,CV_32FC1);
 	

	Point2d phase_shiftXY;
        phase_shiftXY = phaseCorrelate(dst1,dst2);//fushi
       cout<<endl<<"warp :"<<endl<<"\tX shift : "<<phase_shiftXY.x<<"\tY shift : "<<-phase_shiftXY.y<<endl;


//**********第一张图zhengshi
	vector<int> vecDataX3(size,0),vecDataY3(size,0);
	shiftX=0;
	shiftY=0;
        for (int i = 0; i < size; i++)
        {
//                vecDataX3.push_back(round(cloud->points[i].x));
//		vecDataY3.push_back(round(cloud->points[i].z));
		vecDataX3[i]=round(cloud->points[i].x);
		vecDataY3[i]=round(cloud->points[i].z);
        }
        maxElementX = *(std::max_element(vecDataX3.begin(), vecDataX3.end()));
        minElementX = *(std::min_element(vecDataX3.begin(), vecDataX3.end()));
        maxElementY = *(std::max_element(vecDataY3.begin(), vecDataY3.end()));
        minElementY = *(std::min_element(vecDataY3.begin(), vecDataY3.end()));


	if(-minElementX>shiftX)
                shiftX=-minElementX;
	if(-minElementY>shiftY)
                shiftY=-minElementY;
//	imshow("image", image);


//**********第二张图zhengshi
	vector<int> vecDataX4(size1,0),vecDataY4(size1,0);
        for (int i = 0; i < size1; i++)
        {
//                vecDataX4.push_back(round(cloud1->points[i].x));
//		vecDataY4.push_back(round(cloud1->points[i].z));
		vecDataX4[i]=round(cloud1->points[i].x);
		vecDataY4[i]=round(cloud1->points[i].z);
        }
        maxElementX1 = *(std::max_element(vecDataX4.begin(), vecDataX4.end()));
        minElementX1 = *(std::min_element(vecDataX4.begin(), vecDataX4.end()));
        maxElementY1 = *(std::max_element(vecDataY4.begin(), vecDataY4.end()));
        minElementY1 = *(std::min_element(vecDataY4.begin(), vecDataY4.end()));

	if(-minElementX1>shiftX)
                shiftX=-minElementX1;
	if(-minElementY1>shiftY)
                shiftY=-minElementY1;


	Xmax=max(maxElementX,maxElementX1);
	Ymax=max(maxElementY,maxElementY1);
	Xmin=min(minElementX,minElementX1);
	Ymin=min(minElementY,minElementY1);
//********开始绘制第一张图的正视投影
	Mat image3(Ymax-Ymin,Xmax-Xmin,CV_8UC1,Scalar(0));
        for (int i = 0; i < size; i++)
	{
		vecDataX3[i]+=shiftX;

		vecDataY3[i]+=shiftY;
	}
		
        for (int i = 0; i < size; i++)
	{
		image3.at<uchar>(abs(Ymax-Ymin-vecDataY3[i]),vecDataX3[i]) = 255;	
	}
	cout<<"rows:"<<image3.rows<<endl;
	cout<<"cols:"<<image3.cols<<endl;
 
        imwrite("new1_zhenghsi.png", image3,compression_params);
//	imshow("image", image);
//********开始绘制第二张图的正视投影
//	Mat image1(maxElementY1-minElementY1,maxElementX1-minElementX1,CV_8UC1,Scalar(0));

	Mat image4(Ymax-Ymin,Xmax-Xmin,CV_8UC1,Scalar(0));
        for (int i = 0; i < size1; i++)
	{
		vecDataX4[i]+=shiftX;

		vecDataY4[i]+=shiftY;
	}
        for (int i = 0; i < size1; i++)
	{
		image4.at<uchar>(abs(Ymax-Ymin-vecDataY4[i]),vecDataX4[i]) = 255;			
	}
	cout<<"rows:"<<image4.rows<<endl;
	cout<<"cols:"<<image4.cols<<endl;

 
        imwrite("new2_zhengshi.png", image4,compression_params);
//	imshow("image4", image4);


//	cvtColor( image,image1, CV_BGR2GRAY );     //转换为灰度图像
        image3.convertTo(dst11,CV_32FC1);       //转换为32位浮点型
//	cvtColor( src2, src2, CV_BGR2GRAY );
        image4.convertTo(dst22,CV_32FC1);
 	
//	cout<<dst11.size()<<"  "<<dst22.size()<<endl;
	Point2d phase_shiftXZ;
        phase_shiftXZ = phaseCorrelate(dst11,dst22);//fushi
        cout<<endl<<"warp :"<<endl<<"\tX shift : "<<phase_shiftXZ.x<<"\tZ shift : "<<-phase_shiftXZ.y<<endl;


//**********第一张图ceshi
	vector<int> vecDataX5(size,0),vecDataY5(size,0);
	shiftX=0;
	shiftY=0;
        for (int i = 0; i < size; i++)
        {
//                vecDataX5.push_back(round(cloud->points[i].y));
//		vecDataY5.push_back(round(cloud->points[i].z));
		vecDataX5[i]=round(cloud->points[i].y);
		vecDataY5[i]=round(cloud->points[i].z);
        }
        maxElementX = *(std::max_element(vecDataX5.begin(), vecDataX5.end()));
        minElementX = *(std::min_element(vecDataX5.begin(), vecDataX5.end()));
        maxElementY = *(std::max_element(vecDataY5.begin(), vecDataY5.end()));
        minElementY = *(std::min_element(vecDataY5.begin(), vecDataY5.end()));


	if(-minElementX>shiftX)
                shiftX=-minElementX;
	if(-minElementY>shiftY)
                shiftY=-minElementY;
//	imshow("image", image);


//**********第二张图ceshi
	vector<int> vecDataX6(size1,0),vecDataY6(size1,0);
        for (int i = 0; i < size1; i++)
        {
 //               vecDataX6.push_back(round(cloud1->points[i].y));
//		vecDataY6.push_back(round(cloud1->points[i].z));
		vecDataX6[i]=round(cloud1->points[i].y);
		vecDataY6[i]=round(cloud1->points[i].z);
        }
        maxElementX1 = *(std::max_element(vecDataX6.begin(), vecDataX6.end()));
        minElementX1 = *(std::min_element(vecDataX6.begin(), vecDataX6.end()));
        maxElementY1 = *(std::max_element(vecDataY6.begin(), vecDataY6.end()));
        minElementY1 = *(std::min_element(vecDataY6.begin(), vecDataY6.end()));

	if(-minElementX1>shiftX)
                shiftX=-minElementX1;
	if(-minElementY1>shiftY)
                shiftY=-minElementY1;


	Xmax=max(maxElementX,maxElementX1);
	Ymax=max(maxElementY,maxElementY1);
	Xmin=min(minElementX,minElementX1);
	Ymin=min(minElementY,minElementY1);
//********开始绘制第一张图的侧视投影
	Mat image5(Ymax-Ymin,Xmax-Xmin,CV_8UC1,Scalar(0));
        for (int i = 0; i < size; i++)
	{
		vecDataX5[i]+=shiftX;
		vecDataY5[i]+=shiftY;
	}
        for (int i = 0; i < size; i++)
	{
		image5.at<uchar>(abs(Ymax-Ymin-vecDataY5[i]),vecDataX5[i]) = 255;	
	}
	cout<<"rows:"<<image5.rows<<endl;
	cout<<"cols:"<<image5.cols<<endl;
 
        imwrite("new1_cehsi.png", image5,compression_params);
//	imshow("image", image);
//********开始绘制第二张图的侧视投影
//	Mat image1(maxElementY1-minElementY1,maxElementX1-minElementX1,CV_8UC1,Scalar(0));

	Mat image6(Ymax-Ymin,Xmax-Xmin,CV_8UC1,Scalar(0));
        for (int i = 0; i < size1; i++)
	{
		vecDataX6[i]+=shiftX;
		vecDataY6[i]+=shiftY;
	}
        for (int i = 0; i < size1; i++)
	{
		image6.at<uchar>(abs(Ymax-Ymin-vecDataY6[i]),vecDataX6[i]) = 255;			
	}
	cout<<"rows:"<<image6.rows<<endl;
	cout<<"cols:"<<image6.cols<<endl;

 
        imwrite("new2_ceshi.png", image6,compression_params);
//	imshow("image1", image1);


//	cvtColor( image,image1, CV_BGR2GRAY );     //转换为灰度图像
        image5.convertTo(dst111,CV_32FC1);       //转换为32位浮点型
//	cvtColor( src2, src2, CV_BGR2GRAY );
        image6.convertTo(dst222,CV_32FC1);
 	

	Point2d phase_shiftYZ;
       phase_shiftYZ = phaseCorrelate(dst111,dst222);//fushi
        cout<<endl<<"warp :"<<endl<<"\tY shift : "<<phase_shiftYZ.x<<"\tZ shift : "<<-phase_shiftYZ.y<<endl;


	x=(phase_shiftXY.x+phase_shiftXZ.x)/2.0;
	y=(-phase_shiftXY.y+phase_shiftYZ.x)/2.0;
	z=(-phase_shiftXZ.y-phase_shiftYZ.y)/2.0;
	cout<<endl<<x<<endl<<y<<endl<<z<<endl;

	R(0,3)=-x;
	R(1,3)=-y;
	R(2,3)=-z;

	cout<<"Final R ="<<endl<<R<<endl;

	//cout<<"Final R，T ="<<endl<<R1<<endl;






//图形显示模块
    pcl::visualization::PCLVisualizer viewer;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> incloudHandler(cloud2, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler(cloud, 225, 30, 30);

	int v1 = 1;
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPortCamera(v1);
    viewer.addPointCloud(cloud2, incloudHandler, "In",v1);
    viewer.addPointCloud(cloud, outcloudHandler, "Out",v1);
    viewer.addCoordinateSystem(0.1, "cloud", v1);
 
    int v2 = 1;
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.createViewPortCamera(v2);
    viewer.setCameraFieldOfView(0.785398,v2);
    viewer.setBackgroundColor(0.0, 0.2, 1.0,v2);
    viewer.setCameraPosition(
        0, 0, 0,
        0, 0, -1,
        0, 0, 0,v2);

	pcl::transformPointCloud(*cloud2, *cloudOut, R);

 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> incloudHandler1(cloudOut, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler1(cloud, 225, 30, 30);
   // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud1cloudHandler1(cloud1, 0, 255, 0);
    //viewer.addPointCloud(cloud, incloudHandler1, "In22",v2);
    viewer.addPointCloud(cloudOut, incloudHandler1, "Out22",v2);
    viewer.addPointCloud(cloud, outcloudHandler1, "Final22",v2);
    viewer.addCoordinateSystem(0.1, "cloud22", v2);
 

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
    	tree2->setInputCloud(cloud);
	pcl::PointCloud<pcl::PointXYZ> cloudOut1;
	cloudOut1=*cloudOut;
	
	double fitness_score = 0.0;

  	
        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        int nr = 0;
        for (size_t i = 0; i < cloudOut1.points.size (); ++i)
     	{
    		tree2->nearestKSearch (cloudOut1.points[i], 1, nn_indices, nn_dists);
		fitness_score += nn_dists[0];
      		nr++;
  	}
	fitness_score /= nr;
	cout<<"fitness_score is "<<fitness_score <<endl;


//*********输出合并的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
  cloud3->width    = cloud->points.size ()+cloudOut->points.size ();
  cloud3->height   = 1;
  cloud3->is_dense = false;
  cloud3->points.resize (cloud3->width * cloud3->height);
//	cout<<endl<<cloud3->width<<endl;

	size_t j;
  for (j = 0; j < cloud->points.size (); ++j)
  {
    cloud3->points[j].x = cloud->points[j].x;
    cloud3->points[j].y = cloud->points[j].y;
    cloud3->points[j].z = cloud->points[j].z;
  }
  for (j = cloud->points.size (); j < cloud->points.size ()+cloudOut->points.size (); ++j)
  {
    cloud3->points[j].x = cloudOut->points[j-cloud->points.size ()].x;
    cloud3->points[j].y = cloudOut->points[j-cloud->points.size ()].y;
    cloud3->points[j].z = cloudOut->points[j-cloud->points.size ()].z;
  }
  pcl::io::savePCDFileASCII ("merged_pcd.pcd", *cloud3);


    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
 
/*
  tree1->setInputCloud(cloud1);
    n1.setInputCloud(cloud1);
    n1.setSearchMethod(tree1);
    n1.setKSearch(k);
    n1.compute(*normals1);

	for(size_t i=0;i < normals1->points.size (); ++i)
	{
		 XOZ1[i][0]= normals1->points[i].normal_x;
		 XOZ1[i][1]= normals1->points[i].normal_z;
	}
   	for(size_t i=0;i < normals1->points.size (); ++i)
   	{
   		Ry1[i]= DirecAngle(XOZ1[i][1],XOZ1[i][0]);
	}
	ry1=Histogram(Ry1,bin,sizeCloud1);

	if(ry>ry1) pitch=ry1-ry;
		else pitch=2*pi-(ry1-ry);


//	pitch=ry1-ry;
	if(abs(ry1-ry)<0.05) pitch=0;
	
//	pitch =0.03*pi;

	RY(0,0)=cos(pitch);
	RY(0,2)=sin(pitch);
	RY(2,0)=-sin(pitch);
	RY(2,2)=cos(pitch);

	R *= RY;
 	
	pcl::transformPointCloud(*cloud1, *cloud1, RY);
*/

/*	
	for (size_t i = 0; i < cloudOut->points.size (); ++i)
   	{        
		cloudOut1.points[i]=cloudout->points[i];
	}
*/
/*
	if(rz1>rz) yaw=rz1-rz;
 		else yaw=2*pi-(rz-rz1);
	if(ry1>ry) pitch=ry1-ry;
		else pitch=2*pi-(ry-ry1);
	if(rx1>rx) roll=rx1-rx;
		else roll=2*pi-(rx-rx1);


	if(rz>rz1) yaw=rz-rz1;// 目标点云角度大于源点云，则旋转角为正，可以直接作差
 		else yaw=2*pi-(rz1-rz);//否则计算角度差的周角补角
	if(ry>ry1) pitch=ry-ry1;
		else pitch=2*pi-(ry1-ry);
	if(rx>rx1) roll=rx-rx1;
		else roll=2*pi-(rx1-rx);

	if(abs(rz1-rz)<0.1) yaw=0;//旋转角度小于pi/30=6度时，视作无旋转
	if(abs(rx1-rx)<0.1) pitch=0;
	if(abs(ry1-ry)<0.1) roll=0;
	
	RZ(0,0)=cos(yaw);
	RZ(0,1)=-sin(yaw);
	RZ(1,0)=sin(yaw);
	RZ(1,1)=cos(yaw);
	RY(0,0)=cos(pitch);
	RY(0,2)=sin(pitch);
	RY(2,0)=-sin(pitch);
	RY(2,2)=cos(pitch);
	RX(1,1)=cos(roll);
	RX(1,2)=-sin(roll);
	RX(2,1)=sin(roll);
	RX(2,2)=cos(roll);

	
	rotation_matrix = RZ*RY*RX;
	cout<<"rotation_matrix = "<<endl<<rotation_matrix<<endl<<endl;
	rotation_matrix1.block<3,3>(0,0) =rotation_matrix.block<3,3>(0,0);
   	pcl::transformPointCloud(*cloud1, *cloud1, rotation_matrix1);
	cout<<"rotation_matrix1 = "<<endl<<rotation_matrix1<<endl<<endl;
	R=R*rotation_matrix;
	R1=R1*rotation_matrix1;

*/
/*
	if (I==0) 
	{
		pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);
		
		Eigen::MatrixXd Q(sizeCloud1,3);
		for(size_t i=0;i < sizeCloud1; ++i)
	   	{
			Q(i,0)=cloud1->points[i].x;
			Q(i,1)=cloud1->points[i].y;
			Q(i,2)=cloud1->points[i].z;
		}	
	}
	else
	{
		for (size_t i = 0; i < sizeCloud1; ++i)
		{
			cloud1->points[i].x = Q(i,0);
			cloud1->points[i].y = Q(i,1);
			cloud1->points[i].z = Q(i,2);
		}
	}

*/	


   // pcl::io::loadPCDFile<pcl::PointXYZ>(argv[p_file_indices[1]], *cloud1);



  //  system("pause");
	//输出测试txt文件
	
//cout<<"Final rotation_matrix = "<<endl<<R<<endl<<endl;
      //  cout<<rotation_matrix<<endl;
/*
    ofstream Angleout("DirecAngleCar22_RT.txt");
    for(size_t i=0;i < normals->points.size (); ++i)
   	{
		Angleout<<Rz[i]<<" "<<Rx[i]<<" "<<Ry[i]<<endl;
	}
    Angleout.close();

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);  
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);  
    //pcl::io::savePCDFileASCII("car00.pcd", *cloud);
    pcl::io::savePCDFileASCII(argv[p_file_indices[1]], *cloud_with_normals);

*/
    return 0;
}

