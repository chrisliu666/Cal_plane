#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
 

int main()
{
//*************第一幅图
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());//点云对象
	if (pcl::io::loadPCDFile("../robot1d_y4p32r32.pcd", *cloud))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return 0;
	}
	//创建一个模型参数对象，用于记录结果
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

	std::cout << "x：" << coefficients->values[0] << endl;
	std::cout << "y：" << coefficients->values[1] << endl;
	std::cout << "z：" << coefficients->values[2] << endl;
	//平面点获取
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < inliers->indices.size(); ++i)
	{
		clicked_points_3d->points.push_back(cloud->points.at(inliers->indices[i]));
	}

//*****************第二幅图

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());//点云对象
	if (pcl::io::loadPCDFile("../robot1d.pcd", *cloud1))
	{
		std::cerr << "ERROR: Cannot open file " << std::endl;
		return 0;
	}
	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
	//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZ> seg1;

	seg1.setOptimizeCoefficients(true);

	seg1.setModelType(pcl::SACMODEL_PLANE);
	//分割方法：随机采样法
	seg1.setMethodType(pcl::SAC_RANSAC);
	//设置阈值
	seg1.setDistanceThreshold(3.5);
	//输入点云
	seg1.setInputCloud(cloud1);
	//分割点云，获得平面和法向量
	seg1.segment(*inliers1, *coefficients1);

	std::cout << "x：" << coefficients1->values[0] << endl;
	std::cout << "y：" << coefficients1->values[1] << endl;
	std::cout << "z：" << coefficients1->values[2] << endl;
	//平面点获取
	pcl::PointCloud<pcl::PointXYZ>::Ptr clicked_points_3d1(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < inliers1->indices.size(); ++i)
	{
		clicked_points_3d1->points.push_back(cloud1->points.at(inliers1->indices[i]));
	}

//*******平面icp配准

    //可视化初始化
    pcl::visualization::PCLVisualizer viewer;
    viewer.setCameraFieldOfView(0.785398);//fov 45°  视场角
    viewer.setBackgroundColor(0.2, 0.2, 0.2);
    viewer.setCameraPosition(
        0, 0, 0,
        0, 0, -1,
        0, 0, 0);
 
    //点云可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> incloudHandler(cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler(cloud1, 225, 30, 30);

	int v1 = 1;
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPortCamera(v1);
    viewer.addPointCloud(clicked_points_3d, incloudHandler, "In",v1);
    viewer.addPointCloud(clicked_points_3d1, outcloudHandler, "Out",v1);
    viewer.addCoordinateSystem(0.1, "cloud", v1);
 
    int v2 = 1;
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.createViewPortCamera(v2);
    viewer.setCameraFieldOfView(0.785398,v2);//fov 45°  视场角
    viewer.setBackgroundColor(0.0, 0.2, 1.0,v2);
    viewer.setCameraPosition(
        0, 0, 0,
        0, 0, -1,
        0, 0, 0,v2);

 
    //ICP配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(clicked_points_3d);
    icp.setInputTarget(clicked_points_3d1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*Final);
 
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
    //输出最终的变换矩阵（4x4）
    std::cout << icp.getFinalTransformation() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*clicked_points_3d, *cloud2, icp.getFinalTransformation());


    //点云可视化
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> incloudHandler1(cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> outcloudHandler1(cloud1, 225, 30, 30);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> FinalcloudHandler1(cloud2, 0, 255, 0);
    //viewer.addPointCloud(cloud, incloudHandler1, "In22",v2);
    viewer.addPointCloud(clicked_points_3d1, outcloudHandler1, "Out22",v2);
    viewer.addPointCloud(cloud2, FinalcloudHandler1, "Final22",v2);
    viewer.addCoordinateSystem(0.1, "cloud22", v2);
 
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
 
    system("pause");
    return 0;
/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

	viewer->addPointCloud(cloud, "data");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(clicked_points_3d, 255, 0, 0);
	std::string cloudName="plane";
	viewer->addPointCloud(clicked_points_3d, red, cloudName);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudName);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
*/

}

