
/*
程序实现
1.读入zed获取的点云
2.相机坐标系到机械臂底座坐标系的转换，点云视角的变换
3.半径滤波
4.降采样
5.转换成octomap，发布此消息
*/
#include <iostream>
#include <assert.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
 #include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

PointCloud::Ptr cloud (new PointCloud);
PointCloud::Ptr cloud_transformed (new PointCloud);
PointCloud::Ptr cloud_voxelfilter (new PointCloud);
PointCloud::Ptr cloud_conditionfilter (new PointCloud);
PointCloud::Ptr cloud_static (new PointCloud);
PointCloud::Ptr cloud_radiusfilter (new PointCloud);

//octomap::ColorOcTree cloudpub(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_cloud_zed");
  ros::NodeHandle nh;
 // PointCloud::Ptr cloud;
  
  if (pcl::io::loadPCDFile<PointT> ("ZED_cloud4.pcd", *cloud) == -1) 
    {
        PCL_ERROR ("Couldn't read file this pcd \n");
        return (-1);
    }
    
    cout<<"raw point cloud size:  "<<cloud->points.size()<<"data points."<<endl;
    //1.坐标系变换
    float thetax = 88.6/180*M_PI;
    float thetay = M_PI;
    float thetaz = -75/180*M_PI;
    /*
    Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();
    transform1(0,0)=;
    transform1(0,1)=;
    transform1(1,0)=;
    transform1(1,1)=;
    transform1(0,3)=;
    transform1(1,3)=;
    transform1(2,3)=;
*/
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    transform2.translation()<<1.740,0.39,0.58;
    transform2.rotate(Eigen::AngleAxisf (thetax,Eigen::Vector3f::UnitX()));
    transform2.rotate(Eigen::AngleAxisf (thetay,Eigen::Vector3f::UnitY()));
    transform2.rotate(Eigen::AngleAxisf (thetaz,Eigen::Vector3f::UnitZ()));
    cout<<transform2.matrix()<<endl;
    

    //2.降采样(体素滤波器)
    pcl::VoxelGrid<PointT> filter_VG;
    filter_VG.setInputCloud(cloud);
    filter_VG.setLeafSize(0.01f,0.01f,0.01f);
    filter_VG.filter(*cloud_voxelfilter);

    cout <<"after VoxelGrid filtering,"
         << cloud_voxelfilter->size()<< " data points." << std::endl;
    //pcl::io::savePCDFileASCII ("voxelfilterd.pcd", *cloud_voxelfilter);

    //3.条件滤波
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT>());

    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, -0.5)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, 0.33)));

    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, -0.22)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, 0.75)));

    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, 2.7 )));

    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud_voxelfilter);
    condrem.setKeepOrganized(true);
    condrem.filter (*cloud_conditionfilter);

    vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_conditionfilter,*cloud_conditionfilter,indices);
    cout << "after conditional filtering: " 
         <<cloud_conditionfilter->size()<<"data points."<<endl;
    
    
    pcl::io::savePCDFileASCII ("conditionFiltered.pcd", *cloud_conditionfilter);  
                  
/*
    //3.半径滤波
    pcl::RadiusOutlierRemoval<PointT> outrem;
    outrem.setInputCloud(cloud_conditionfilter);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(200);
    outrem.filter(*cloud_radiusfilter);

    cout <<"after Radius_Outlier_Removal filtering,"
         << cloud_radiusfilter->size()<< " data points."<< std::endl;
    pcl::io::savePCDFileASCII ("radiusfilter.pcd", *cloud_radiusfilter); 
*/

//统计滤波器        
    pcl::StatisticalOutlierRemoval<PointT> filter_SOR;
    filter_SOR.setInputCloud(cloud_conditionfilter);
    filter_SOR.setMeanK(50);
    filter_SOR.setStddevMulThresh(1.0);
    filter_SOR.filter(*cloud_static);

    cout <<"after Statistical_Outlier_Removal filtering,"
        << cloud_static->size()<< " data points."  << std::endl;
    pcl::io::savePCDFileASCII("staticfilter.pcd",*cloud_static);  

    pcl::transformPointCloud(*cloud_static,*cloud_transformed,transform2);

    //4.转换成八叉树
    cout<<"copy data into octomap..."<<endl;
    // 带颜色，参数为分辨率，这里设成了0.05
    octomap::ColorOcTree octree( 0.05 );

    //用普通的for循环
    for(int i=0;i<cloud_static->points.size();i++)
    {
    // 坐标
      octree.updateNode( octomap::point3d(cloud_static->points[i].x, cloud_static->points[i].y, cloud_static->points[i].z), true );
       }
    // 颜色
    for (int i=0;i<cloud_static->points.size();i++)
    {
        octree.integrateNodeColor( cloud_static->points[i].x, cloud_static->points[i].y, cloud_static->points[i].z,
        cloud_static->points[i].r, cloud_static->points[i].g, cloud_static->points[i].b );
    }
     // 更新octomap
    octree.updateInnerOccupancy();
    octree.write("octree_zed.ot");

    cout<<"data copied into octomap."<<endl;

    octomap_msgs::Octomap bmap_msg;
    octomap_msgs::binaryMapToMsg(octree, bmap_msg);
    bmap_msg.header.frame_id = "zedoctree";    
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("zed_test",1);
        
    sensor_msgs::PointCloud2 cloud_trans_msg;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_static,cloud_msg);
    pcl::toROSMsg(*cloud_transformed,cloud_trans_msg);
    cloud_msg.header.frame_id="zedcloud";
    cloud_trans_msg.header.frame_id="zedcloud";
    ros::Publisher cloud_pub=nh.advertise<sensor_msgs::PointCloud2> ("zed_cloud",1);
    ros::Publisher cloud_pub1=nh.advertise<sensor_msgs::PointCloud2> ("zed_trans",1);

    ros::Rate loop_rate(1);
    while(nh.ok())
    {
        cloud_pub.publish(cloud_msg);
        cloud_pub1.publish(cloud_trans_msg);
        octomap_pub.publish(bmap_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

