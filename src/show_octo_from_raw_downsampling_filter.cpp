
#include <iostream>
#include <assert.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char** argv){

  ros::init(argc, argv, "rviz_show_octo_from_raw_ds");
  ros::NodeHandle nh;
  PointCloud::Ptr cloud (new PointCloud);
 
  //相机内参
  const double camera_factor = 1000;
  const double camera_cx = 325.5;
  const double camera_cy = 253.5;
  const double camera_fx = 518.0;
  const double camera_fy = 519.0;

  Mat rgb,depth;
  rgb = imread("0000_color.jpg");
  depth = imread("0000_depth.png",-1);
   
  
  //遍历深度图
  for(int m=0;m<depth.rows;m++)
    for(int n=0; n<depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if(d==0)
                continue;
            PointT p;
            p.z = double(d)/camera_factor;
            p.x = (n-camera_cx)*p.z/camera_fx;
            p.y = (m-camera_cy)*p.z/camera_fy;
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back(p);
        }
  
 cloud->height = 1;
 cloud->width = cloud->points.size();
 cout<<"point cloud size = "<<cloud->points.size()<<endl;
 cloud->is_dense = false;
 //pcl::io::savePCDFile("test.pcd",*cloud);
 //cout<<"point cloud saved."<<endl;
 

 //downsampling
 pcl::PointCloud<PointT> downsampled_cloud;

 pcl::VoxelGrid<PointT> voxelSampler;
 voxelSampler.setInputClo5ud(cloud);
 voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
 voxelSampler.filter(downsampled_cloud);
 
 //filter
 pcl::PointCloud<PointT> filtered_cloud;

 pcl::StatisticalOutlierRemoval<PointT> statFilter;
 statFilter.setInputCloud(downsampled_cloud.makeShared());
 statFilter.setMeanK(10);
 statFilter.setStddevMulThresh(0.2);
 statFilter.filter(filtered_cloud);

 
// 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
 cout<<"copy data into octomap..."<<endl;
 octomap::ColorOcTree octree( 0.05 );

 //用普通的for循环
  for(int i=0;i<filtered_cloud.points.size();i++)
 {
// 将点云里的点插入到octomap中
    octree.updateNode( octomap::point3d(filtered_cloud.points[i].x, filtered_cloud.points[i].y, filtered_cloud.points[i].z), true );
  }
 
// 设置颜色
 for (int i=0;i<filtered_cloud.points.size();i++)
 {
    octree.integrateNodeColor( filtered_cloud.points[i].x, filtered_cloud.points[i].y, filtered_cloud.points[i].z,
    filtered_cloud.points[i].r, filtered_cloud.points[i].g, filtered_cloud.points[i].b );
 }

// 更新octomap
 octree.updateInnerOccupancy();
 cout<<"data copied into octomap."<<endl;
 

 octomap_msgs::Octomap bmap_msg;
 octomap_msgs::binaryMapToMsg(octree, bmap_msg);

 bmap_msg.header.frame_id = "ds";

 ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("filtered",1);

 //octomap_publisher.publish(bmap_msg);
 //ros::spinOnce();
 
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
 int v1(0);
 int v2(0);
 int v3(0);
 viewer->createViewPort(0.0,0.0,0.5,0.5,v1);
 viewer->setBackgroundColor(0,0,0,v1);
 
 viewer->createViewPort(0.5,0.0,1.0,0.5,v2);
 viewer->setBackgroundColor(0,0,0,v2);
 
 viewer->createViewPort(0.0,0.5,0.5,1.0,v3);
 viewer->setBackgroundColor(0,0,0,v3);

 viewer->addCoordinateSystem(1.0);
 viewer->initCameraParameters();

 viewer->addPointCloud<pcl::PointXYZRGBA>(cloud,"raw",v1);
 viewer->addPointCloud<pcl::PointXYZRGBA>(downsampled_cloud.makeShared(),"ds",v2);
 viewer->addPointCloud<pcl::PointXYZRGBA>(filtered_cloud.makeShared(),"filtered",v3);
 
 ros::Rate loop_rate(1);
 while (ros::ok())
 {
   if(!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    octomap_publisher.publish(bmap_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
 }
 return 0;
}
