
#include <iostream>
#include <assert.h>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

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


 
  ros::init(argc, argv, "rviz_show_octo_from_raw");
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

 cout<<"copy data into octomap..."<<endl;
 
// 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
 octomap::ColorOcTree octree( 0.05 );
 
//用普通的for循环
  for(int i=0;i<cloud->points.size();i++)
 {
// 将点云里的点插入到octomap中
    octree.updateNode( octomap::point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z), true );
  }
 
// 设置颜色
 for (int i=0;i<cloud->points.size();i++)
 {
    octree.integrateNodeColor( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
    cloud->points[i].r, cloud->points[i].g, cloud->points[i].b );
 }
 

// 更新octomap
 octree.updateInnerOccupancy();
 cout<<"data copied into octomap."<<endl;

 octomap_msgs::Octomap bmap_msg;
 octomap_msgs::binaryMapToMsg(octree, bmap_msg);

 bmap_msg.header.frame_id = "raw";

 ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("raw",1);

 octomap_publisher.publish(bmap_msg);

 ros::Rate loop_rate(1);
 while (ros::ok())
 {
   octomap_publisher.publish(bmap_msg);
   ros::spinOnce();
   loop_rate.sleep();
 }

 return 0;
}
