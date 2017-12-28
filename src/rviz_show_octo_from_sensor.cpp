
/*
程序实现
1.订阅zed发布的点云消息
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
PointCloud::Ptr cloud (new PointCloud);
PointCloud::Ptr cloud_transf (new PointCloud);
PointCloud::Ptr cloud_radiusf (new PointCloud);
PointCloud::Ptr cloud_voxelf
int num = 0;
void pclsubCallback(const sensor_msgs::PointCloud2& msg)
    {
        num++;
        if(num%100==0)
     {
        ros::NodeHandle nh;
        cout<<"receiving point cloud from sensor..."<<endl;
        pcl::fromROSMsg(msg, *cloud);
        cout<<"point cloud from sensor received."<<endl;
        cout<<"raw point cloud size = "<<cloud->points.size()<<endl;
        //半径滤波
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius(2);
        outrem.filter(*cloud_radiusf);
    
        cout <<"after Radius_Outlier_Removal filtering,"
             << cloud_radiusf->width * cloud_radiusf->height<< " data points."<< std::endl;
    /*//统计滤波器
        
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter_SOR;
        filter_SOR.setInputCloud(cloud);
        filter_SOR.setMeanK(50);
        filter_SOR.setStddevMulThresh(1.0);
        filter_SOR.filter(*cloud_filtered_SOR);

        cout <<"after Statistical_Outlier_Removal filtering,"
            << cloud_filtered_SOR->width * cloud_filtered_SOR->height
            << " data points."
            << std::endl;
    */      
                
        //降采样(体素滤波器)
        pcl::VoxelGrid<pcl::PointXYZ> filter_VG;
        filter_VG.setInputCloud(cloud);
        filter_VG.setLeafSize(0.01f,0.01f,0.01f);
        filter_VG.filter(*cloud_voxelf);

        cout <<"after VoxelGrid filtering,"
             << cloud_voxelf->width * cloud_voxelf->height<< " data points." << std::endl;





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

        bmap_msg.header.frame_id = "octomap-from-kinect2";

        ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("octo_show_from_kiect2",1);

        octomap_publisher.publish(bmap_msg);
      }
     
     }

 


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_show_octo_from_sensor");
  ros::NodeHandle nh;
  //ros::Rate loop_rate(10);
  //ros::Subscriber pcl_subscriber = nh.subscribe("kinect2/hd/points", 1000, pclsubCallback);
 //while(ros::ok())
 //{ 
  //ros::Subscriber pcl_subscriber = nh.subscribe("kinect2/hd/points", 1000, pclsubCallback);//kinect2发布的点云话题
  ros::Subscriber pcl_subscriber = nh.subscribe("point_cloud/cloud_registered", 1000, pclsubCallback); //zed发布的点云话题
  
  //ros::spinOnce();
 // loop_rate.sleep();

 
 ros::spin();
 return 0;
}
