#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>

int main (int argc, char **argv)
{
    ros::init (argc, argv, "rviz_show_pcd");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_show", 1);
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    sensor_msgs::PointCloud2 output;
    
    //pcl::PointCloud<pcl::PointXYZ> cloud (new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("/home/cbc/桌面/ZED_cloud1.pcd", cloud) == -1) 
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
      }
     
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "pointcloud";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
