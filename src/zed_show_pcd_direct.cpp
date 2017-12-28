#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
int num=0;
void zedSubCallback(const sensor_msgs::PointCloud2& msg)
{
    ros::NodeHandle nh;
    cout<<"receiving pointcloud from zed..."<<endl;
    
}