
#include <iostream>
#include <assert.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
//#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "rviz_show_octo_from_pcd");
 
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZRGBA> cloud;

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], cloud) == -1) 
      {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
      }

  cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;
  cout<<"copy data into octomap..."<<endl;
 
  // 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
  octomap::ColorOcTree octree( 0.001 );
 

//用普通的for循环
  for(int i=0;i<cloud.points.size();i++)
{
    // 将点云里的点插入到octomap中
    octree.updateNode( octomap::point3d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z), true );
 }
 
 // 设置颜色
 for (int i=0;i<cloud.points.size();i++)
 {
    octree.integrateNodeColor( cloud.points[i].x, cloud.points[i].y, cloud.points[i].z,
     cloud.points[i].r, cloud.points[i].g, cloud.points[i].b );
 }
 

// 更新octomap
  octree.updateInnerOccupancy();
  cout<<"data copied into octomap."<<endl;
  
  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);

  bmap_msg.header.frame_id = "octomap-from-pcd";

  ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("octo_show_from_pcd",1);

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
