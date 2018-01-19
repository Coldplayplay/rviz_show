
#include <iostream>
#include <assert.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
//#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void readCalibration(string child_frame, Eigen::Matrix4f& cal)
{
  string calibFile;

  calibFile = ros::package::getPath("calibration_glasgow") + "/" + child_frame + ".calib";
  ROS_INFO("Calibration read from \n%s: ",calibFile.c_str());

  std::ifstream file;
  file.open((calibFile).c_str());

  string buffer;
  for (int i=0; i<4; i++)
  {
    for (int j=0; j<4; j++)
    {
      file >> buffer;
      cal(i,j) = stof(buffer);

    }           

  }

  file.close();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_show_octo_from_pcd");

  if (argc < 2)
  {
    std::cout<<"Usage: need  xxx.pcd(raw) [xxx_trans.pcd] [xxx.bt]"<<std::endl;
    return -1;
  }
 
  ros::NodeHandle nh;

  PointCloud::Ptr cloud_in (new PointCloud);;
  PointCloud::Ptr cloud (new PointCloud);;


  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud_in) == -1) 
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  ROS_INFO_STREAM("Step 1 point cloud loaded, piont size = "<< cloud_in->points.size());

  //读取标定文件内的矩阵变换数据
  Eigen::Matrix4f cam2rb = Eigen::Matrix4f::Identity();  
  readCalibration("camera2rb", cam2rb); //OK!
  cout<<cam2rb<<endl;  
   
  pcl::transformPointCloud(*cloud_in,*cloud,cam2rb);
  ROS_INFO("Step 2 has transformed the point cloud.");

  if(argc>2)
  {
    pcl::io::savePCDFileASCII (argv[2], *cloud); 
    ROS_INFO("point cloud transformed saved.");
  } 
 
  // 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
  octomap::OcTree octree( 0.001 ); 

//用普通的for循环
  for(int i=0;i<cloud->points.size();i++)
{
    // 将点云里的点插入到octomap中
    octree.updateNode( octomap::point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z), true );
 }
 /*
 // 设置颜色
 for (int i=0;i<cloud->points.size();i++)
 {
    octree.integrateNodeColor( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z,
     cloud->points[i].r, cloud->points[i].g, cloud->points[i].b );
 }
 */

// 更新octomap
  octree.updateInnerOccupancy();
  ROS_INFO("Step 3 Data copied into octomap.");

  if(argc>3)
  {
    octree.writeBinary(argv[3]);
    cout<<endl;
    ROS_INFO("octomap saved.");
  }  

//pointcloud消息
  sensor_msgs::PointCloud2 cloud_msg;
  sensor_msgs::PointCloud2 cloud_trans_msg;
  pcl::toROSMsg(*cloud_in,cloud_msg);
  pcl::toROSMsg(*cloud,cloud_trans_msg);
  cloud_msg.header.frame_id="base";
  cloud_trans_msg.header.frame_id="base";

//octomap消息
  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  std:;cout<<std::endl;
  bmap_msg.header.frame_id = "base";
  
  ros::Publisher cloud_pub=nh.advertise<sensor_msgs::PointCloud2> ("zed_cloud",1);
  ros::Publisher cloud_pub1=nh.advertise<sensor_msgs::PointCloud2> ("zed_trans",1);
  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("octo_show_from_pcd",1);
 
  ROS_INFO("Step 4 All transformed to msgs and published.");

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cloud_pub.publish(cloud_msg);
    cloud_pub1.publish(cloud_trans_msg);
    octomap_pub.publish(bmap_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
