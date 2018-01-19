#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void readCalibration(string child_frame, Eigen::Matrix4f& cal)
{
  string calibFile;

  calibFile = ros::package::getPath("calibration_glasgow") + "/" + child_frame + ".calib";
  ROS_INFO("Calibration read from %s: ",calibFile.c_str());

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_publisher");
  if (argc != 2)
  {
      cout<<"Usage: need a xxx.pcd"<<endl;
      return -1;
  }

  ros::NodeHandle nh;

  PointCloud::Ptr cloud_in (new PointCloud);
  PointCloud::Ptr cloud (new PointCloud);

  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud_in) == -1) 
  {
    PCL_ERROR ("Couldn't read pcd file. \n");
    return (-1);
  }

  cout<<"point cloud loaded, piont size = "<<cloud_in->points.size()<<endl;

  //读取标定文件内的矩阵变换数据
  Eigen::Matrix4f cam2rb = Eigen::Matrix4f::Identity();  
  readCalibration("camera2rb", cam2rb); //OK!
  cout<<cam2rb<<endl;  
   
  pcl::transformPointCloud(*cloud_in,*cloud,cam2rb);
  //pcl::io::savePCDFileASCII (argv[2], *cloud); 
  //cout<<"point cloud transformed saved."<<endl;

 
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
  cout<<"data copied into octomap."<<endl;
 
//octomap消息
  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(octree, bmap_msg);
  //bmap_msg.header.frame_id = "world";
  std::cout<<std::endl;

  //publisher for the planning scene
  ros::Publisher octomap_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
                                          //move_group/monitored_planning_scene   planning_scene
  while (octomap_pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
  
  ros::Rate loop_rate(10);

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.name = "redbox";
  //planning_scene.robot_model_name = "robot_description"; 
  planning_scene.world.octomap.header.frame_id = "world";
  planning_scene.world.octomap.octomap.header.frame_id = "world";
  planning_scene.world.octomap.octomap.binary = true;
  planning_scene.world.octomap.octomap.id = "OcTree";
  planning_scene.world.octomap.octomap.data = bmap_msg.data;

  planning_scene.is_diff = false;

  while (octomap_pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
  int num = octomap_pub.getNumSubscribers();
  std::cout<<"octomap的订阅者数目："<<num<<std::endl;

  ROS_INFO("more than one subscriber, start publishing msgs on and on...");

  int count = 0;
  while (ros::ok())
  { 
    planning_scene.world.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
    octomap_pub.publish(planning_scene);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if(count > 20)
    {
      ROS_INFO("Sending data...");
      count =0;
    }
    
  } 
  return 0;
}