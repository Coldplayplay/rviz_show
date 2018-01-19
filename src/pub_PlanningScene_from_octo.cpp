#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_publisher");
  if (argc != 2)
  {
      std::cout<<"Usage: need a xxx.bt"<<std::endl;
      return -1;
  }

  ros::NodeHandle n;

  //publisher for the planning scene
  ros::Publisher octomap_pub = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  ros::Rate loop_rate(10);

  octomap::OcTree* octree = new octomap::OcTree(argv[1]);
  //octomap::OcTree octree(argv[1]);//"redbox.bt"
  std::cout << "File Read Sccessfully" << std::endl;
  int count = 0;
  static octomap_msgs::Octomap octomap;
  octomap_msgs::binaryMapToMsg(*octree, octomap);//如果是不是指针形式，执行这一句会出现Writing 8089 nodes to output stream...
  std::cout << "write to msg over." << std::endl;

  //定义planningscene的消息
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.octomap.header.frame_id = "odom_combined";
  planning_scene.world.octomap.header.stamp = ros::Time::now();
  planning_scene.world.octomap.octomap.header.frame_id = "odom_combined";
  planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
  planning_scene.world.octomap.octomap.binary = true;
  planning_scene.world.octomap.octomap.id = "OcTree";
  planning_scene.world.octomap.octomap.data = octomap.data;

  octomap_pub.publish(planning_scene);

   while (octomap_pub.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }
  int num = octomap_pub.getNumSubscribers();
  std::cout<<"octomap的订阅者数目："<<num<<std::endl;

  ROS_INFO("more than one subscriber, start publishing msgs on and on...");

  while (ros::ok())
  {          
      
    planning_scene.world.octomap.header.frame_id = "odom_combined";
    planning_scene.world.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.header.frame_id = "odom_combined";
    planning_scene.world.octomap.octomap.header.stamp = ros::Time::now();
    planning_scene.world.octomap.octomap.binary = true;
    planning_scene.world.octomap.octomap.id = "OcTree";
    planning_scene.world.octomap.octomap.data = octomap.data;

    //ROS_INFO("Adding the octomap into the world.");
    octomap_pub.publish(planning_scene);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  } 
  
  return 0;
}