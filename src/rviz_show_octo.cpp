#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
//#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "rviz_show_octo");
 // ros::AsyncSpinner spinner(1);
  //spinner.start();
  ros::NodeHandle nh;

  std::cout<<"Reading File"<<std::endl;

  octomap::OcTree* octree = new octomap::OcTree("0000.bt");
  std::cout << "File Read Sccessfully" << std::endl;

  for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end=octree->end_leafs(); it!= end; ++it)
    { 
        std::cout << "Node center: " << it.getCoordinate();
        std::cout << " value: " << it->getValue() << "\n";
    }

  octomap_msgs::Octomap bmap_msg;
  octomap_msgs::binaryMapToMsg(*octree, bmap_msg);

  bmap_msg.header.frame_id = "octomap";

  ros::Publisher octomap_publisher = nh.advertise<octomap_msgs::Octomap>("octo_show",1);

  octomap_publisher.publish(bmap_msg);

  while(octomap_publisher.getNumSubscribers() < 1)
  {
       ROS_WARN("Waiting for Subscribers");
  }
  ros::Rate loop_rate(1);
  while (ros::ok())
    {
        octomap_publisher.publish(bmap_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }



  //ros::shutdown();
  return 0;
}
