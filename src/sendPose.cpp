
#include <fstream>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

void readCalibration(string child_frame, Eigen::Affine3d& cal)
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
//四元数----->旋转矩阵3x3
cv::Mat getRotation(double x, double y, double z, double w)
{
    cv::Mat rotK = Mat::zeros(3,3,CV_32F);

    double sqw = w*w;
    double sqx = x*x;
    double sqy = y*y;
    double sqz = z*z;

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    double m11 = (-sqx + sqy - sqz + sqw)*invs ;
    double m22 = (-sqx - sqy + sqz + sqw)*invs ;

    double tmp1 = x*y;
    double tmp2 = z*w;
    double m10 = 2.0 * (tmp1 + tmp2)*invs ;
    double m01 = 2.0 * (tmp1 - tmp2)*invs ;

    tmp1 = x*z;
    tmp2 = y*w;
    double m20 = 2.0 * (tmp1 - tmp2)*invs ;
    double m02 = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = y*z;
    tmp2 = x*w;
    double m21 = 2.0 * (tmp1 + tmp2)*invs ;
    double m12 = 2.0 * (tmp1 - tmp2)*invs ;

    rotK.at<float>(0,0) = (float)m00;
    rotK.at<float>(0,1) = (float)m01;
    rotK.at<float>(0,2) = (float)m02;
    rotK.at<float>(1,0) = (float)m10;
    rotK.at<float>(1,1) = (float)m11;
    rotK.at<float>(1,2) = (float)m12;
    rotK.at<float>(2,0) = (float)m20;
    rotK.at<float>(2,1) = (float)m21;
    rotK.at<float>(2,2) = (float)m22;

    return rotK;
}

Eigen::Matrix3d euler2RotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle*pitchAngle*rollAngle;
    Eigen::Matrix3d r = q.matrix();
    return r;
}




int main(int argc, char** argv)
{
    Eigen::Affine3d cam2rb;
    readCalibration("camera2rb", cam2rb);
    Eigen::Matrix3d R2 = cam2rb.rotation();
    
    //四元数--->矩阵
    Eigen::Quaterniond q;
    q.x() = -0.0147629;
    q.y() = 0.704109;
    q.z() = 0.04774;
    q.w() = 0.708331;    
    Eigen::Matrix3d R1 = q.normalized().toRotationMatrix();
    
    //欧拉角--->矩阵
    Eigen::Matrix3d R0 = euler2RotationMatrix(M_PI, 0, 0);
    Eigen::Matrix3d R_mo = R2 * R1 * R0 ;//R0表示末端到横担，R1表示横担2相机，R2表示相机2base；
    
    cv::Mat R_m(3, 3, CV_64FC1);    
    cv::eigen2cv(R_mo, R_m);

    cv::Mat r_mo;
    cv::Rodrigues(R_m, r_mo);//矩阵2向量
    cout<<"机械臂末端姿态向量："<<"["<<r_mo.at<double>(0)<<", "<<r_mo.at<double>(1)<<", "<<r_mo.at<double>(2)<<"]"<<endl;
    return 0;

}