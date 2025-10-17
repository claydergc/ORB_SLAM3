/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"/home/ros-noetic/src/ORB_SLAM3/include/System.h"
#include"/home/ros-noetic/src/ORB_SLAM3/include/ImuTypes.h"

using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bRect, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


cv::Mat imLeft, imRight;
double t_im, t_imu;
ORB_SLAM3::System* SLAM = NULL;
//bool newImg = false;

void imageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if(SLAM==NULL)
    return;
    
  try
  {
    imLeft = cv_bridge::toCvShare(msg, "bgr8")->image;
    t_im = msg->header.stamp.toSec();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  //std::cout<<"Img\n";
}

void imageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if(&SLAM==NULL)
    return;
  
  try
  {
    imRight = cv_bridge::toCvShare(msg, "bgr8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  
  //newImg = true;
}

std::queue<sensor_msgs::ImuConstPtr> imuBuf;
std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

void imuCallback(const sensor_msgs::ImuConstPtr &msg)
{

  if(&SLAM==NULL)
    return;
  
  t_imu = msg->header.stamp.toSec();
  
  //std::cout<<t_im<<"-------"<<t_imu<<"\n";
  
  //std::cout<<"IMU\n";

  if( (t_im-t_imu)<=0.066666 ) {
  //if( t_imu<t_im ) {
  //if(newImg = true) {
    imuBuf.push(msg);
  }
  else {
  //if(t_imu > t_im) {
    //std::cout<<"imuBuf: " <<imuBuf.size()<<std::endl;
    vImuMeas.clear();
    
    while(!imuBuf.empty()) {
    
      double t = imuBuf.front()->header.stamp.toSec();
      cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
      cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
      vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
      imuBuf.pop();
    }
    
    //std::cout<<"vImuMeas: " <<vImuMeas.size()<<std::endl;
    
    SLAM->TrackStereo(imLeft,imRight,t_im,vImuMeas);
    std::chrono::milliseconds tSleep(1);
      //std::chrono::microseconds tSleep(100);
    std::this_thread::sleep_for(tSleep);
    //v_imu_msg.clear();
  }

  
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  if(argc < 4 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if(argc==5)
  {
    std::string sbEqual(argv[4]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  //ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);
  SLAM = new ORB_SLAM3::System(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

  // Maximum delay, 5 seconds
   
  //ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft,&igb);
  //ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight,&igb);
  //ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  
  ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 100, imageLeftCallback);
  ros::Subscriber sub_img_right = n.subscribe("/camera/left/image_raw", 100, imageRightCallback);
  ros::Subscriber sub_imu = n.subscribe("/imu", 1000, imuCallback);
  
  std::cout << std::fixed;
  std::cout << std::setprecision(15);

  //std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
  
  //while(ros::ok()) {
  
  //}

  ros::spin();
  
  delete SLAM;

  return 0;
}
