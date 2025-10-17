/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/core/core.hpp>

#include "/home/ros-noetic/src/ORB_SLAM3/include/Frame.h"
#include "/home/ros-noetic/src/ORB_SLAM3/include/ImuTypes.h"
#include "/home/ros-noetic/src/ORB_SLAM3/include/KeyFrame.h"
#include "/home/ros-noetic/src/ORB_SLAM3/include/System.h"
#include "KeyFrame.h"

#include <geometry_msgs/PoseStamped.h>
#include <orb_slam3/KeyPointArray.h>

using namespace std;

class ImuGrabber {
public:
  ImuGrabber(){};
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutex;
};

class ImageGrabber {
public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bRect,
               const bool bClahe)
      : mpSLAM(pSLAM), mpImuGb(pImuGb), do_rectify(bRect), mbClahe(bClahe) {}

  void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);
  void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);
  cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
  void SyncWithImu();

  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
  std::mutex mBufMutexLeft, mBufMutexRight;
  std::mutex keyframe_keys_mutex;

  ORB_SLAM3::System *mpSLAM;
  ORB_SLAM3::KeyFrame *last_keyframe;

  ImuGrabber *mpImuGb;

  const bool do_rectify;
  cv::Mat M1l, M2l, M1r, M2r;

  const bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

  ros::Publisher pub_keypoints_current_frame;
  ros::Publisher pub_keypoints_last_keyframe;
  // ros::Publisher pub_keypoints_prev_frame;
  ros::Publisher pub_pose;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "Stereo_Inertial");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  bool bEqual = false;
  if (argc < 4 || argc > 5) {
    cerr << endl
         << "Usage: rosrun ORB_SLAM3 Stereo_Inertial path_to_vocabulary "
            "path_to_settings do_rectify [do_equalize]"
         << endl;
    ros::shutdown();
    return 1;
  }

  std::string sbRect(argv[3]);
  if (argc == 5) {
    std::string sbEqual(argv[4]);
    if (sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);

  ImuGrabber imugb;
  ImageGrabber igb(&SLAM, &imugb, sbRect == "true", bEqual);

  if (igb.do_rectify) {
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      cerr << "ERROR: Wrong path to settings" << endl;
      return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() ||
        R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
        rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
      cerr << "ERROR: Calibration parameters to rectify stereo are missing!"
           << endl;
      return -1;
    }

    cv::initUndistortRectifyMap(
        K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3),
        cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
    cv::initUndistortRectifyMap(
        K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3),
        cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
  }

  // Maximum delay, 5 seconds
  ros::Subscriber sub_imu =
      n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
  ros::Subscriber sub_img_left = n.subscribe(
      "/camera/left/image_raw", 100, &ImageGrabber::GrabImageLeft, &igb);
  ros::Subscriber sub_img_right = n.subscribe(
      "/camera/right/image_raw", 100, &ImageGrabber::GrabImageRight, &igb);

  igb.pub_keypoints_current_frame =
      n.advertise<orb_slam3::KeyPointArray>("keypoints_current_frame", 10);
  // igb.pub_keypoints_prev_frame =
  //     n.advertise<orb_slam3::KeyPointArray>("keypoints_prev_frame", 10);
  igb.pub_keypoints_last_keyframe =
      n.advertise<orb_slam3::KeyPointArray>("keypoints_last_keyframe", 10);
  igb.pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 10);

  std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

  ros::spin();

  delete igb.mpSLAM;
  delete igb.mpImuGb;
  delete igb.last_keyframe;

  return 0;
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
  mBufMutexLeft.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  // std::cout<< "t_im_left: " << imgLeftBuf.front()->header.stamp.toSec() <<
  // std::endl;
  mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
  mBufMutexRight.lock();
  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(img_msg);
  // std::cout<< "t_im_right: " << imgRightBuf.front()->header.stamp.toSec() <<
  // std::endl;
  mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  if (cv_ptr->image.type() == 0) {
    return cv_ptr->image.clone();
  } else {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu() {
  const double maxTimeDiff = 0.01;
  geometry_msgs::PoseStamped pose_msg;

  while (1) {
    cv::Mat imLeft, imRight;
    double tImLeft = 0, tImRight = 0;
    if (!imgLeftBuf.empty() && !imgRightBuf.empty() &&
        !mpImuGb->imuBuf.empty()) {
      tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      tImRight = imgRightBuf.front()->header.stamp.toSec();

      std::cout << std::fixed;
      std::cout << std::setprecision(15);

      // std::cout << tImLeft << "---"  << tImRight << std::endl;

      this->mBufMutexRight.lock();
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
        imgRightBuf.pop();
        tImRight = imgRightBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRight.unlock();

      this->mBufMutexLeft.lock();
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
        imgLeftBuf.pop();
        tImLeft = imgLeftBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexLeft.unlock();

      if ((tImLeft - tImRight) > maxTimeDiff ||
          (tImRight - tImLeft) > maxTimeDiff) {
        // std::cout << "big img time difference" << std::endl;
        // std::cout << tImLeft << "---"  << tImRight << std::endl;
        continue;
      }

      if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec()) {
        // if( abs(tImLeft - mpImuGb->imuBuf.back()->header.stamp.toSec()) >
        // 0.07 ) { std::cout << "big imu time difference" << std::endl;
        continue;
      }

      this->mBufMutexLeft.lock();
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
      this->mBufMutexLeft.unlock();

      this->mBufMutexRight.lock();
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
      this->mBufMutexRight.unlock();

      // std::cout << tImLeft << "---"  << tImRight << std::endl;

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();

      // std::cout<<"IMU buf size: "<<mpImuGb->imuBuf.size()<<"\n";

      if (!mpImuGb->imuBuf.empty()) {
        // Load imu measurements from buffer
        // std::cout<<"HELLO\n";
        vImuMeas.clear();
        while (!mpImuGb->imuBuf.empty() &&
               mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
          // std::cout<<"HELLO222\n";
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                          mpImuGb->imuBuf.front()->linear_acceleration.y,
                          mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                          mpImuGb->imuBuf.front()->angular_velocity.y,
                          mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if (mbClahe) {
        mClahe->apply(imLeft, imLeft);
        mClahe->apply(imRight, imRight);
      }

      if (do_rectify) {
        cv::remap(imLeft, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r, M2r, cv::INTER_LINEAR);
      }

      // std::cout<<"size: "<<vImuMeas.size()<<std::endl;
      mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

      // added by claydergc
      // std::vector<cv::KeyPoint> keypoints;
      // keypoints.emplace_back(cv::KeyPoint(100, 150, 5.0, 45.0, 0.8, 1, -1));
      // keypoints.emplace_back(cv::KeyPoint(200, 250, 7.0, 90.0, 1.2, 2, 0));

      orb_slam3::KeyPointArray msg;
      // orb_slam3::KeyPointArray msg_last_keyframe;
      ORB_SLAM3::Frame currentFrame = mpSLAM->mpTracker->mCurrentFrame;
      std::vector<cv::KeyPoint> keypoints_current_frame = currentFrame.mvKeys;
      std::vector<bool> mvbMap = mpSLAM->mpFrameDrawer->mvbMap;
      std::vector<bool> mvbVO = mpSLAM->mpFrameDrawer->mvbVO;

      // currentFrame.GetPose();
      // kf = mpSLAM->mpTracker->GetLastKeyFrame(); // check that last key frame
      // is
      //  the correct variable

      // std::cout<<(kf==NULL)<<std::endl;
      //

      // std::vector<cv::KeyPoint> keypoints_prev_frame =
      //     currentFrame.mpPrevFrame->mvKeys;

      if (mvbVO.empty())
        continue;

      // Sophus::SE3f Twc = kf->GetPoseInverse();
      Sophus::SE3f Twc = currentFrame.GetPose().inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();

      // for (const auto& kp : keypoints) {

      // if (!mvbVO.empty()) {
      //

      for (int i = 0; i < keypoints_current_frame.size(); ++i) {

        cv::KeyPoint kp = keypoints_current_frame[i];

        // if (mvbVO[i] || mvbMap[i]) {

        if (mvbVO[i]) {
          orb_slam3::KeyPoint kp_msg;
          kp_msg.x = kp.pt.x;
          kp_msg.y = kp.pt.y;
          kp_msg.size = kp.size;
          kp_msg.angle = kp.angle;
          kp_msg.response = kp.response;
          kp_msg.octave = kp.octave;
          kp_msg.class_id = kp.class_id;

          msg.keypoints.push_back(kp_msg);
        }
      }

      pub_keypoints_current_frame.publish(msg);

      //}

      // if (!kf->isBad()) {
      // if (true) {
      //  publish POSE 3D
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "map"; // Change as needed

      // Assign position
      pose_msg.pose.position.x = t.x();
      pose_msg.pose.position.y = t.y();
      pose_msg.pose.position.z = t.z();

      // Assign orientation
      pose_msg.pose.orientation.w = q.w();
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();

      pub_pose.publish(pose_msg);
      //}
      //

      last_keyframe = currentFrame.mpLastKeyFrame;

      if (last_keyframe == NULL)
        continue;

      std::vector<cv::KeyPoint> keypoints_last_keyframe = last_keyframe->mvKeys;

      msg.keypoints.clear();

      // this->keyframe_keys_mutex.lock();

      for (int i = 0; i < keypoints_last_keyframe.size(); ++i) {

        cv::KeyPoint kp = keypoints_last_keyframe[i];

        orb_slam3::KeyPoint kp_msg;
        kp_msg.x = kp.pt.x;
        kp_msg.y = kp.pt.y;
        kp_msg.size = kp.size;
        kp_msg.angle = kp.angle;
        kp_msg.response = kp.response;
        kp_msg.octave = kp.octave;
        kp_msg.class_id = kp.class_id;

        msg.keypoints.push_back(kp_msg);
      }

      // this->keyframe_keys_mutex.unlock();

      pub_keypoints_last_keyframe.publish(msg);

      // mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

      std::chrono::milliseconds tSleep(1);
      // std::chrono::microseconds tSleep(5);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
  // std::cout<<"HELLO\n";
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}
