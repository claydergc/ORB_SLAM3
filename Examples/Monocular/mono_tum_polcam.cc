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

#include "Constants.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <csignal> // For signal handling

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

struct Tuple {
    double first;
    float second;
    float third;
    int fourth;
};

// Global flag to indicate if Ctrl+C was pressed
volatile sig_atomic_t g_signal_received = 0;

void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\nCtrl+C detected! Shutting down gracefully..." << std::endl;
        g_signal_received = 1; // Set the flag
    }
}

struct PoseStamped
{
    double timestamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

void SaveTrajectoryForAllFramesClayder(const string &filename, std::vector<PoseStamped> trajectory)
{
    cout << endl << "Saving Clayder frames trajectory to " << filename << " ..." << endl;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f(filename.c_str());
    f.imbue(std::locale::classic());

    // CSV header
    // f << "ts (ns),tx (m),ty (m),tz (m),qx,qy,qz,qw\n";

    for (const PoseStamped& pose : trajectory)
    {
        double t = pose.timestamp;
        double x = pose.x;
        double y = pose.y;
        double z = pose.z;

        double qx = pose.qx;
        double qy = pose.qy;
        double qz = pose.qz;
        double qw = pose.qw;

        long long ts_ns = static_cast<long long>(std::round(t * 1e9));

        f << std::fixed << std::setprecision(9) << ts_ns << ','
          << std::scientific << std::setprecision(7)
          << static_cast<double>(x) << ','
          << static_cast<double>(y) << ','
          << static_cast<double>(z) << ','
          << static_cast<double>(qx) << ','
          << static_cast<double>(qy) << ','
          << static_cast<double>(qz) << ','
          << static_cast<double>(qw) << '\n';
    }

    f.close();
}

void SaveKeypointsForAllFramesClayder(const string &filename, std::vector<std::pair<double, uint16_t>> matchedKeypointsPerFrame)
{
    cout << endl << "Saving Clayder keypoints trajectory to " << filename << " ..." << endl;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f(filename.c_str());
    f.imbue(std::locale::classic());

    // CSV header
    // f << "ts (ns),tx (m),ty (m),tz (m),qx,qy,qz,qw\n";

    for (uint32_t i = 0; i < matchedKeypointsPerFrame.size(); i++)
    {
        double t = matchedKeypointsPerFrame[i].first;
        uint16_t numKeypointsPerFrame = matchedKeypointsPerFrame[i].second;
        long long ts_ns = static_cast<long long>(std::round(t * 1e9));

        f << std::fixed << std::setprecision(9) << ts_ns << ','
          << numKeypointsPerFrame << '\n';
    }

    f.close();
}



int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    //if(argc != 4)
    // if(argc != 6)
    if(argc != 7)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_imgs_cam0 path_to_imgs_cam1 path_to_keyframe_trajectory" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/polcamI/rgb.txt";


    string strFileCam0 = string(argv[3])+"/rgb.txt";


    LoadImages(strFileCam0, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM01, true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat imCam0;
    cv::Mat imCam1;
    cv::Mat imAux; //= cv::Mat::zeros(480, 640, CV_8U);
    int mState;

    //added by claydergc

    // std::ofstream mprFile("map_points_ratio.txt");
    // mprFile<<std::fixed<<std::setprecision(9);
    std::vector<Tuple> mprVector;
    float pitchPrev=0.0;
    float pitchCurr=0.0;
    float pitchDelta=0.0;
    uint16_t nMapPoints=0;

    // std::ofstream file_n_keypoints_diff("n_polcam_kp.txt");
    // file_n_keypoints_diff<<std::fixed<<std::setprecision(9);
    // std::vector<std::pair<double, int>> vec_n_keypoints_diff;

    //cv::Rect topHalf(0, 0, 640, 130);
    //cv::Rect topHalf(0, 0, 606, 254);
    //cv::Rect topHalf(0, 0, 640, 90);
    //cv::Rect topHalf(0, 0, 640, 80);
    //cv::Rect topHalf(0, 200, 640, 280);

    cv::Rect mask(0, 0, 606, 140);

    std::vector<cv::KeyPoint> kp0;

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    std::vector<std::pair<double, int>> vec_N_Polcam;

    std::vector<PoseStamped> trajectory;


    // std::cout<<nImages<<std::endl;

    for(int ni=0; ni<nImages && !g_signal_received; ni++)
    //for(int ni=0; ni<1410 && !g_signal_received; ni++)
    {
        // Read image from file

        imCam0 = cv::imread(string(argv[3])+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        imCam1 = cv::imread(string(argv[4])+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI0_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        // imPolcam = cv::imread(string(argv[3])+"/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI90_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        // imPolcam = cv::imread(string(argv[3])+"/polcamI135_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI0/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI45/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        //im = cv::imread(string(argv[3])+"/polcamI0/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //imPolcam = cv::imread(string(argv[3])+"/polcamI45/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI0/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI90/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI0/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI45/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI135/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI90_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im  = cv::imread(string(argv[3])+"/polcamI90/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI135/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im  = cv::imread(string(argv[3])+"/polcamI135/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI45/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);   // works well for 0830  and 0839full resolution

        // im  = cv::imread(string(argv[3])+"/polcamI0_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);   // works well for 0830  and 0839full resolution
        // imPolcam = cv::imread(string(argv[3])+"/polcamI135_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);   // works well for 0830 full resolution

        // im = cv::imread(string(argv[3])+"/polcamI0_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI135_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); // works well for 0839 full resolution
        // imPolcam = cv::imread(string(argv[3])+"/polcamI90_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // im = cv::imread(string(argv[3])+"/polcamI0_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imPolcam = cv::imread(string(argv[3])+"/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); // works well for 1746 full resolution




        double tframe = vTimestamps[ni];

        //im(mask) = 0;
        //imPolcam(mask) = 0;

        // clahe->apply(im,im);
        // clahe->apply(imPolcam,imPolcam);
        //


        // std::cout<<(argv[3]+vstrImageFilenames[ni])<<std::endl;
        // std::cout<<(argv[4]+vstrImageFilenames[ni])<<std::endl;

        if(imCam0.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
          //std::cout<<"HEY"<<std::endl;
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            //int width = im.cols * imageScale;
            //int height = im.rows * imageScale;
            //cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            //t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            //SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        //double alpha = 1.8; // contrast control (1.0 = no change)
        //int beta = 20;      // brightness control (0 = no change)

        // Apply transformation
        //im.convertTo(im, -1, alpha, beta);
        // Pass the image to the SLAM system
        //SLAM.TrackMonocular(im,tframe);
        //SLAM.TrackMonocularPolcam(im,imPolcam,tframe,vec_n_keypoints_diff);
        // SLAM.TrackMonocularPolcam(imCam0,imCam1,tframe);

        Sophus::SE3f Tcw = SLAM.TrackMonocularPolcam(imCam0,imCam1,tframe);
        Sophus::SE3f Twc = Tcw.inverse();

        // std::cout<<tframe<<" "<<Tcw.translation()<<std::endl;

        double tx = Twc.translation().x();
        double ty = Twc.translation().y();
        double tz = Twc.translation().z();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        trajectory.push_back({
            tframe,
            tx, ty, tz,
            q.x(), q.y(), q.z(), q.w()
        });

        // vec_N_Polcam.push_back(std::make_pair(SLAM.mpTracker->mCurrentFrame.mTimeStamp, SLAM.mpTracker->mCurrentFrame.N_Cam1));

        //std::cout<<im.cols<<std::endl;


        // Sophus::SE3f pose = SLAM.TrackMonocular(im,tframe).inverse();

        //std::cout<<SLAM.mTrackingState<<"->"<<pose.translation().transpose()<<std::endl;
        //std::cout<<"HOLA"<<std::endl;

        //added by claydergc

        /*if(ni>0) {
          pitchCurr = SLAM.mpTracker->mCurrentFrame.GetPose().rotationMatrix().transpose().eulerAngles(0,1,2)[1]*180.0/M_PI;
          pitchDelta = abs(pitchCurr-pitchPrev);
          pitchPrev = pitchCurr;

          nMapPoints = 0;

          for(uint16_t i=0; i<SLAM.mpTracker->mCurrentFrame.mvpMapPoints.size(); ++i) {
            if(SLAM.mpTracker->mCurrentFrame.mvpMapPoints[i]!=nullptr)
              nMapPoints++;
          }

          if(pitchDelta!=0) {
            mprVector.push_back({tframe, nMapPoints, pitchDelta, SLAM.mTrackingState});
          }

        }

        if(ni==0) {
          pitchPrev = SLAM.mpTracker->mCurrentFrame.GetPose().rotationMatrix().transpose().eulerAngles(0,1,2)[1]*180.0/M_PI;
        }*/

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];


        //cv::DMatch myMatch(1,1,1.0f);
        //std::vector<cv::DMatch> myMatches2;
        //myMatches2.reserve(30);

        //myMatches2[0] = cv::DMatch(1,1,1.0f);

        //std::cout<<"MyMatch: "<<myMatches2[1].queryIdx<<std::endl;

        /*
        //if(ni==220) {
        //if(false) {
        //if(ni==40) {
        if(ni==120) {
          kp0 = SLAM.mpTracker->mCurrentFrame.mvKeysUn;
        }

        //if(false) {
        //if(ni>40) {
        //if(ni>220) {
        if(ni>120) {

          std::vector<cv::DMatch> myMatches = SLAM.mpTracker->matcher.myMatches;
          std::vector<cv::KeyPoint> kp1 = SLAM.mpTracker->mCurrentFrame.mvKeys;

          //std::cout<<myMatches.size()<<std::endl;

          for (const auto& match : myMatches) {

            if(match.queryIdx<0)
              continue;
            //if(match.queryIdx>0)
            //  std::cout<<match.queryIdx<<std::endl;
            //continue;

            cv::Point2f pt0 = kp0[match.queryIdx].pt;
            cv::Point2f pt1 = kp1[match.trainIdx].pt;

            // Filter very large displacements (optional)
            //if (cv::norm(pt0 - pt1) > 10.0 && cv::norm(pt0 - pt1) < 50.0)
            //  isBigMatchDiff = true;

            //if(pt0!=nullptr && pt1!=nullptr)
            //std::cout<<pt0.x<<" "<<pt1.x<<std::endl;
            //std::cout<<kp0.size()<<" "<<match.queryIdx<<" "<<kp1.size()<<" "<<match.trainIdx<<std::endl;

            //if (cv::norm(pt0 - pt1) > 5.0 && cv::norm(pt0 - pt1) < 50.0) {
                cv::line(imAux, pt0, pt1, cv::Scalar(0, 255, 0), 1);
            //}

            //points.push_back(pt1);
          }

          kp0 = kp1;

          cv::imshow("imAUx", imAux);
        }*/



        //cv::waitKey(0);

        if(ttrack<T)
            usleep((T-ttrack)*1e6);


    }

    // Stop all threads
    SLAM.Shutdown();

    //cv::destroyAllWindows();


    //added by claydergc
    //for(uint16_t i=0; i<mprVector.size(); ++i)
      //mprFile<<mprVector[i].first<<" "<<mprVector[i].second<<" "<<mprVector[i].third<<" "<<mprVector[i].fourth<<"\n";
    //mprFile.close();

    //for(uint16_t i=0; i<vec_n_keypoints_diff.size(); ++i)
    //  file_n_keypoints_diff<<vec_n_keypoints_diff[i].first<<" "<<vec_n_keypoints_diff[i].second<<"\n";
    //file_n_keypoints_diff.close();

    // for(uint16_t i=0; i<vec_N_Polcam.size(); ++i)
    //   file_n_keypoints_diff<<vec_N_Polcam[i].first<<" "<<vec_N_Polcam[i].second<<"\n";
    // file_n_keypoints_diff.close();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM("KeyFrameTrajectory.txt");

    // SLAM.SaveKeyFrameTrajectoryTUM(string(argv[5]));
    SaveKeypointsForAllFramesClayder(string(argv[6]), SLAM.mpTracker->matchedKeypointsPerFrame);
    SaveTrajectoryForAllFramesClayder(string(argv[5]), trajectory);

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
