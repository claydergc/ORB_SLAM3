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
#include "PolarizationCameraUtils.h"

#include <cmath>
#include <cstdint>
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

void SaveKeypointsForAllFramesClayder(const string &filename, std::vector<std::pair<double, uint16_t>> matchedKeypointsPerFrame, std::vector<std::pair<double, uint8_t>> isNewKeyFrameVector)
{
    cout << endl << "Saving Clayder keypoints trajectory to " << filename << " ..." << endl;

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f(filename.c_str());
    f.imbue(std::locale::classic());

    // CSV header
    // f << "ts (ns),tx (m),ty (m),tz (m),qx,qy,qz,qw\n";
    //
    std::cout<<matchedKeypointsPerFrame.size()<<' '<<isNewKeyFrameVector.size()<<std::endl;

    for (uint32_t i = 0; i < matchedKeypointsPerFrame.size(); i++)
    {
        double t = matchedKeypointsPerFrame[i].first;
        double t2 = isNewKeyFrameVector[i+1].first;

        // std::cout<<t<<' '<<t2<<std::endl;

        uint16_t numKeypointsPerFrame = matchedKeypointsPerFrame[i].second;
        uint8_t isNewKeyFrame = isNewKeyFrameVector[i+1].second;
        long long ts_ns = static_cast<long long>(std::round(t * 1e9));
        // long long ts_ns2 = static_cast<long long>(std::round(t2 * 1e9));

        // std::cout<<ts_ns<<' '<<ts_ns2<<std::endl;

        // if(ts_ns==ts_ns2) {

        f << std::fixed << std::setprecision(9) << ts_ns << ','
        << numKeypointsPerFrame
        << ',' << (uint16_t)(isNewKeyFrame) << '\n';
        // }
    }

    f.close();
}



int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    //if(argc != 4)
    // if(argc != 6)
    // if(argc != 7)
    // {
    //     cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_imgs_cam0 path_to_imgs_cam1 path_to_keyframe_trajectory" << endl;
    //     return 1;
    // }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/polcamI/rgb.txt";


    string strFileCam0 = "/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024/rgb.txt";


    LoadImages(strFileCam0, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM01, true);
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM0, true);
    // ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, ORB_SLAM3::Constants::POLCAM0, false);
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
    cv::Mat imCam2;
    cv::Mat imCam3;

    cv::Mat I0;
    cv::Mat I45;
    cv::Mat I90;
    cv::Mat I135;
    cv::Mat I;
    cv::Mat Itheta0;
    cv::Mat Itheta1;

    double current_pol_angle0 = 45.0*M_PI/180.0;
    double current_pol_angle1 = 90.0*M_PI/180.0;

    cv::Mat imAux; //= cv::Mat::zeros(480, 640, CV_8U);
    int mState;

    cv::Rect mask(0, 0, 606, 140);

    std::vector<cv::KeyPoint> kp0;

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    std::vector<std::pair<double, int>> vec_N_Polcam;

    std::vector<PoseStamped> trajectory;

    uint16_t imCounter = 0;

    // std::cout<<nImages<<std::endl;

    for(int ni=0; ni<nImages && !g_signal_received; ni++)
    //for(int ni=0; ni<1410 && !g_signal_received; ni++)
    {
        // Read image from file

        I0 = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        I45 = cv::imread(string(argv[4])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        I90 = cv::imread(string(argv[5])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        I135 = cv::imread(string(argv[6])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // I = cv::imread(string(argv[7])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // imCam0 = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // imCam1 = cv::imread(string(argv[4])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //


        // imCam0 = I;
        // imCam1 = I45;
        // imCam1 = I;
        //
        // I and I90 work best at Yandiwanba
        // imCam0 = I;
        // imCam1 = I90;
        // With I0 and I45, it fails sometimes
        // imCam1 = I0;
        // imCam1 = I45;

        // Works well at Yandiwanba
        // imCam0 = I90;
        // imCam1 = I135;
        //


        double tframe = vTimestamps[ni];


        // if(ni>1312 && ni<1792)
        // // if(ni>=1312)
        // {
        //     // std::cout<<ni<<std::endl;
        //     // imCam0 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI0_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam0 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI90_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam0 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI135_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam0 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam1 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam1 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam1 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI90_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     imCam0 = cv::imread(string(argv[5])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        //     imCam1 = cv::imread(string(argv[6])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // }
        // else {
        //     // imCam0 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam0 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI45_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        //     // imCam1 = cv::imread("/home/ros-noetic/datasets/Polcam02/KelvinGrove/20260128/0835/polcamI90_1224x1024/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        //     imCam0 = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE); //Work best in 0830!!!
        //     imCam1 = cv::imread(string(argv[4])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
        // }

        // if(ni>1350 && ni<1792) {

        // if(ni>200) {
        if(ni>1144 && ni<1792) {

            // if(imCounter%100==0 && current_pol_angle>5*M_PI/180.0) {
            // if(imCounter%60==0 && current_pol_angle0>2*M_PI/180.0) {
            // if(current_pol_angle0>2*M_PI/180.0) {
            // if(imCounter%60==0 && current_pol_angle0>1*M_PI/180.0) {

            // if(imCounter%60==0)


            if(imCounter%30==0 &&current_pol_angle0>0*M_PI/180.0) {
            // if(current_pol_angle0>1*M_PI/180.0) {

                //imCounter = 0;
                // Itheta = PolarizationCameraUtils::computePolarizationAngleImageParallel(I0, I45, I90, I135, current_pol_angle0);

                //if(current_pol_angle>5*M_PI/180.0)
                // current_pol_angle0 = current_pol_angle0 - 1.0*M_PI/180.0;
                current_pol_angle0 = current_pol_angle0 - 5.0*M_PI/180.0;
                // std::cout<<current_pol_angle0*180.0/M_PI<<std::endl;

                //imCam0 = Itheta;
            }

            if(imCounter%30==0 && current_pol_angle1>45*M_PI/180.0) {
                //imCounter = 0;

                current_pol_angle1 = current_pol_angle1 - 5.0*M_PI/180.0;


                //imCam0 = Itheta;
            }


            Itheta0 = PolarizationCameraUtils::computePolarizationAngleImageParallel(I0, I45, I90, I135, current_pol_angle0);
            Itheta1 = PolarizationCameraUtils::computePolarizationAngleImageParallel(I0, I45, I90, I135, current_pol_angle1);
            imCam0 = Itheta0;
            imCam1 = Itheta1;

            if(imCounter%30==0) {
                imCounter = 0;
                std::cout<<current_pol_angle0*180.0/M_PI<<" "<<current_pol_angle1*180.0/M_PI<<std::endl;
            }

            // else {
            //     // imCam0 = I0;
            //     Itheta = PolarizationCameraUtils::computePolarizationAngleImageParallel(I0, I45, I90, I135, 0.0);
            //     imCam0 = Itheta;
            // }

            imCounter++;
            // imCam0 = I0;
        }
        else {
            // imCam0 = I;
            imCam0 = I45;
            imCam1 = I90;
        }
        //
        // imCam0 = I45;
        // imCam1 = I90;

        // imCam1 = I90;

        // if (current_pol_angle<5*M_PI/180.0) {
        //     imCam0 = I0;
        // }


        //im(mask) = 0;
        //imPolcam(mask) = 0;

        // clahe->apply(im,im);
        // clahe->apply(imPolcam,imPolcam);
        //

        // std::cout<<I.empty()<<std::endl;
        // std::cout<<imCam0.empty()<<std::endl;

        if(imCam0.empty() || imCam1.empty())
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
        //



        // if(ni>536) {

        //     // if(imCounter%100==0 && current_pol_angle>5*M_PI/180.0) {
        //     if(imCounter%60==0 && current_pol_angle>5*M_PI/180.0) {

        //         imCounter = 0;
        //         Itheta = PolarizationCameraUtils::computePolarizationAngleImageParallel(I0, I45, I90, I135, current_pol_angle);

        //         //if(current_pol_angle>5*M_PI/180.0)
        //         current_pol_angle = current_pol_angle - 10.0*M_PI/180.0;
        //         std::cout<<current_pol_angle*180.0/M_PI<<std::endl;

        //         imCam0 = Itheta;
        //     }

        //     imCounter++;
        // }

        // if (current_pol_angle<5*M_PI/180.0) {
        //     imCam0 = I0;
        // }


        // if(ni>400) {

        //     SLAM.setPolcamMode(ORB_SLAM3::Constants::POLCAM0);
        // }

        Sophus::SE3f Tcw = SLAM.TrackMonocularPolcam(imCam0,imCam1,tframe);
        // Sophus::SE3f Tcw = SLAM.TrackMonocular(imCam0,tframe);
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


    SLAM.SaveKeyFrameTrajectoryTUM(string(argv[7]));
    SaveTrajectoryForAllFramesClayder(string(argv[8]), trajectory);
    SaveKeypointsForAllFramesClayder(string(argv[9]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);

    // SLAM.SaveKeyFrameTrajectoryTUM(string(argv[8]));
    // SaveTrajectoryForAllFramesClayder(string(argv[9]), trajectory);
    // SaveKeypointsForAllFramesClayder(string(argv[10]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);
    //
    // SLAM.SaveKeyFrameTrajectoryTUM(string(argv[5]));
    // SaveTrajectoryForAllFramesClayder(string(argv[6]), trajectory);
    // SaveKeypointsForAllFramesClayder(string(argv[7]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);

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
