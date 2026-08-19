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
#include <string>

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

int computeSmoothAngleTransition(int theta_curr, int theta_target, int max_theta_diff) {
    int diff = theta_target - theta_curr;

    if (abs(diff)>=max_theta_diff) //saturate in 10
        return (diff<0)?theta_curr-max_theta_diff:theta_curr+max_theta_diff;
    else
        return theta_target;
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
    string strFile = string(argv[3])+"/rgb.txt";


    LoadImages(strFile, vstrImageFilenames, vTimestamps);

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
    cv::Mat polcam_img;
    cv::Mat Itheta0;
    cv::Mat Itheta1;

    cv::Mat imAux; //= cv::Mat::zeros(480, 640, CV_8U);
    int mState;

    std::vector<PoseStamped> trajectory;

    uint16_t imCounter = 0;

    int aolp_mean_curr = 0;
    int aolp_mean_prev = -999;
    int aolp_std = 0;
    double dolp_mean = 0;

    int theta0_curr = 155;
    int theta0_prev = 0;
    // uint16_t theta0_target = 0;
    int theta1_curr = 98;
    int theta1_prev = 0;

    double theta_3std_left;
    double theta_3std_right;

    const uint FPS = 20;
    const uint T_STEP = (uint)(2.0) * FPS ;
    const int Iangle = -1;
    const uint8_t THETA_SHIFT_MIN = 5; //degrees
    const double DOLP_MIN = 0.21;
    // const int THETA_MIN = -22; //degrees
    // const int THETA_MAX = 22; //degrees
    const int THETA_MIN = -40; //degrees
    const int THETA_MAX = 40; //degrees


    int theta0_curr_aux = 0;
    int theta1_curr_aux = 0;
    int theta0_target = 0;
    int theta1_target = 0;


    // for(int ni=0; ni<nImages && !g_signal_received; ni++)
    for(int ni=19; ni<nImages && !g_signal_received; ni++)
    // for(int ni=23; ni<nImages && !g_signal_received; ni++)
    //for(int ni=0; ni<1410 && !g_signal_received; ni++)
    {
        // Read image from file
        polcam_img = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);

        // std::array<double, 3> aolp_dolp_stats = PolarizationCameraUtils::demosaicPolImageAndComputeStats(polcam_img, (double)(theta0_curr)*M_PI/180.0, (double)(theta1_curr)*M_PI/180.0, Itheta0, Itheta1);
        std::array<double, 3> aolp_dolp_stats = PolarizationCameraUtils::demosaicPolImageAndComputeStats(polcam_img, 45*M_PI/180.0, 155*M_PI/180.0, Itheta0, Itheta1);

        aolp_mean_curr = aolp_dolp_stats[0];
        aolp_std = aolp_dolp_stats[1];
        dolp_mean = aolp_dolp_stats[2];

        // if(ni%T_STEP==0) //each T_STEP seconds
            // std::cout<<"dolp_mean: "<<dolp_mean<<std::endl;
            //
            //


        // if(ni==19) {
        //     theta0_curr_aux = theta_3std_left;
        //     theta1_curr_aux = aolp_mean_curr + 90;

        //     std::cout<<"theta0_curr_aux: "<<theta0_curr_aux<<" theta1_curr_aux: "<<theta1_curr_aux<<std::endl;
        // }

        if(dolp_mean>DOLP_MIN) {
            if(ni%T_STEP==0) { //each T_STEP seconds

                // theta_3std_left = aolp_mean_curr-2.3*aolp_std;
                // theta_3std_right = aolp_mean_curr+2.3*aolp_std;
                theta_3std_left = aolp_mean_curr-2.3*aolp_std;
                theta_3std_right = aolp_mean_curr+2.3*aolp_std;


                // std::cout<<"theta0_curr_aux: "<<theta0_curr_aux<<" theta1_curr_aux: "<<theta1_curr_aux<<std::endl;

                if(aolp_mean_curr>-8 && aolp_mean_curr<8){
                    theta0_curr_aux = theta_3std_left;
                    // theta0_curr_aux = theta_3std_right;
                    theta1_curr_aux = aolp_mean_curr + 90;

                    // add smooth transition to this
                }
                else if(aolp_mean_curr<THETA_MIN || aolp_mean_curr>THETA_MAX) {
                    theta0_curr_aux = aolp_mean_curr;
                    theta1_curr_aux = aolp_mean_curr + 90;
                }


                int aolp_mean_diff = aolp_mean_curr-aolp_mean_prev;

                if(std::abs(aolp_mean_diff)>=THETA_SHIFT_MIN) {
                // if(std::abs(aolp_mean_curr-aolp_mean_prev)>=THETA_SHIFT_MIN) {

                    // if(aolp_mean_curr>THETA_MIN && aolp_mean_curr<THETA_MAX) {

                        // std::cout<<"Turning - aolp_mean_curr: "<<aolp_mean_curr<<std::endl;

                        // if(aolp_mean_curr>0) {//skewed to right
                            // theta0_aux = theta_3std_left;
                        // } else if(aolp_mean_curr<0) {//skewed to left
                        //     theta0_aux = theta_3std_right;
                        // }

                        // theta0_aux = theta_3std_left;
                        // theta1_aux = aolp_mean_curr + 90;

                        // std::cout<<"-22<aolp<22"<<" theta0_aux: "<<theta0_aux<<" theta1_aux: "<<theta1_aux<<std::endl;
                        // std::cout<<"-22<aolp<22 aolp_mean_curr: "<<aolp_mean_curr<<" theta_3std_left: "<<theta_3std_left<<" theta_3std_right: "<<theta_3std_right<<std::endl;
                    // }
                    // else {

                    // if(aolp_mean_curr>8) {
                    //     theta0_curr_aux = theta_3std_right;
                    //     theta1_curr_aux = aolp_mean_curr + 90;
                    // }
                    // else if(aolp_mean_curr>-8) {
                    //     theta0_curr_aux = theta_3std_left;
                    //     theta1_curr_aux = aolp_mean_curr + 90;
                    // }

                        theta0_target = aolp_mean_curr;
                        theta1_target = aolp_mean_curr + 90;

                        // int diff = theta0_target - theta0_curr_aux;

                        std::cout<<"theta0_curr_aux: "<<theta0_curr_aux<<" theta0_target: "<<theta0_target<<std::endl;

                        // if (abs(diff)>=10)
                        //     theta0_curr_aux = (diff<0)?theta0_curr_aux-10:theta0_curr_aux+10;
                        // else
                        //     theta0_curr_aux = theta0_target;
                        //
                        theta0_curr_aux = computeSmoothAngleTransition(theta0_curr_aux, theta0_target, 10);

                        theta1_curr_aux = aolp_mean_curr + 90;

                        // std::cout<<"theta0_curr_aux: "<<theta0_curr_aux<<" theta1_curr_aux: "<<theta1_curr_aux<<std::endl;

                        // std::cout<<"else theta0_target: "<<theta0_target<<" theta1_target: "<<theta1_target<<std::endl;
                        // std::cout<<"else aolp_mean_curr: "<<aolp_mean_curr<<" theta_3std_left: "<<theta_3std_left<<" theta_3std_right: "<<theta_3std_right<<std::endl;
                    // }
                }
                // else {
                //     theta0_curr_aux = theta_3std_left;
                //     theta1_curr_aux = aolp_mean_curr + 90;
                // }


                theta0_curr = theta0_curr_aux;
                theta1_curr = theta1_curr_aux;


                // std::cout<<"std::abs(aolp_mean_curr-aolp_mean_prev): "<<std::abs(aolp_mean_curr-aolp_mean_prev)<<std::endl;
                std::cout<<"theta0_curr_aux: "<<theta0_curr_aux<<" theta1_curr_aux: "<<theta1_curr_aux<<std::endl;


                // else {
                //     theta0_curr_aux = aolp_mean_curr;;
                //     theta1_curr_aux = aolp_mean_curr + 90;

                //     std::cout<<"theta0_curr_aux: "<<theta0_curr_aux<<" theta1_curr_aux: "<<theta1_curr_aux<<std::endl;
                // }

                // std::cout<<"Turning - aolp_mean_curr: "<<aolp_mean_curr<<std::endl;

                aolp_mean_prev = aolp_mean_curr;
                theta0_prev = theta0_curr;
                theta1_prev = theta1_curr;
            }
        }

        imCam0 = Itheta0;
        imCam1 = Itheta1;


        // std::cout<<"theta0: "<<(int)(theta0)<<" theta1: "<<(int)(theta1)<<std::endl;

        double tframe = vTimestamps[ni];

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


    SLAM.SaveKeyFrameTrajectoryTUM(string(argv[4]));
    SaveTrajectoryForAllFramesClayder(string(argv[5]), trajectory);
    SaveKeypointsForAllFramesClayder(string(argv[6]), SLAM.mpTracker->matchedKeypointsPerFrame, SLAM.mpTracker->isNewKeyFrameVector);

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
