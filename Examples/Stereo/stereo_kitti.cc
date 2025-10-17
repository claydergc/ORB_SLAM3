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
#include<iomanip>
#include<chrono>
#include <csignal> // For signal handling


#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

// Helper to normalize angles to [-180, 180)
inline float normalizeAngle(float angle_deg) {
    if (angle_deg >= 180.0f) angle_deg -= 180.0f;
    if (angle_deg < -180.0f) angle_deg += 180.0f;
    return angle_deg;
}

struct Tuple {
    double first;
    float second;
    float third;
    uint16_t fourth;
    int fifth;
};

// Global flag to indicate if Ctrl+C was pressed
volatile sig_atomic_t g_signal_received = 0;

void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "\nCtrl+C detected! Shutting down gracefully..." << std::endl;
        g_signal_received = 1; // Set the flag
    }
}


int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    std::cout<<"Before"<<std::endl;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    std::cout<<"After"<<std::endl;
    const int nImages = vstrImageLeft.size();
    std::cout<<"After1"<<std::endl;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);
    std::cout<<"After2"<<std::endl;
    float imageScale = SLAM.GetImageScale();
    std::cout<<"After3"<<std::endl;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    std::ofstream mprFile("map_points_ratio.txt");
    mprFile<<std::fixed<<std::setprecision(9);
    std::vector<Tuple> mprVector;


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    double t_track = 0.f;
    double t_resize = 0.f;
    

    float pitchPrev=0.0;
    float pitchCurr=0.0;
    float pitchDelta=0.0;
    uint16_t nMapPoints=0;


    // Main loop
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages && !g_signal_received; ni++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        
        
        if(imLeft.cols!=640 && imLeft.rows!=480) {
          cv::resize(imLeft, imLeft, cv::Size(640,480));
          cv::resize(imRight, imRight, cv::Size(640,480));
        }
        
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);
        
        if(ni>0) {
          pitchCurr = SLAM.mpTracker->mCurrentFrame.GetPose().rotationMatrix().transpose().eulerAngles(0,1,2)[1]*180.0/M_PI;
          pitchDelta = abs(pitchCurr-pitchPrev);
          //float pitchDelta = abs(normalizeAngle(pitchCurr)-normalizeAngle(pitchPrev));
          //std::cout<<"Pitch curr: "<<pitchCurr<<", pitchPrev: "<<pitchPrev<<std::endl;
          pitchPrev = pitchCurr;
          
          nMapPoints = 0;
          
          for(uint16_t i=0; i<SLAM.mpTracker->mCurrentFrame.mvpMapPoints.size(); ++i) {
            if(SLAM.mpTracker->mCurrentFrame.mvpMapPoints[i]!=nullptr)
              nMapPoints++;
          }
          
          //std::cout<<"Map points: "<<mCurrentFrame.mvpMapPoints.size()<<", pitchDelta: "<<pitchDelta<<". MP/PD: "<<mCurrentFrame.mvpMapPoints.size()/pitchDelta<<std::endl;
          
          if(pitchDelta!=0) {
          
            //std::cout<<"Map points: "<<nMapPoints<<", pitchDelta: "<<pitchDelta<<". MP/PD: "<<nMapPoints/pitchDelta<<std::endl;
          
            //mprFile<<tframe<<" "<<nMapPoints/pitchDelta<<"\n";
            
            //mprVector.push_back(std::make_pair(tframe, nMapPoints/pitchDelta));
            //mprVector.push_back({tframe, nMapPoints/pitchDelta, SLAM.mTrackingState});
            mprVector.push_back({tframe, nMapPoints, pitchDelta, SLAM.mpTracker->th_global, SLAM.mTrackingState});
          }
          
        }
        
        if(ni==0) {
          pitchPrev = SLAM.mpTracker->mCurrentFrame.GetPose().rotationMatrix().transpose().eulerAngles(0,1,2)[1]*180.0/M_PI;
        }

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
    
    //mprFile.close();
    // Stop all threads
    SLAM.Shutdown();
    
    
    for(uint16_t i=0; i<mprVector.size(); ++i)
      mprFile<<mprVector[i].first<<" "<<mprVector[i].second<<" "<<mprVector[i].third<<" "<<mprVector[i].fourth<<" "<<mprVector[i].fifth<<"\n";
    
    mprFile.close();

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
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/left/";
    string strPrefixRight = strPathToSequence + "/right/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + "_left.png";
        vstrImageRight[i] = strPrefixRight + ss.str() + "_right.png";
    }
}
