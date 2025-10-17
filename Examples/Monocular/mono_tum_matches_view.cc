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



int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
    //if(argc != 4)
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
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
    cv::Mat im;
    cv::Mat imAux; //= cv::Mat::zeros(480, 640, CV_8U);
    int mState;
    
    //added by claydergc
    /*
    std::ofstream mprFile("map_points_ratio.txt");
    mprFile<<std::fixed<<std::setprecision(9);
    std::vector<Tuple> mprVector;
    float pitchPrev=0.0;
    float pitchCurr=0.0;
    float pitchDelta=0.0;
    uint16_t nMapPoints=0;*/
    
    //cv::Rect topHalf(0, 0, 640, 130);
    //cv::Rect topHalf(0, 0, 640, 90);
    cv::Rect topHalf(0, 0, 640, 80);
    //cv::Rect topHalf(0, 200, 640, 280);
    
    std::vector<cv::KeyPoint> kp0;
    
    for(int ni=0; ni<nImages && !g_signal_received; ni++)
    {
        // Read image from file
        
        //std::cout<<string(argv[3])+"/"+vstrImageFilenames[ni]<<std::endl;
        //im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);        
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_GRAYSCALE);
          
          
        if(im.cols!=640 && im.rows!=480)
          cv::resize(im, im, cv::Size(640, 480));
                
        double tframe = vTimestamps[ni];

        //im(topHalf) = 0;
        cv::cvtColor(im, imAux, cv::COLOR_GRAY2BGR);

        //std::cout<<im.rows<<" "<<im.cols<<std::endl;

        if(im.empty())
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
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
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

        //double alpha = 1.8; // contrast control (1.0 = no change)
        //int beta = 20;      // brightness control (0 = no change)

        // Apply transformation
        //im.convertTo(im, -1, alpha, beta);
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);
                
        
        //Sophus::SE3f pose = SLAM.TrackMonocular(im,tframe).inverse();
        
        //std::cout<<SLAM.mTrackingState<<"->"<<pose.translation().transpose()<<std::endl;
        //std::cout<<"HOLA"<<std::endl;
        
        //added by claydergc
        /*
        if(ni>0) {
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
        }

        
        
        //cv::waitKey(0); 
        
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
            
        
    }

    // Stop all threads
    SLAM.Shutdown();
    
    cv::destroyAllWindows();

    
    /*for(uint16_t i=0; i<mprVector.size(); ++i)
      mprFile<<mprVector[i].first<<" "<<mprVector[i].second<<" "<<mprVector[i].third<<" "<<mprVector[i].fourth<<"\n";
    
    mprFile.close();*/

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
