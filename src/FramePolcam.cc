#include "Frame.h"
#include <cstdint>
#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>


namespace ORB_SLAM3
{
class FramePolcam : public Frame {

    public:
    //FramePolcam(string n) : Frame(n) {}
    FramePolcam(const cv::Mat &imGray, const cv::Mat &imPolarized, const double &timeStamp, ORBextractor* extractor, ORBextractor* extractorPolcam, ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF, const IMU::Calib &ImuCalib):
         // Frame(&imGray, &imPolarized, &timeStamp, extractor, extractorPolcam, voc, pCamera, &distCoef, &bf, &thDepth, pPrevF, &ImuCalib)
         Frame(imGray, imPolarized, timeStamp, extractor, extractorPolcam, voc, pCamera, distCoef, bf, thDepth, pPrevF, ImuCalib)
         {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        // mnScaleLevels = mpORBextractorLeft->GetLevels();
        // mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        // mfLogScaleFactor = log(mfScaleFactor);
        // mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        // mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        // mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        // mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
    #ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
    #endif

        thread threadMono(&Frame::ExtractORBPolcam,this,0,imGray,0,0);
        thread threadPolcam(&Frame::ExtractORBPolcam,this,1,imPolarized,0,0);
        threadMono.join();
        threadPolcam.join();
    #ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

        mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
    #endif


        //added by claydergc
        //std::vector<cv::KeyPoint> mvKeysPolcamNonOverlapped;
        // std::vector<cv::Mat> mDescriptorsPolcamNonOverlappedVec;

        // computeFeatureOverlap(mvKeys, mvKeysCam1Overlapped, mvKeysCam1, mDescriptorsCam1Overlapped, mDescriptorsPolcamNonOverlappedVec, 3.0);
        // //std::cout<<"mvKeysPolcamNonOverlapped: "<<mvKeysPolcamNonOverlapped.size()<<std::endl;



        // mvKeysCam0 = mvKeys;
        // mDescriptorsCam0 = mDescriptors;


        // //Join mvKeys and mvKeysPolcamNonOverlapped
        // std::vector<cv::KeyPoint> mvKeysJoined;
        // mvKeysJoined = mvKeys;
        // mvKeysJoined.insert(mvKeysJoined.end(), mvKeysCam1.begin(), mvKeysCam1.end());
        // mvKeys = mvKeysJoined;

        // //std::cout<<"mDescriptors: "<<mDescriptors.size()<<" "<<"mDescriptorsDiff: "<<mDescriptorsDiff.size()<<std::endl;

        // //Join mDescriptors and mDescriptorsNonOverlapped
        // cv::vconcat(mDescriptorsPolcamNonOverlappedVec, mDescriptorsCam1);
        // //std::cout<<"mDescriptorsPolcamNonOverlapped: "<<mDescriptorsPolcamNonOverlapped.size()<<std::endl;
        // cv::vconcat(mDescriptors, mDescriptorsCam1, mDescriptors);

        // //std::cout<<"mDescriptors: "<<mDescriptors.size()<<std::endl;
        // //added by claydergc


        // // N = mvKeys.size();
        // N_Cam0 = mvKeysCam0.size();
        // N_Cam1 = mvKeysCam1.size();
        // N = N_Cam0 + N_Cam1;



        // if(mvKeys.empty())
        //     return;

        // UndistortKeyPoints();
        // UndistortKeyPointsNormalcam(); //added by claydergc
        // UndistortKeyPointsPolcam();

        // //std::cout<<"mvKeysUnNormalcam: "<<mvKeysUnNormalcam.size()<<std::endl;
        // //std::cout<<"mvKeysUnPolcam: "<<mvKeysUnPolcam.size()<<std::endl;

        // // Set no stereo information
        // mvuRight = vector<float>(N,-1);
        // mvDepth = vector<float>(N,-1);
        // mnCloseMPs = 0;

        // mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

        // mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
        // mmMatchedInImage.clear();

        // mvbOutlier = vector<bool>(N,false);

        // // This is done only for the first Frame (or after a change in the calibration)
        // if(mbInitialComputations)
        // {
        //     ComputeImageBounds(imGray);

        //     mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        //     mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        //     fx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,0);
        //     fy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,1);
        //     cx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,2);
        //     cy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,2);
        //     invfx = 1.0f/fx;
        //     invfy = 1.0f/fy;

        //     mbInitialComputations=false;
        // }


        // mb = mbf/fx;

        // //Set no stereo fisheye information
        // Nleft = -1;
        // Nright = -1;
        // mvLeftToRightMatch = vector<int>(0);
        // mvRightToLeftMatch = vector<int>(0);
        // mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        // monoLeft = -1;
        // monoRight = -1;

        // // AssignFeaturesToGrid();
        // // AssignFeaturesToGridNormalcam();
        // // AssignFeaturesToGridPolcam();

        // if(pPrevF)
        // {
        //     if(pPrevF->HasVelocity())
        //     {
        //         SetVelocity(pPrevF->GetVelocity());
        //     }
        // }
        // else
        // {
        //     mVw.setZero();
        // }

        // mpMutexImu = new std::mutex();
    }

};
}
