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

// double computeWeight(double DoLP_mean, double DoLP_min = 0.26, double DoLP_max = 0.38) {
// double computeWeight(double DoLP_mean, double DoLP_min = 0.25, double DoLP_max = 0.385) {
// double computeWeight(double DoLP_mean, double DoLP_min = 0.22, double DoLP_max = 0.4) { //this works on 0835
double computeWeight(double DoLP_mean, double DoLP_min = 0.25, double DoLP_max = 0.42) {
// double computeWeight(double DoLP_mean, double DoLP_min = 0.27, double DoLP_max = 0.4) {
    double t = (DoLP_mean - DoLP_min) / (DoLP_max - DoLP_min);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t;
}

inline double clampVal(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// double f(double AoLP_mean, double AoLP_3stddev, double AoLP_max = M_PI / 2.0) {
//     double v = clampVal(std::abs(AoLP_mean) / AoLP_max, 0.0, 1.0);
//     double g = 1.0 - v;
//     return (1.0 - g) * AoLP_mean + g * AoLP_3stddev;
// }
//
double f(double AoLP_mean, double AoLP_3stddev, double p, double AoLP_max = M_PI / 2.0) {
// double f(double AoLP_mean, double AoLP_3stddev, double p, double AoLP_max = M_PI) {
    double v = clampVal(std::abs(AoLP_mean) / AoLP_max, 0.0, 1.0);

    std::cout << "AoLP_mean: " << AoLP_mean*180.0/M_PI << " v: " << v << std::endl;

    double v_p = std::pow(v, p);
    // double v_p = v*p;
    double g = 1.0 - v_p;
    // double g = 1.0 - v_p;
    // return (1.0 - g) * AoLP_mean + g * AoLP_3stddev;
    return v_p * AoLP_mean + g * AoLP_3stddev;
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
    // double theta0 = f(43.732*M_PI/180.0, 92*M_PI/180.0, 1.0);
    double theta0 = f(8*M_PI/180.0, 92*M_PI/180.0, 1.0);

    double d = (43.732*M_PI/180.0);

    std::cout << "theta0: " << theta0*180.0/M_PI << std::endl;

    return 0;
}
