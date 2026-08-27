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

#include "PolarizationCameraUtils.h"

#include <cmath>
#include<iostream>
#include <csignal> // For signal handling

#include<opencv2/core/core.hpp>

#include<System.h>
#include <opencv2/core/mat.hpp>
#include <string>

using namespace std;


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

    cv::Mat polcam_img;
    cv::Mat Itheta0, Itheta1;
    double aolp_mean_curr, aolp_std, dolp_mean;

    polcam_img = cv::imread("/home/ros-noetic/DATA/datasets/Polcam02/KelvinGrove/20260128/0835/polcam/1769553311606931767.tiff",cv::IMREAD_GRAYSCALE);

    std::array<double, 3> aolp_dolp_stats = PolarizationCameraUtils::demosaicPolImageAndComputeStats(polcam_img, 0*M_PI/180.0, 0*M_PI/180.0, Itheta0, Itheta1, false);

    aolp_mean_curr = aolp_dolp_stats[0];
    aolp_std = aolp_dolp_stats[1];
    dolp_mean = aolp_dolp_stats[2];

    std::cout<<"aolp_mean_curr: "<<aolp_mean_curr<<", aolp_std: "<<aolp_std<<", dolp_mean: "<<dolp_mean<<std::endl;

    return 0;
}
