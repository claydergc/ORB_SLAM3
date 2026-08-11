#include "PolarizationCameraUtils.h"

#include <iostream>
#include <thread>
#include <vector>
#include <vector>
#include <algorithm>
#include <numeric>

// namespace ORB_SLAM3 {

std::array<double,2> PolarizationCameraUtils::demosaicPolImageAndComputeStats(const cv::Mat& polcam_img, double theta0, double theta1, cv::Mat& Itheta0, cv::Mat& Itheta1) {

    assert(input_img.type() == CV_8UC1);

    const int inner_rows = 1024;
    const int inner_cols = 1224;

    cv::Mat I0Mat(inner_rows, inner_cols, CV_8UC1);
    cv::Mat I45Mat(inner_rows, inner_cols, CV_8UC1);
    cv::Mat I90Mat(inner_rows, inner_cols, CV_8UC1);
    cv::Mat I135Mat(inner_rows, inner_cols, CV_8UC1);

    // Prepare HSV output
    Itheta0 = cv::Mat(inner_rows, inner_cols, CV_8UC1);
    Itheta1 = cv::Mat(inner_rows, inner_cols, CV_8UC1);
    // cv::Mat AoLP(polcam_img.rows, polcam_img.cols, CV_8UC1);
    // cv::Mat DoLP(polcam_img.rows, polcam_img.cols, CV_8UC1);
    double Ipixeltheta0;
    double Ipixeltheta1;

    double costheta0 = cos(2.0*theta0);
    double sintheta0 = sin(2.0*theta0);
    double costheta1 = cos(2.0*theta1);
    double sintheta1 = sin(2.0*theta1);

    // double sin_sum = 0.0, cos_sum = 0.0;
    uint32_t n = inner_cols*inner_rows;

    const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
    // std::cout<<num_threads<<" threads"<<std::endl;
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    std::vector<double> partial_sin(num_threads, 0.0);
    std::vector<double> partial_cos(num_threads, 0.0);

    auto worker = [&](int start_row, int end_row, int tid) {
        double local_sin = 0.0, local_cos = 0.0;
        for (int i = start_row; i < end_row; ++i) {
            int y = 2 * i;
            const uchar* row0 = polcam_img.ptr<uchar>(y);
            const uchar* row1 = polcam_img.ptr<uchar>(y + 1);

            uchar* I0_row = I0Mat.ptr<uchar>(i);
            uchar* I45_row = I45Mat.ptr<uchar>(i);
            uchar* I90_row = I90Mat.ptr<uchar>(i);
            uchar* I135_row = I135Mat.ptr<uchar>(i);

            uchar* Itheta0_row = Itheta0.ptr<uchar>(i);
            uchar* Itheta1_row = Itheta1.ptr<uchar>(i);

            for (int j = 0; j < inner_cols; ++j) {
                int x = 2 * j;
                uchar I90 = row0[x]; //90
                uchar I45 = row0[x + 1]; //45
                uchar I0 = row1[x + 1]; //0
                uchar I135 = row1[x]; //135

                I90_row[j] = I90; //90
                I45_row[j] = I45; //45
                I0_row[j] = I0; //0
                I135_row[j] = I135; //135

                double S0 = (I0 + I45 + I90 + I135) / 2.0;
                double S1 = I0 - I90;
                double S2 = I45 - I135;

                // AoLP
                double aolp = std::atan2(S2, S1)/2.0;
                if (aolp < 0) aolp += M_PI;
                // uchar AoLP_uint8 = static_cast<uchar>(aolp * 180.0/M_PI);
                // if (AoLP_uint8 == 180) AoLP_uint8 = 0;

                // AoLP_row[j] = AoLP_uint8;

                // DoLP
                double dolp = std::sqrt(S1*S1 + S2*S2);
                dolp = (S0 != 0.0) ? dolp/S0 : 0.0;
                if (std::isnan(dolp) || std::isinf(dolp)) dolp = 0.0;
                uchar DoLP_uint8 = static_cast<uchar>(std::min(dolp*255.0, 255.0));
                // DoLP_row[j] = DoLP_uint8;

                Ipixeltheta0 = 0.5*(S0+S1*costheta0+S2*sintheta0);
                Itheta0_row[j] = static_cast<uchar>(std::min(Ipixeltheta0, 255.0));
                Ipixeltheta1 = 0.5*(S0+S1*costheta1+S2*sintheta1);
                Itheta1_row[j] = static_cast<uchar>(std::min(Ipixeltheta1, 255.0));

                // double theta_rad = aolp;
                // double two_theta = 2.0 * theta_rad;
                // sin_sum += std::sin(two_theta);
                // cos_sum += std::cos(two_theta);
                double theta_rad = aolp;
                double two_theta = 2.0 * theta_rad;
                local_sin += std::sin(two_theta);
                local_cos += std::cos(two_theta);
            }
        }
        partial_sin[tid] = local_sin;
        partial_cos[tid] = local_cos;
    };

    // Divide rows among threads
    int rows_per_thread = inner_rows / num_threads;
    int remainder = inner_rows % num_threads;

    int row = 0;
    for (unsigned int t = 0; t < num_threads; ++t) {
        int start = row;
        int end   = row + rows_per_thread + (t < remainder ? 1 : 0);
        row = end;
        threads.emplace_back(worker, start, end, t);
    }

    for (auto &th : threads)
        th.join();

    double sin_sum = std::accumulate(partial_sin.begin(), partial_sin.end(), 0.0);
    double cos_sum = std::accumulate(partial_cos.begin(), partial_cos.end(), 0.0);

    double mean_phi = std::atan2(sin_sum, cos_sum);       // (-pi, pi]
    double mean_theta = 0.5 * (mean_phi * 180.0 / CV_PI);  // (-90, 90]
        // wrap into [0, 180)
    mean_theta = std::fmod(mean_theta, 180.0);
    if (mean_theta < 0) mean_theta += 180.0;

    double R = std::sqrt(sin_sum * sin_sum + cos_sum * cos_sum) / (double)(n);
    double circ_std = 0.5 * (std::sqrt(-2.0 * std::log(R)) * 180.0 / CV_PI);

    if( (int)(theta0*180.0/M_PI) == 0) { Itheta0 = I0Mat; std::cout<<"theta0=0"<<std::endl; }
    else if( (int)(theta0*180.0/M_PI) == 45) { Itheta0 = I45Mat; std::cout<<"theta0=45"<<std::endl; }
    else if( (int)(theta0*180.0/M_PI) == 90) { Itheta0 = I90Mat; std::cout<<"theta0=90"<<std::endl; }
    else if( (int)(theta0*180.0/M_PI) == 135) { Itheta0 = I135Mat; std::cout<<"theta0=135"<<std::endl; }

    if( (int)(theta1*180.0/M_PI) == 0) { Itheta1 = I0Mat; std::cout<<"theta1=0"<<std::endl; }
    else if( (int)(theta1*180.0/M_PI) == 45) { Itheta1 = I45Mat; std::cout<<"theta1=45"<<std::endl; }
    else if( (int)(theta1*180.0/M_PI) == 90) { Itheta1 = I90Mat; std::cout<<"theta1=90"<<std::endl; }
    else if( (int)(theta1*180.0/M_PI) == 135) { Itheta1 = I135Mat; std::cout<<"theta1=135"<<std::endl; }

    return {mean_theta, circ_std};
}

std::vector<cv::Mat> PolarizationCameraUtils::computeItheta0Itheta1AoLPDoLP(const cv::Mat& I0,
                                      const cv::Mat& I45,
                                      const cv::Mat& I90,
                                      const cv::Mat& I135,
                                      double theta0,
                                      double theta1)
{

    // Convert input to float [0,1]
    // cv::Mat I0f, I45f, I90f, I135f;
    // I_0.convertTo(I0f, CV_64F, 1.0/255.0);
    // I_45.convertTo(I45f, CV_64F, 1.0/255.0);
    // I_90.convertTo(I90f, CV_64F, 1.0/255.0);
    // I_135.convertTo(I135f, CV_64F, 1.0/255.0);

    // Prepare HSV output
    cv::Mat Itheta0(I0.rows, I0.cols, CV_8UC1);
    cv::Mat Itheta1(I0.rows, I0.cols, CV_8UC1);
    cv::Mat AoLP(I0.rows, I0.cols, CV_8UC1);
    cv::Mat DoLP(I0.rows, I0.cols, CV_8UC1);
    double Ipixeltheta0;
    double Ipixeltheta1;

    double costheta0 = cos(2.0*theta0);
    double sintheta0 = sin(2.0*theta0);
    double costheta1 = cos(2.0*theta1);
    double sintheta1 = sin(2.0*theta1);

    const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    auto worker = [&](int start_row, int end_row) {
        for (int i = start_row; i < end_row; ++i) {
            // const double* I0_ptr = I0f.ptr<double>(i);
            // const double* I45_ptr = I45f.ptr<double>(i);
            // const double* I90_ptr = I90f.ptr<double>(i);
            // const double* I135_ptr = I135f.ptr<double>(i);

            const uchar* I0_ptr = I0.ptr<uchar>(i);
            const uchar* I45_ptr = I45.ptr<uchar>(i);
            const uchar* I90_ptr = I90.ptr<uchar>(i);
            const uchar* I135_ptr = I135.ptr<uchar>(i);

            uchar* Itheta0_row = Itheta0.ptr<uchar>(i);
            uchar* Itheta1_row = Itheta1.ptr<uchar>(i);
            uchar* AoLP_row = AoLP.ptr<uchar>(i);
            uchar* DoLP_row = DoLP.ptr<uchar>(i);

            for (int j = 0; j < I0.cols; ++j) {
                double S0 = (I0_ptr[j] + I45_ptr[j] + I90_ptr[j] + I135_ptr[j]) / 2.0;
                double S1 = I0_ptr[j] - I90_ptr[j];
                double S2 = I45_ptr[j] - I135_ptr[j];

                // AoLP
                double aolp = std::atan2(S2, S1)/2.0;
                if (aolp < 0) aolp += M_PI;
                uchar AoLP_uint8 = static_cast<uchar>(aolp * 180.0/M_PI);
                if (AoLP_uint8 == 180) AoLP_uint8 = 0;

                AoLP_row[j] = AoLP_uint8;

                // DoLP
                double dolp = std::sqrt(S1*S1 + S2*S2);
                dolp = (S0 != 0.0) ? dolp/S0 : 0.0;
                if (std::isnan(dolp) || std::isinf(dolp)) dolp = 0.0;
                uchar DoLP_uint8 = static_cast<uchar>(std::min(dolp*255.0, 255.0));
                DoLP_row[j] = DoLP_uint8;

                // I = 0.5*(S0+S1*cos(2*pol_angle)+S2*sin(2*pol_angle)*cos(aolp)); //wrong equation
                Ipixeltheta0 = 0.5*(S0+S1*costheta0+S2*sintheta0);
                Itheta0_row[j] = static_cast<uchar>(std::min(Ipixeltheta0, 255.0));
                Ipixeltheta1 = 0.5*(S0+S1*costheta1+S2*sintheta1);
                Itheta1_row[j] = static_cast<uchar>(std::min(Ipixeltheta1, 255.0));

            }
        }
    };

    // Divide rows among threads
    unsigned int rows_per_thread = I0.rows / num_threads;
    unsigned int remainder = I0.rows % num_threads;
    unsigned int row = 0;

    for (unsigned int t = 0; t < num_threads; ++t) {
        unsigned int start = row;
        unsigned int end = row + rows_per_thread + (t < remainder ? 1 : 0);
        row = end;
        threads.emplace_back(worker, start, end);
    }

    for (auto& th : threads) th.join();

    // cv::resize(I_pol_angle, I_pol_angle, cv::Size(new_width, new_height));

    std::vector<cv::Mat> Itheta = {Itheta0, Itheta1, AoLP, DoLP};

    return Itheta;
}

cv::Mat PolarizationCameraUtils::makeBottomHalfMask(const cv::Size& size) {
    cv::Mat mask = cv::Mat::zeros(size, CV_8U);
    mask(cv::Rect(0, size.height / 2, size.width, size.height - size.height / 2)) = 255;
    return mask;
}

std::vector<double> PolarizationCameraUtils::computeAoLPCircularStats(const cv::Mat& aolp,
                                                                       const cv::Mat& mask) {
    cv::Mat aolp_64f;
    aolp.convertTo(aolp_64f, CV_64F);

    bool useMask = !mask.empty();
    if (useMask) {
        CV_Assert(mask.size() == aolp.size());
        CV_Assert(mask.type() == CV_8U);
    }

    double sin_sum = 0.0, cos_sum = 0.0, n = 0.0;

    for (int r = 0; r < aolp_64f.rows; ++r) {
        const double* row = aolp_64f.ptr<double>(r);
        const uchar* maskRow = useMask ? mask.ptr<uchar>(r) : nullptr;

        for (int c = 0; c < aolp_64f.cols; ++c) {
            if (useMask && maskRow[c] == 0) continue;

            double theta_rad = row[c] * CV_PI / 180.0;
            double two_theta = 2.0 * theta_rad;
            sin_sum += std::sin(two_theta);
            cos_sum += std::cos(two_theta);
            n += 1.0;
        }
    }

    double mean_phi = std::atan2(sin_sum, cos_sum);       // (-pi, pi]
    double mean_theta = 0.5 * (mean_phi * 180.0 / CV_PI);  // (-90, 90]

    // wrap into [0, 180)
    mean_theta = std::fmod(mean_theta, 180.0);
    if (mean_theta < 0) mean_theta += 180.0;

    double R = std::sqrt(sin_sum * sin_sum + cos_sum * cos_sum) / n;
    double circ_std = 0.5 * (std::sqrt(-2.0 * std::log(R)) * 180.0 / CV_PI);

    return {mean_theta, circ_std};
}
// }
