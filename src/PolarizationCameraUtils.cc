#include "PolarizationCameraUtils.h"

#include <thread>
#include <vector>
#include <vector>
#include <algorithm>

// namespace ORB_SLAM3 {

std::array<cv::Mat,4> PolarizationCameraUtils::demosaicPolarizationImageParallel(const cv::Mat& input_img) {

    assert(input_img.type() == CV_8UC1);

    const int inner_rows = 1024;
    const int inner_cols = 1224;

    cv::Mat out1(inner_rows, inner_cols, CV_8UC1);
    cv::Mat out2(inner_rows, inner_cols, CV_8UC1);
    cv::Mat out3(inner_rows, inner_cols, CV_8UC1);
    cv::Mat out4(inner_rows, inner_cols, CV_8UC1);

    const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    auto worker = [&](int start_row, int end_row) {
        for (int i = start_row; i < end_row; ++i) {
            int y = 2 * i;
            const uchar* row0 = input_img.ptr<uchar>(y);
            const uchar* row1 = input_img.ptr<uchar>(y + 1);

            uchar* o1 = out1.ptr<uchar>(i);
            uchar* o2 = out2.ptr<uchar>(i);
            uchar* o3 = out3.ptr<uchar>(i);
            uchar* o4 = out4.ptr<uchar>(i);

            for (int j = 0; j < inner_cols; ++j) {
                int x = 2 * j;
                o1[j] = row0[x]; //90
                o2[j] = row0[x + 1]; //45
                o3[j] = row1[x + 1]; //0
                o4[j] = row1[x]; //135
            }
        }
    };

    // Divide rows among threads
    int rows_per_thread = inner_rows / num_threads;
    int remainder = inner_rows % num_threads;

    int row = 0;
    for (unsigned int t = 0; t < num_threads; ++t) {
        int start = row;
        int end   = row + rows_per_thread + (t < remainder ? 1 : 0);
        row = end;

        threads.emplace_back(worker, start, end);
    }

    for (auto &th : threads)
        th.join();

    // Resize results to double size
    cv::Mat r1, r2, r3, r4;
    cv::resize(out1, r1, cv::Size(inner_cols*2, inner_rows*2), 0, 0, cv::INTER_LINEAR);
    cv::resize(out2, r2, cv::Size(inner_cols*2, inner_rows*2), 0, 0, cv::INTER_LINEAR);
    cv::resize(out3, r3, cv::Size(inner_cols*2, inner_rows*2), 0, 0, cv::INTER_LINEAR);
    cv::resize(out4, r4, cv::Size(inner_cols*2, inner_rows*2), 0, 0, cv::INTER_LINEAR);

    return { r1, r2, r3, r4 };
}

cv::Mat PolarizationCameraUtils::computePolarizationAngleImageParallel(const cv::Mat& I0,
                                      const cv::Mat& I45,
                                      const cv::Mat& I90,
                                      const cv::Mat& I135,
                                      double pol_angle)
{

    // Convert input to float [0,1]
    // cv::Mat I0f, I45f, I90f, I135f;
    // I_0.convertTo(I0f, CV_64F, 1.0/255.0);
    // I_45.convertTo(I45f, CV_64F, 1.0/255.0);
    // I_90.convertTo(I90f, CV_64F, 1.0/255.0);
    // I_135.convertTo(I135f, CV_64F, 1.0/255.0);

    // Prepare HSV output
    cv::Mat Itheta(I0.rows, I0.cols, CV_8UC1);
    double I;

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

            uchar* Itheta_row = Itheta.ptr<uchar>(i);

            for (int j = 0; j < I0.cols; ++j) {
                double S0 = (I0_ptr[j] + I45_ptr[j] + I90_ptr[j] + I135_ptr[j]) / 2.0;
                double S1 = I0_ptr[j] - I90_ptr[j];
                double S2 = I45_ptr[j] - I135_ptr[j];

                // AoLP
                // double aolp = std::atan2(S2, S1)/2.0;

                // if (aolp < 0) aolp += M_PI;

                // if (abs(aolp-M_PI)<0.025) aolp = M_PI-0.02;

                // I = 0.5*(S0+S1*cos(2*pol_angle)+S2*sin(2*pol_angle)*cos(aolp)); //wrong equation
                I = 0.5*(S0+S1*cos(2*pol_angle)+S2*sin(2*pol_angle));

                // I_pol_angle_row[j] = static_cast<uchar>(std::min(I*255.0, 255.0));
                Itheta_row[j] = static_cast<uchar>(std::min(I, 255.0));

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

    return Itheta;
}

std::vector<cv::Mat> PolarizationCameraUtils::computeItheta0Itheta1(const cv::Mat& I0,
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

            for (int j = 0; j < I0.cols; ++j) {
                double S0 = (I0_ptr[j] + I45_ptr[j] + I90_ptr[j] + I135_ptr[j]) / 2.0;
                double S1 = I0_ptr[j] - I90_ptr[j];
                double S2 = I45_ptr[j] - I135_ptr[j];

                // AoLP
                // double aolp = std::atan2(S2, S1)/2.0;

                // if (aolp < 0) aolp += M_PI;

                // if (abs(aolp-M_PI)<0.025) aolp = M_PI-0.02;

                // I = 0.5*(S0+S1*cos(2*pol_angle)+S2*sin(2*pol_angle)*cos(aolp)); //wrong equation
                Ipixeltheta0 = 0.5*(S0+S1*costheta0+S2*sintheta0);

                // I_pol_angle_row[j] = static_cast<uchar>(std::min(I*255.0, 255.0));
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

    std::vector<cv::Mat> Itheta = {Itheta0, Itheta1};

    return Itheta;
}

// }
