/**
 * PolarizationCameraUtils.h
 *
 * Utility class for polarization camera image processing.
 * Provides methods for demosaicing raw polarized images, computing
 * Stokes parameters, degree/angle of linear polarization (DoLP/AoLP),
 * and generating intensity images from polarization data.
 */

#ifndef POLARIZATION_CAMERA_UTILS_H
#define POLARIZATION_CAMERA_UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <array>
#include <cassert>
#include <cmath>  // for std::isnan, std::isinf
#include <dirent.h>
#include <sys/stat.h>


namespace ORB_SLAM3
{

class PolarizationCameraUtils
{
public:
    /**
     * Polarization channel angles supported by the sensor mosaic.
     * Typical Sony IMX250MZR layout: 0°, 45°, 90°, 135°.
     */
    enum class PolAngle { DEG_0 = 0, DEG_45 = 45, DEG_90 = 90, DEG_135 = 135 };

    // -------------------------------------------------------------------------
    // Construction / destruction
    // -------------------------------------------------------------------------

    PolarizationCameraUtils() = default;
    ~PolarizationCameraUtils() = default;

    // Non-copyable, non-movable (stateless utility class — use static methods)
    PolarizationCameraUtils(const PolarizationCameraUtils&) = delete;
    PolarizationCameraUtils& operator=(const PolarizationCameraUtils&) = delete;

    // -------------------------------------------------------------------------
    // Demosaicing
    // -------------------------------------------------------------------------

    /**
     * Splits a raw polarization mosaic image into four polarization channels.
     * Assumes a 2×2 super-pixel layout (Sony IMX250MZR / similar).
     *
     * @param rawImage   Input raw mosaic image (CV_8U or CV_16U).
     * @param ch0        Output channel at 0°.
     * @param ch45       Output channel at 45°.
     * @param ch90       Output channel at 90°.
     * @param ch135      Output channel at 135°.
     */

    static std::array<cv::Mat,4> demosaicPolarizationImageParallel(const cv::Mat& input_img);

    /**
     * Returns a single polarization channel extracted from a raw mosaic.
     *
     * @param rawImage   Input raw mosaic image.
     * @param angle      The desired polarization angle channel.
     * @return           Extracted channel image (half resolution in each dimension).
     */

    static cv::Mat computePolarizationAngleImageParallel(const cv::Mat& I_0,
                                          const cv::Mat& I_45,
                                          const cv::Mat& I_90,
                                          const cv::Mat& I_135,
                                          double pol_angle,
                                          int new_height,
                                          int new_width);

}; // class PolarizationCameraUtils

} // namespace ORB_SLAM3

#endif // POLARIZATION_CAMERA_UTILS_H
