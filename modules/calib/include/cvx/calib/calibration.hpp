#ifndef __CALIBRATION_HPP__
#define __CALIBRATION_HPP__

#include <vector>
#include <string>
#include <cvx/calib/pattern.hpp>
#include <cvx/util/camera/camera.hpp>

namespace cvx { namespace camera {

class CameraCalibration {
public:
    CameraCalibration() = default ;

    enum Type { Intrinsics = 0x01, Extrinsics = 0x02 } ;

    // calibration data
    struct Data {
        cv::Size fsize_ ;
        std::vector<cv::Point3f> coords_ ;
        std::vector<std::vector<CalibrationPattern::Marker>> markers_ ;
        std::vector<std::string> image_paths_ ;

        void save(const std::string &path) const ;
        void load(const std::string &path) ;
    };

    // run the pattern detector on the list of input images and write results on the data field
    void detect(const std::vector<std::string> &images, const CalibrationPattern &pattern, Data &data) ;

    double run(const Data &calib_data, cvx::util::PinholeCamera &cam, std::vector<Eigen::Matrix4d> &extrinsics, int flags = 0) ;

    double refine(const Data &calib_data, cvx::util::PinholeCamera &cam, std::vector<Eigen::Matrix4d> &extrinsics, bool intrinsics = true );
} ;


}}

#endif
