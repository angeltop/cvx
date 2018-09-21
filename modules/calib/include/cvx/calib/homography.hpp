#ifndef __HOMOGRAPHY_ESTIMATOR_HPP__
#define __HOMOGRAPHY_ESTIMATOR_HPP__

#include <cvx/util/geometry/point_list.hpp>
#include <cvx/util/math/ransac.hpp>

namespace cvx { namespace camera {

class HomographyEstimator {
public:
    struct Parameters ;

    HomographyEstimator() = default ;
    HomographyEstimator(const Parameters &params): params_(params) {}

    // solve using direct linear transform ( 4 or more points )
    void solve(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, Eigen::Matrix3d &H) ;
    // solve with ransac, get inliers
    bool solveRansac(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, Eigen::Matrix3d &H, std::vector<size_t> &inliers) ;
    // refine solution using levenberg-marquart iterations
    bool solveLM(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, const std::vector<size_t> &inliers, Eigen::Matrix3d &H);

    // get reciprocal reprojection error residuals
    void residuals(const std::vector<cv::Point2f> &pts1,  const std::vector<cv::Point2f> &pts2, const Eigen::Matrix3d &H, std::vector<double> &res) ;

public:
    struct Parameters {
        cvx::util::RANSAC::Parameters ransac_params_ ;
        struct LMParams {
            uint max_iter_ = 100 ;
            float ftol_ = std::numeric_limits<float>::epsilon() ;
        };

        LMParams lm_params_ ;
    };


private:

    Parameters params_ ;
};



}}

#endif
