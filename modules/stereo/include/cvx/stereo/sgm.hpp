#ifndef __CVX_STEREO_SGM_HPP__
#define __CVX_STEREO_SGM_HPP__

#include <opencv2/opencv.hpp>

namespace cvx { namespace stereo {

// Semi global matcher

class SGMStereoMatcher {
public:
    struct Parameters {
        ushort small_threshold_ = 3 ;  // P1
        ushort large_threshold_ = 20 ; // P2
        int num_paths_ = 8 ;
    } ;

    SGMStereoMatcher(const Parameters &params): params_(params) {}
    SGMStereoMatcher() = default ;

    cv::Mat computeDisparity(const cv::Mat &left, const cv::Mat &right, size_t disparity_range) ;


private:

    Parameters params_ ;

};










}}

























#endif
