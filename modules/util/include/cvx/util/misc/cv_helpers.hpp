#ifndef __CV_HELPERS_HPP__
#define __CV_HELPERS_HPP__

#include <opencv2/opencv.hpp>

// various opencv helpers

namespace cvx { namespace util {

// use fprintf format pattern to create filename then call cv::imwrite
void imwritef(const cv::Mat &im, const char *format, ...) ;

// return all pixels on line
void getScanLine(const cv::Point &p1, const cv::Point &p2, std::vector<cv::Point> &pts) ;

} /*namespace util */ }
#endif
