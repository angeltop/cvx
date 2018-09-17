#ifndef __IMGPROC_GABOR_HPP__
#define __IMGPROC_GABOR_HPP__

#include <opencv2/opencv.hpp>
namespace cvx { namespace util {

// Gabor filter-bank creation
void makeGaborFilterBank(int nOctaves, int nAngles, double sigma, std::vector<cv::Mat> &kernels) ;

// filter image with filter-bank
void applyGaborFilterBank(const cv::Mat &src, std::vector<cv::Mat> &kernels, std::vector<cv::Mat> &responses) ;

}}

#endif
