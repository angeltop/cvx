#ifndef __CAMERA_POSE_HPP__
#define __CAMERA_POSE_HPP__

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Geometry>

#include <cvx/util/camera/camera.hpp>

namespace cvx { namespace camera {

// estimate camera pose using OpenCV PnPRansac

Eigen::Affine3d estimatePose(const std::vector<cv::Point2f> &imagePts, const std::vector<cv::Point3f> &objPts,
                                    const cvx::util::PinholeCamera &cam, const Eigen::Affine3d &a, bool usePrevious, double &err) ;

// estimate pose of planar target

Eigen::Affine3d estimatePosePlanar(const std::vector<cv::Point2f> &imagePts, const std::vector<cv::Point3f> &objPts,
                            const cvx::util::PinholeCamera &cam, double &err) ;


Eigen::Matrix4d rodriguesToAffine(const cv::Mat &rvec, const cv::Mat &tvec) ;

void affineToRodrigues(const Eigen::Matrix4d &mat, cv::Mat &rvec, cv::Mat &tvec) ;

}}

#endif
