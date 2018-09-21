#include <iostream>
#include <ostream>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <cvx/calib/homography.hpp>
#include <cvx/util/math/rng.hpp>

using namespace std ;
using namespace cvx::util ;
using namespace cvx::camera ;
using namespace Eigen ;

void find_correspondences(const vector<cv::KeyPoint> &kpts1, const cv::Mat &desc1,
                          const vector<cv::KeyPoint> &kpts2, const cv::Mat &desc2,
                          float match_ratio_threshold,
                          std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2, vector<cv::DMatch> &imatches ) {
    cv::FlannBasedMatcher matcher ;
    matcher.add(desc2) ;
    matcher.train() ;

    // establish matches using knn search

    vector<vector<cv::DMatch> > cmatches ;

    matcher.knnMatch(desc1, cmatches, 2);

    // perform ratio test to prune unreliable matches

    for(int i=0 ; i<cmatches.size() ; i++) {
        if ( cmatches[i].size() > 1 ) {
            if ( cmatches[i][0].distance / cmatches[i][1].distance < match_ratio_threshold ) {
                  const cv::DMatch &match = cmatches[i][0] ;
                  int qidx = match.queryIdx ;
                  int tidx = match.trainIdx ;

                  pts1.emplace_back(kpts1[qidx].pt) ;
                  pts2.emplace_back(kpts2[tidx].pt) ;

                  imatches.emplace_back(match) ;
               }
           }
       }

}

void draw_matches(cv::Mat &res, const cv::Mat &im1, const vector<cv::Point2f> &pts1, const cv::Mat &im2, const vector<cv::Point2f> &pts2,
                  const vector<size_t> &inliers) {
    uint w = im1.cols + im2.cols, h = std::max(im1.rows, im2.rows) ;

    res = cv::Mat::zeros(h, w, CV_8UC3) ;

    im1.copyTo(res(cv::Rect(0, 0, im1.cols, im1.rows))) ;
    im2.copyTo(res(cv::Rect(im1.cols, 0, im2.cols, im2.rows)))  ;

    RNG rng ;
    for( uint i=0 ; i<inliers.size() ; i++ ) {
        size_t idx = inliers[i] ;
        cv::Point p1(pts1[idx].x, pts1[idx].y) ;
        cv::Point p2(pts2[idx].x + im1.cols, pts2[idx].y) ;

        cv::Scalar color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)) ;
        cv::line(res, p1, p2, color, 2) ;
    }

}

int main(int argc, char *argv[]) {

    cv::Mat im1 = cv::imread("/home/malasiot/Downloads/caps/IMG_20170525_083918.jpg") ;

    vector<cv::Point2f> pts1 ; /*= { { 1544, 592 }, { 2236, 512 }, { 1552, 2140}, {2144, 1608}, {2416, 1332} } ;*/

    cv::Mat im2 = cv::imread("/home/malasiot/Downloads/caps/IMG_20170525_083921.jpg") ;

    vector<cv::Point2f> pts2 ; /*= { { 292, 656 }, { 992, 596 }, { 360, 2332}, { 964, 1700 }, {1212, 1412} } ; */

    cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(200, 3, 0.01, 4, 1.6);

    vector<cv::KeyPoint> kpts1, kpts2 ;
    cv::Mat desc1, desc2 ;

    sift->detectAndCompute(im1, cv::Mat(), kpts1, desc1) ;
    sift->detectAndCompute(im2, cv::Mat(), kpts2, desc2) ;

    vector<cv::DMatch> imatches, hmatches ;
    find_correspondences(kpts1, desc1, kpts2, desc2, 0.8, pts1, pts2, imatches) ;
    cv::Mat res ;
    cv::drawMatches(im1, kpts1, im2, kpts2, imatches, res, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS) ;
    cv::imwrite("/tmp/matches.png", res) ;

    HomographyEstimator::Parameters hparams ;
    hparams.ransac_params_.max_num_trials_ = 1000 ;
    hparams.ransac_params_.max_error_ = 10.0 ;

    HomographyEstimator he ;

    Matrix3d H ;
    std::vector<size_t> inliers ;
    he.solveRansac(pts1, pts2, H, inliers) ;

    hmatches.resize(inliers.size()) ;
    std::transform(inliers.begin(), inliers.end(), hmatches.begin(), [&] ( size_t idx) { return imatches[idx] ; }) ;


    he.solveLM(pts1, pts2, inliers, H) ;

    cv::Mat hres ;
    draw_matches(hres, im1, pts1, im2, pts2, inliers) ;
    cv::imwrite("/tmp/hmatches.png", hres) ;


    cout << inliers.size() << endl ;


  //  cout << H << ' ' << he.residual(pts1, pts2, H) << endl ;

}
