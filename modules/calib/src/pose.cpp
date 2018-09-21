#include <cvx/calib/pose.hpp>
#include <RPP/RPP.h>

using namespace std ;
using namespace Eigen ;
using namespace cvx::util ;

namespace cvx { namespace camera {

double computeReprojectionError(
        const vector<cv::Point3f>& objectPoints,
        const vector<cv::Point2f>& imagePoints,
        const cv::Mat rvec, const cv::Mat & tvec,
        const PinholeCamera &cam )
{
    vector<cv::Point2f> imagePoints2;

    cv::projectPoints(objectPoints, rvec, tvec, cam.getMatrix(), cam.getDistortion(), imagePoints2) ;
    double err = cv::norm(cv::Mat(imagePoints), cv::Mat(imagePoints2), CV_L2);
    err = (float)std::sqrt(err/objectPoints.size());

    return err ;
}

Eigen::Affine3d estimatePose(const vector<cv::Point2f> &imagePts, const vector<cv::Point3f> &objPts,
                                    const PinholeCamera &cam, const Affine3d &a, bool usePrevious, double &err)
{
    cv::Mat rvec, tvec ;

    Matrix3d r = a.rotation();
    Vector3d t = a.translation() ;

    cv::Mat_<double> rmat(3, 3), tmat(1, 3) ;

    for(int i=0 ; i <3 ; i++ )
    {
        for(int j=0 ; j<3 ; j++ )
            rmat(i, j) = r(i, j) ;

        tmat(0, i) = t(i) ;
    }

    tvec = tmat ;
    cv::Rodrigues(rmat, rvec) ;

    cv::solvePnPRansac(objPts, imagePts, cam.getMatrix(), cam.getDistortion(), rvec, tvec, usePrevious, 500) ;

    err = computeReprojectionError(objPts, imagePts, rvec, tvec, cam) ;

    cv::Mat rmat_ ;
    cv::Rodrigues(rvec, rmat_) ;

    rmat = rmat_ ;
    tmat = tvec ;

    r << rmat(0, 0), rmat(0, 1), rmat(0, 2), rmat(1, 0), rmat(1, 1), rmat(1, 2), rmat(2, 0), rmat(2, 1), rmat(2, 2) ;
    t << tmat(0, 0), tmat(0, 1), tmat(0, 2) ;

    Eigen::Affine3d tr = Eigen::Translation3d(t) * r ;

    return tr ;
}


Affine3d estimatePosePlanar(const vector<cv::Point2f> &imagePts, const vector<cv::Point3f> &objPts,
                            const PinholeCamera &cam, double &err)
{
    cv::Mat translation, rotation ;
    double obj_err ;
    int iterations = 100;

    int n = imagePts.size() ;

    cv::Mat model = cv::Mat::zeros(3, n, CV_64F); // 3D points, z is zero
    cv::Mat ipts = cv::Mat::ones(3, n, CV_64F); // 2D points, homogenous points

    for(int i=0 ; i<n ; i++)
    {
        model.at<double>(0, i) = objPts[i].x ;
        model.at<double>(1, i) = objPts[i].y ;

        ipts.at<double>(0, i) = (imagePts[i].x - cam.cx())/cam.fx();
        ipts.at<double>(1, i) = (imagePts[i].y - cam.cy())/cam.fy();
    }

    RPP::Rpp(model, ipts, rotation, translation, iterations, obj_err, err);

    cv::Mat_<double> rot_(rotation) ;

    Matrix3d r ;
    r << rot_(0, 0) , rot_(0, 1) , rot_(0, 2),
            rot_(1, 0) , rot_(1, 1) , rot_(1, 2),
            rot_(2, 0) , rot_(2, 1) , rot_(2, 2) ;

    Affine3d tr = Eigen::Translation3d(translation.at<double>(0, 0), translation.at<double>(1, 0), translation.at<double>(2, 0)) * r ;

    return tr ;

}


Eigen::Matrix4d rodriguesToAffine(const cv::Mat &rvec, const cv::Mat &tvec)
{
    cv::Mat rmat_ ;
    cv::Rodrigues(rvec, rmat_) ;

    cv::Mat_<double> rmat(3, 3), tmat(1, 3) ;

    rmat = rmat_ ;
    tmat = tvec ;

    Matrix3d r ;
    Vector3d t  ;

    r << rmat(0, 0), rmat(0, 1), rmat(0, 2), rmat(1, 0), rmat(1, 1), rmat(1, 2), rmat(2, 0), rmat(2, 1), rmat(2, 2) ;
    t << tmat(0, 0), tmat(0, 1), tmat(0, 2) ;

    Eigen::Affine3d tr = Eigen::Translation3d(t) * r ;

    return tr.matrix() ;
}

void affineToRodrigues(const Matrix4d &mat, cv::Mat &rvec, cv::Mat &tvec)
{
    Affine3d a(mat) ;
    Matrix3d r = a.rotation();
    Vector3d t = a.translation() ;

    cv::Mat_<double> rmat(3, 3), tmat(1, 3) ;

    for(int i=0 ; i <3 ; i++ )
    {
        for(int j=0 ; j<3 ; j++ )
            rmat(i, j) = r(i, j) ;

        tmat(0, i) = t(i) ;
    }

    tvec = tmat ;
    cv::Rodrigues(rmat, rvec) ;

}

}}
