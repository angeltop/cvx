#include <cvx/calib/pattern.hpp>

#include <cvx/util/misc/path.hpp>
#include <cvx/util/misc/strings.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>

#include <Eigen/Geometry>

#include <fstream>

using namespace std ;
using namespace Eigen ;
using namespace cvx::util ;

namespace cvx { namespace camera {


std::vector<CalibrationPattern::Marker> OCVCalibrationPattern::findPoints(const cv::Mat &im) const
{
    std::vector<CalibrationPattern::Marker> targets ;
    cv::Mat gray ;

    cv::cvtColor(im, gray, CV_BGR2GRAY) ;

    bool patternfound ;
    vector<cv::Point2f> pts ;

    if ( type_ == Chessboard )
    {
        patternfound = cv::findChessboardCorners(gray, board_size_, pts, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE );

        if ( patternfound )
        {
            // find minimum distance among neighboring pts

            double min_distance = DBL_MAX ;

            for( int i = 0 ; i < board_size_.height-1; i++ )
                for( int j = 0; j < board_size_.width-1; j++ )
                {
                    int idx = i * board_size_.width + j ;
                    int idx1 = (i+1)*board_size_.width + j ;
                    int idx2 = i * board_size_.width + j + 1 ;

                    min_distance = std::min(cv::norm(pts[idx] - pts[idx1]), min_distance) ;
                    min_distance = std::min(cv::norm(pts[idx] - pts[idx2]), min_distance) ;

                }


            // use it to determine the search radius of corner refinement

            float rad = min_distance/2 ;

            cv::cornerSubPix(gray, pts, cv::Size(rad, rad), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        }

    }
    else if ( type_ == CirclesGrid )
        patternfound = cv::findCirclesGrid(gray, board_size_, pts, cv::CALIB_CB_SYMMETRIC_GRID) ;
    else if ( type_ == AsymmetricCirclesGrid )
        patternfound = cv::findCirclesGrid(gray, board_size_, pts, cv::CALIB_CB_ASYMMETRIC_GRID) ;

    if ( !patternfound ) return targets ;

    // TODO: assymetric grid

    for( int i = 0, k ; i < board_size_.height; i++ )
        for( int j = 0; j < board_size_.width; j++, k++ )  {
           targets.emplace_back(pts[k], k);
        }

    /*
    if ( patternfound && (pts[0].y > pts[boardSize.width].y) )
    {
        // reverse direction

        vector<cv::Point2f> rpts_ ;

        for( int i=pts.size()-1 ; i>=0 ; i-- )
            rpts_.push_back(pts[i]) ;

        pts = rpts_ ;
    }
    */

    return targets ;
}


void OCVCalibrationPattern::draw(cv::Mat &img, const vector<Marker> &corners) const
{
    vector<cv::Point2f> corner_pts ;
    for( const Marker &p: corners)
        corner_pts.emplace_back(p.pt_) ;

    cv::drawChessboardCorners(img, board_size_, cv::Mat(corner_pts), !corners.empty());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

AprilTagDetector::AprilTagDetector(AprilTagFamily family, size_t decimation_levels): tf_(0), td_(0) {

    if ( family == AprilTag36h11 )
        tf_ = tag36h11_create();
    else if ( family == AprilTag25h9 )
        tf_ = tag25h9_create();
    else if ( family == AprilTag16h5 )
        tf_ = tag16h5_create();
    else {
        return ;
    }

    apriltag_detector *td ;
    td_ = td = apriltag_detector_create();
    apriltag_detector_add_family(td, (apriltag_family_t *)tf_);

    td->debug = 0 ;
    td->refine_pose = 1 ;
    td->nthreads = 4 ;
    td->quad_decimate = decimation_levels ;
}

void AprilTagDetector::detect(const cv::Mat &imc, vector<AprilTagDetector::Result> &results)
{
    assert( td_ ) ;

    cv::Mat gray ;

    if ( imc.type() == CV_8UC1 ) gray = imc ;
    else cv::cvtColor(imc, gray, CV_BGR2GRAY) ;

    image_u8_t *im = image_u8_create(imc.cols, imc.rows ) ;
    uint8_t *dst = im->buf ;

    for(int i=0 ; i<im->height ; i++, dst += im->stride)
        memcpy(dst, gray.ptr(i), im->width) ;

    zarray_t *detections = apriltag_detector_detect((apriltag_detector *)td_, im);

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        if ( det->family != tf_ ) continue ;

        Result res ;
        res.id = det->id ;
        for(int i=0 ; i<4 ; i++ )
        {
            res.pts[i].x = det->p[i][0] ;
            res.pts[i].y = det->p[i][1] ;
        }

        results.push_back(res) ;

        apriltag_detection_destroy(det);
    }

    zarray_destroy(detections);

    image_u8_destroy(im);

}

void AprilTagDetector::draw(cv::Mat &im, const std::vector<Result> &results)
{
    for(int i=0 ; i<results.size() ; i++)
    {
        const Result &res = results[i] ;
        cv::line(im, cv::Point(res.pts[0].x, res.pts[0].y), cv::Point(res.pts[1].x, res.pts[1].y), cv::Scalar(255, 0, 0)) ;
        cv::line(im, cv::Point(res.pts[1].x, res.pts[1].y), cv::Point(res.pts[2].x, res.pts[2].y), cv::Scalar(0, 255, 255)) ;
        cv::line(im, cv::Point(res.pts[2].x, res.pts[2].y), cv::Point(res.pts[3].x, res.pts[3].y), cv::Scalar(0, 0, 255)) ;
        cv::line(im, cv::Point(res.pts[3].x, res.pts[3].y), cv::Point(res.pts[0].x, res.pts[0].y), cv::Scalar(0, 255, 0)) ;

        int baseline=0;
        cv::Size textSize = cv::getTextSize(std::to_string(res.id), cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                                            0.5, 2, &baseline);

        cv::Point2f pc = 0.25*(res.pts[0] + res.pts[1] + res.pts[2] + res.pts[3]) ;
        pc -= cv::Point2f(textSize.width/2, -textSize.height/2) ;

        cv::putText(im, std::to_string(res.id), cv::Point(pc.x, pc.y),
                    cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 2, 8);
    }

}

AprilTagDetector::~AprilTagDetector()
{
    apriltag_detector_destroy((apriltag_detector *)td_) ;
}




void AprilTagGridPattern::makePattern36H11(const string &tagFolder, const string &outSvg, const cv::Size &boardSize, float tileSize, float tileOffset)
{

    float tx = boardSize.width * (tileSize + tileOffset) ;
    float ty = boardSize.height * (tileSize + tileOffset) ;

    float bs = tileSize/8 ;

    ofstream strm(outSvg.c_str()) ;

    strm << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"" ;
    strm << tx*100 << "cm\" height=\"" << ty*100 << "cm\" viewBox=\"0 0 " ;
    strm << tx << ' ' << ty << "\">\n<g fill=\"black\">" ;

    for(int i=0, k=0 ; i<boardSize.height ; i++)
        for(int j=0 ; j<boardSize.width ; j++, k++)
        {
            string p = tagFolder + "/tag36_11_" + format(k, 5, 'd', '0') + ".png" ;

            if ( !Path::exists(p) ) continue ;

            cv::Mat cc = cv::imread(p) ;

            float x0 = j * (tileSize + tileOffset) ;
            float y0 = i * (tileSize + tileOffset) ;

            strm << "<g>\n" ;
            for( int y=0 ; y<8 ; y++)
                for( int x=0 ; x<8 ; x++)
                {

                    if ( cc.at<cv::Vec3b>(y+1, x+1)[0] == 0 )
                    {
                        float xx = x0 + x * bs ;
                        float yy = y0 + y * bs ;

                        strm << "<rect x=\"" << xx << "\" y=\"" << yy << "\" width=\"" << bs << "\" height=\"" << bs << "\"/>\n" ;
                    }
                }
            strm << "</g>\n" ;
        }

    strm << "</g></svg>" ;

    strm.flush() ;
}




AprilTagGridPattern::AprilTagGridPattern(const cv::Size &boardSize, float tileSize, float tileBorder, AprilTagDetector &detector):
    board_size_(boardSize), tile_size_(tileSize), tile_border_(tileBorder), detector_(detector)
{
    const float z = 0.0 ;

    for( uint i=0 ; i<board_size_.height ; i++ )
        for( uint j=0 ; j<board_size_.width ; j++ ) {

            float x = j * (tile_size_ + tile_border_) ;
            float y = i * (tile_size_ + tile_border_) ;

            obj_.emplace_back(x, y, z) ;
            obj_.emplace_back(x + tile_size_, y, z) ;
            obj_.emplace_back(x + tile_size_, y + tile_size_, z) ;
            obj_.emplace_back(x, y + tile_size_, z) ;
        }
}

vector<CalibrationPattern::Marker> AprilTagGridPattern::findPoints(const cv::Mat &im) const
{
    vector<AprilTagDetector::Result> results ;
    vector<CalibrationPattern::Marker> pts ;

    detector_.detect(im, results) ;

    for(int k=0 ; k<results.size() ; k++)
    {
        const AprilTagDetector::Result &res = results[k] ;

        int row = res.id / board_size_.width ;
        int col = res.id % board_size_.width ;

        if ( row >= board_size_.height) continue ;

        int index = 4*(row * board_size_.width + col) ;

        pts.emplace_back(res.pts[0], index++) ;
        pts.emplace_back(res.pts[1], index++) ;
        pts.emplace_back(res.pts[2], index++) ;
        pts.emplace_back(res.pts[3], index++) ;

    }

    return pts ;
}

void AprilTagGridPattern::draw(cv::Mat &im, const std::vector<CalibrationPattern::Marker> &targets) const
{

    // find homography and draw grid

    vector<cv::Point2f> pp, pts ;

    for( const Marker &p: targets ) {
        pts.emplace_back(p.pt_) ;
        pp.emplace_back(cv::Point2f(obj_[p.id_].x, obj_[p.id_].y)) ;
    }

    cv::Mat H = cv::findHomography(pp, pts) ;

    for(int i=0 ; i<board_size_.height ; i++ )
        for(int j=0 ; j<board_size_.width ; j++ )
        {
            cv::Mat_<double> q(3, 1), p ;

            q(0, 0) = j * (tile_size_ + tile_border_) ;
            q(1, 0) = i * (tile_size_ + tile_border_) ;
            q(2, 0) = 1.0 ;

            p = H*q ;

            cv::circle(im, cv::Point(p(0, 0)/p(2, 0), p(1, 0)/p(2, 0)), 2, cv::Scalar(0, 255, 0)) ;

            q(0, 0) = j * (tile_size_ + tile_border_) + tile_size_ ;
            q(1, 0) = i * (tile_size_ + tile_border_) ;
            q(2, 0) = 1.0 ;

            p = H*q ;

            cv::circle(im, cv::Point(p(0, 0)/p(2, 0), p(1, 0)/p(2, 0)), 2, cv::Scalar(0, 255, 0)) ;

            q(0, 0) = j * (tile_size_ + tile_border_) + tile_size_ ;
            q(1, 0) = i * (tile_size_ + tile_border_) + tile_size_ ;
            q(2, 0) = 1.0 ;

            p = H*q ;

            cv::circle(im, cv::Point(p(0, 0)/p(2, 0), p(1, 0)/p(2, 0)), 2, cv::Scalar(0, 255, 0)) ;

            q(0, 0) = j * (tile_size_ + tile_border_) ;
            q(1, 0) = i * (tile_size_ + tile_border_) + tile_size_ ;
            q(2, 0) = 1.0 ;

            p = H*q ;

            cv::circle(im, cv::Point(p(0, 0)/p(2, 0), p(1, 0)/p(2, 0)), 2, cv::Scalar(0, 255, 0)) ;


        }

    // draw only detected tiles

    for(int k=0 ; k<pts.size() ; )
    {
        cv::Point p[4] ;

        p[0] = cv::Point(pts[k].x, pts[k].y) ; ++k ;
        p[1] = cv::Point(pts[k].x, pts[k].y) ; ++k ;
        p[2] = cv::Point(pts[k].x, pts[k].y) ; ++k ;
        p[3] = cv::Point(pts[k].x, pts[k].y) ; ++k ;

        cv::line(im, p[0], p[1], cv::Scalar(128, 255, 0), 2) ;
        cv::line(im, p[1], p[2], cv::Scalar(128, 255, 0), 2) ;
        cv::line(im, p[2], p[3], cv::Scalar(128, 255, 0), 2) ;
        cv::line(im, p[3], p[0], cv::Scalar(128, 255, 0), 2) ;

        cv::Point2f p0 = 0.25*(p[0] + p[1] + p[2] + p[3]) ;
        cv::Point2f p1 = 0.5*(p[0] + p[1]) ;
        cv::Point2f p2 = 0.5*(p[1] + p[2]) ;

        cv::line(im, cv::Point(p0.x, p0.y), cv::Point(p1.x, p1.y), cv::Scalar(0, 0, 255), 2) ;
        cv::line(im, cv::Point(p0.x, p0.y), cv::Point(p2.x, p2.y), cv::Scalar(255, 0, 0), 2) ;
    }
}

void CalibrationPattern::makePointList(const std::vector<CalibrationPattern::Marker> &markers, std::vector<cv::Point2f> &image_pts, std::vector<cv::Point3f> &object_pts)
{
    uint n_markers = markers.size() ;

    object_pts.resize(n_markers) ;
    image_pts.resize(n_markers) ;

     for( size_t j=0 ; j < n_markers ; j++ ) {
         const auto &m = markers[j] ;
         int idx = m.id_ ;
         object_pts[j] = obj_[idx] ;
         image_pts[j] = m.pt_ ;
     }
}

}}
