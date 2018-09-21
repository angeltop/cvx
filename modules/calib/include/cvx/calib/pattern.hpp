#ifndef __CALIBRATION_PATTERN_HPP__
#define __CALIBRATION_PATTERN_HPP__

#include <vector>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Geometry>

namespace cvx { namespace camera {

// interface for defining custom calibration patterns

class CalibrationPattern
{
public:
    CalibrationPattern() {}

    struct Marker {
        Marker(const cv::Point2d &pt, int id): pt_(pt), id_(id) {}
        Marker(): id_(-1) {}

        cv::Point2f pt_ ;
        int id_ = -1 ; // The unique location on the calibration pattern (i.e. grid)
    };

    const std::vector<cv::Point3f> &coords() const { return obj_ ; }

    // Find calibration targets on image. For each marker found the image location and an index to the the corresponding 3D point array
    // is returned

    virtual std::vector<Marker> findPoints(const cv::Mat &im) const  = 0 ;

    virtual void draw(cv::Mat &im, const std::vector<Marker> &markers) const {}

    // copy markers to associated list of 2d-3d points suitable for opencv functions
    void makePointList(const std::vector<Marker> &markers, std::vector<cv::Point2f> &pts, std::vector<cv::Point3f> &objs) ;


protected:
    std::vector<cv::Point3f> obj_ ; // should be filled by overriding classes with 3D point coordinates of the callibration pattern
};

// wrapper for OpenCV calibration pattern algorithms

class OCVCalibrationPattern: public CalibrationPattern
{
public:

    enum Type { Chessboard, CirclesGrid, AsymmetricCirclesGrid } ;

    OCVCalibrationPattern(const cv::Size &boardSize, const cv::Size &tileSize, Type ptype = Chessboard):
        board_size_(boardSize), tile_size_(tileSize), type_(ptype) {
        for( int i = 0, k ; i < board_size_.height; i++ )
                for( int j = 0; j < board_size_.width; j++, k++ )
                    obj_.emplace_back(float(j*tile_size_.width), float(i*tile_size_.height), 0.0) ;
    }

    std::vector<Marker> findPoints(const cv::Mat &im) const override ;

    void draw(cv::Mat &img, const std::vector<Marker> &corners) const override ;

private:

    cv::Size board_size_, tile_size_ ;
    Type type_ ;
};

enum AprilTagFamily { AprilTag36h11, AprilTag25h9, AprilTag16h5 } ;

// detector of apriltag patterns based on http://april.eecs.umich.edu/wiki/index.php/AprilTags

class AprilTagDetector {
public:

    struct Result {
        uint64_t id ;        // tag id
        cv::Point2f pts[4] ; // detected corners of tag box (clockwise)
    };

    AprilTagDetector(AprilTagFamily family = AprilTag36h11, size_t decimation_levels = 1) ;

    void detect(const cv::Mat &imc, std::vector<Result> &results) ;

    void draw(cv::Mat &im, const std::vector<Result> &results) ;

    ~AprilTagDetector() ;

private:

    void *tf_, *td_ ;
};

// a grid pattern of apriltags

class AprilTagGridPattern: public CalibrationPattern
{
public:

    // boardSize: number of tiles in each dimension, tileSize: size of the tile in meteres, tileBorder: border between tiles in meters
    AprilTagGridPattern(const cv::Size &boardSize, float tileSize, float tileBorder, AprilTagDetector &detector);

    std::vector<Marker> findPoints(const cv::Mat &im) const override;

    // creates a pattern of given dimensions to be used as a calibration target
    // tagFolder: folder where tag pattern images are located (downloaded from website)
    // outSvg: filename of the resulting SVG image.

    static void makePattern36H11(const std::string &tagFolder, const std::string &outSvg,
                          const cv::Size &boardSize, float tileSize, float tileOffset) ;

    void draw(cv::Mat &im, const std::vector<Marker> &pts) const override;

protected:

    cv::Size board_size_ ;
    float tile_size_, tile_border_ ;
    AprilTagDetector &detector_ ;

};

}}

#endif
