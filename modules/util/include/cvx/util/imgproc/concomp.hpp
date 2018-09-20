#ifndef __IMGPROC_CONCOMP_HPP__
#define __IMGPROC_CONCOMP_HPP__

#include <memory>
#include <opencv2/opencv.hpp>

namespace cvx { namespace util {

// find connected components in a binary image. The components are returned as a label image (unsigned int).
// nc can be 4 for 4-connected and 8 for 8-connected regions

unsigned int connectedComponents(const cv::Mat &src, cv::Mat &labelImage, int nc = 8) ;

// class for iterating over regions/blobs returned by connected components

class RegionIterator {

public:

    RegionIterator(const cv::Mat &src): src_(src) {
        reset(src) ;
    }

    RegionIterator(const RegionIterator &other): src_(other.src_), data_(other.data_), it_(other.it_) {}
    RegionIterator(): src_(cv::Mat()), data_(new ContainerType), it_(data_->end()) {}

    bool operator == (const RegionIterator &other) const { return it_ == other.it_ ; }
    bool operator != (const RegionIterator &other) const { return it_ != other.it_ ; }

    operator int () const { return it_ != data_->end() ; }
    RegionIterator & operator++()  { ++it_ ; return *this ; }
    RegionIterator operator++(int) { RegionIterator tmp(*this) ; ++it_; return tmp ; }

    // return area of the region
    unsigned int area() const { return it_->second.area_ ; }

    // return outer contour of the region
    std::vector<cv::Point> contour() const ;

    // return label of the region
    unsigned int label() const { return it_->second.label_; }

    // return bounding rectangle
    cv::Rect rect() const { return it_->second.rect_ ; }

    // return mask of the same size as the source image with region pixels set to 255
    cv::Mat mask() const ;

private:

    void reset(const cv::Mat &src) ;

    struct RegionData {
        unsigned int label_ ;
        cv::Rect rect_ ;
        unsigned int area_ ;
    };

    typedef std::map<unsigned int, RegionData> ContainerType ;

    std::shared_ptr<ContainerType> data_ ;
    ContainerType::const_iterator it_ ;
    cv::Mat src_ ;
};

// performs connected components analysis on the source binary image (CV_8UC1) and finds the largest blob. It returns
// an iterator to the extracted region list pointing to the found region.

RegionIterator findLargestBlob(const cv::Mat &src,  unsigned int minArea = 0) ;

}}

#endif
