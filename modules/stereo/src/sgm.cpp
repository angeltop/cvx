#include <cvx/stereo/sgm.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <ctime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


extern void grayscaleGaussianBlur(cv::Mat &source, cv::Mat &destination, int sizeX, int sizeY = -1);

#define BLUR_RADIUS 3
#define MAX_PATHS_PER_SCAN 8
#define MAX_SHORT std::numeric_limits<unsigned short>::max()

namespace cvx { namespace stereo {


namespace impl {

struct path {
    short rowDiff;
    short colDiff;
    short index;
};


unsigned short calculatePixelCostOneWayBT(int row, int leftCol, int rightCol, const cv::Mat &leftImage, const cv::Mat &rightImage) {

    char leftValue, rightValue, beforeRightValue, afterRightValue, rightValueMinus, rightValuePlus, rightValueMin, rightValueMax;

    if(leftCol<0)
        leftValue = 0;
    else
        leftValue = leftImage.at<uchar>(row, leftCol);

    if(rightCol<0)
        rightValue = 0;
    else
        rightValue = rightImage.at<uchar>(row, rightCol);

    if (rightCol > 0) {
        beforeRightValue = rightImage.at<uchar>(row, rightCol - 1);
    } else {
        beforeRightValue = rightValue;
    }

    //std::cout << rightCol <<" " <<leftCol<< std::endl;
    if (rightCol + 1 < rightImage.cols && rightCol>0) {
        afterRightValue = rightImage.at<uchar>(row, rightCol + 1);
    } else {
        afterRightValue = rightValue;
    }

    rightValueMinus = round((rightValue + beforeRightValue) / 2.f);
    rightValuePlus = round((rightValue + afterRightValue) / 2.f);

    rightValueMin = std::min(rightValue, std::min(rightValueMinus, rightValuePlus));
    rightValueMax = std::max(rightValue, std::max(rightValueMinus, rightValuePlus));

    return std::max(0, std::max((leftValue - rightValueMax), (rightValueMin - leftValue)));
}

static int64_t *census(const cv::Mat& src, int kw, int kh) {
    int64_t *dst = new int64_t [src.rows * src.cols] ;
    memset(dst, 0, src.rows * src.cols * sizeof(uint64_t));

    cv::Mat_<uchar> src_(src) ;

    int kw2 = kw/2, kh2 = kh/2 ;
#pragma omp parallel for
    for ( int y = kh2; y < src.rows - kh2; y++ )  {
        for (int x = kw2; x < src.cols - kw2; x++ ) {
            uint64_t c = 0;
            for ( int dy = -kh2; dy <= kh2; dy++ ) {
                for ( int dx = -kw2; dx <= kw2; dx++ ) {
                    c <<= 1;
                    if ( dx != 0 && dy != 0 ) {
                        c += (src_[y][x]
                                <= src_[y + dy][x + dx]) ? 0 : 1;
                    }
                }
            }
            *(dst + y * src.cols + x) = c ;
       }
    }

    return dst ;
}

inline unsigned short calculatePixelCostBT(int row, int leftCol, int rightCol, const cv::Mat &leftImage, const cv::Mat &rightImage) {
    return std::min(calculatePixelCostOneWayBT(row, leftCol, rightCol, leftImage, rightImage),
        calculatePixelCostOneWayBT(row, rightCol, leftCol, rightImage, leftImage));
}
/*
void calculatePixelCost(const cv::Mat &firstImage, const cv::Mat &secondImage, int disparityRange, cv::Mat &C) {
    for (int row = 0; row < firstImage.rows; ++row) {
        for (int col = 0; col < firstImage.cols; ++col) {
            for (int d = 0; d < disparityRange; ++d) {
                 C.at<ushort>(row, col, d) = calculatePixelCostBT(row, col, col - d, firstImage, secondImage);
            }
        }
    }
}
*/

//if your compiler supports 64-bit integers
int hamming(uint64_t x, uint64_t y)
{
    return __builtin_popcountll(x ^ y);
}

void calculatePixelCost(const cv::Mat &firstImage, const cv::Mat &secondImage, int disparityRange, cv::Mat &C) {
    int64_t *cl = census(firstImage, 7, 3) ;
    int64_t *cr = census(secondImage, 7, 3) ;

    for (int row = 0; row < firstImage.rows; ++row) {
        for (int col = 0; col < firstImage.cols; ++col) {
            for (int d = 0; d < disparityRange; ++d) {
                C.at<ushort>(row, col, d) = ( col - d >= 0 ) ? hamming(*(cl + row * firstImage.cols + col), *(cr + row * firstImage.cols + col - d)) : 10000;
                 //C.at<ushort>(row, col, d) = calculatePixelCostBT(row, col, col - d, firstImage, secondImage);
            }
        }
    }

    delete [] cl ;
    delete [] cr ;
}

// pathCount can be 1, 2, 4, or 8
void initializeFirstScanPaths(std::vector<path> &paths, size_t pathCount) {
    for (unsigned short i = 0; i < pathCount; ++i) {
        paths.push_back(path());
    }

    if(paths.size() >= 1) {
        paths[0].rowDiff = 0;
        paths[0].colDiff = -1;
        paths[0].index = 1;
    }

    if(paths.size() >= 2) {
        paths[1].rowDiff = -1;
        paths[1].colDiff = 0;
        paths[1].index = 2;
    }

    if(paths.size() >= 4) {
        paths[2].rowDiff = -1;
        paths[2].colDiff = 1;
        paths[2].index = 4;

        paths[3].rowDiff = -1;
        paths[3].colDiff = -1;
        paths[3].index = 7;
    }

    if(paths.size() >= 8) {
        paths[4].rowDiff = -2;
        paths[4].colDiff = 1;
        paths[4].index = 8;

        paths[5].rowDiff = -2;
        paths[5].colDiff = -1;
        paths[5].index = 9;

        paths[6].rowDiff = -1;
        paths[6].colDiff = -2;
        paths[6].index = 13;

        paths[7].rowDiff = -1;
        paths[7].colDiff = 2;
        paths[7].index = 15;
    }
}

// pathCount can be 1, 2, 4, or 8
void initializeSecondScanPaths(std::vector<path> &paths, unsigned short pathCount) {
    for (unsigned short i = 0; i < pathCount; ++i) {
        paths.push_back(path());
    }

    if(paths.size() >= 1) {
        paths[0].rowDiff = 0;
        paths[0].colDiff = 1;
        paths[0].index = 0;
    }

    if(paths.size() >= 2) {
        paths[1].rowDiff = 1;
        paths[1].colDiff = 0;
        paths[1].index = 3;
    }

    if(paths.size() >= 4) {
        paths[2].rowDiff = 1;
        paths[2].colDiff = 1;
        paths[2].index = 5;

        paths[3].rowDiff = 1;
        paths[3].colDiff = -1;
        paths[3].index = 6;
    }

    if(paths.size() >= 8) {
        paths[4].rowDiff = 2;
        paths[4].colDiff = 1;
        paths[4].index = 10;

        paths[5].rowDiff = 2;
        paths[5].colDiff = -1;
        paths[5].index = 11;

        paths[6].rowDiff = 1;
        paths[6].colDiff = -2;
        paths[6].index = 12;

        paths[7].rowDiff = 1;
        paths[7].colDiff = 2;
        paths[7].index = 14;
    }
}

unsigned short aggregateCost(int row, int col, int d, path &p, int rows, int cols, int disparityRange,  cv::Mat &C, cv::Mat &A,
                             ushort small_penalty, ushort large_penalty ) {
    unsigned short aggregatedCost = 0;

    aggregatedCost += C.at<ushort>(row, col, d); // C(p, d)


    if (row + p.rowDiff < 0 || row + p.rowDiff >= rows || col + p.colDiff < 0 || col + p.colDiff >= cols) {
        // border
        ushort &a = A.at<ushort>(row, col, d) ;
        a += aggregatedCost;
        return a;
    }

    unsigned short minPrev, minPrevOther, prev, prevPlus, prevMinus;
    prev = minPrev = minPrevOther = prevPlus = prevMinus = MAX_SHORT;

    for (int disp = 0; disp < disparityRange; ++disp) {
        unsigned short tmp = A.at<ushort>(row + p.rowDiff, col + p.colDiff, disp); // L(p-r, i)
        if(minPrev > tmp) {
            minPrev = tmp;
        }                           // min_k L(p-r, k)

        if(disp == d) {
            prev = tmp;         // L(p-r, d)
        } else if(disp == d + 1) {
            prevPlus = tmp;     // L(p-r, d+1)
        } else if (disp == d - 1) {
            prevMinus = tmp;    // L(p-r, d-1)
        } else {
            if(minPrevOther > tmp) {
                minPrevOther = tmp; // min_i L(p-r, i)
            }
        }
    }

    //                     min(             L(p-r, d+1) + P1,                L(p-r, d-1 + P1)                     L(p-r, d),  min_i L(p-r, i) + P2
    aggregatedCost += std::min(std::min((int)prevPlus + small_penalty, (int)prevMinus + small_penalty), std::min((int)prev, (int)minPrevOther + large_penalty));
    aggregatedCost -= minPrev; // - min_k L(p-r, k)

    ushort &a = A.at<ushort>(row, col, d) ;
    a += aggregatedCost;

    return a;
}

float printProgress(unsigned int current, unsigned int max, int lastProgressPrinted) {
    int progress = floor(100 * current / (float) max);
    if(progress >= lastProgressPrinted + 5) {
        lastProgressPrinted = lastProgressPrinted + 5;
        std::cout << lastProgressPrinted << "%" << std::endl;
    }
    return lastProgressPrinted;
}

void aggregateCosts(int rows, int cols, int disparityRange,cv::Mat &C, cv::Mat A[], cv::Mat &S, size_t paths_per_scan,
                    ushort short_penalty, ushort large_penalty) {
    std::vector<path> firstScanPaths;
    std::vector<path> secondScanPaths;

    initializeFirstScanPaths(firstScanPaths, paths_per_scan);
    initializeSecondScanPaths(secondScanPaths, paths_per_scan);

    int lastProgressPrinted = 0;
    std::cout << "First scan..." << std::endl;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            for (unsigned int path = 0; path < firstScanPaths.size(); ++path) {
                for (int d = 0; d < disparityRange; ++d) {
                    S.at<ushort>(row, col, d) += aggregateCost(row, col, d, firstScanPaths[path], rows, cols, disparityRange, C, A[path], short_penalty, large_penalty);
                }
            }
        }
        lastProgressPrinted = printProgress(row, rows - 1, lastProgressPrinted);
    }

    lastProgressPrinted = 0;
    std::cout << "Second scan..." << std::endl;
    for (int row = rows - 1; row >= 0; --row) {
        for (int col = cols - 1; col >= 0; --col) {
            for (unsigned int path = 0; path < secondScanPaths.size(); ++path) {
                for (int d = 0; d < disparityRange; ++d) {
                    S.at<ushort>(row, col, d) += aggregateCost(row, col, d, secondScanPaths[path], rows, cols, disparityRange, C, A[path], short_penalty, large_penalty);
                }
            }
        }
        lastProgressPrinted = printProgress(rows - 1 - row, rows - 1, lastProgressPrinted);
    }
}

void optimizeDisparity(cv::Mat &S, int rows, int cols, int disparityRange, cv::Mat &disparityMap) {
    unsigned int disparity = 0, minCost;
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            minCost = MAX_SHORT;
            for (int d = disparityRange - 1; d >= 0; --d) {
                ushort sv = S.at<ushort>(row, col, d) ;
                if(minCost > sv) {
                    minCost = sv;
                    disparity = d;
                }
            }
            disparityMap.at<uchar>(row, col) = disparity;
        }
    }
}



} // namespace impl

cv::Mat SGMStereoMatcher::computeDisparity(const cv::Mat &left, const cv::Mat &right, size_t disp_range) {

    using namespace impl ;

    int rows = left.rows, cols = left.cols ;
    const int sz[] = {rows, cols, (int)disp_range} ;

    cv::Mat C(3, sz, CV_16U) ;
    cv::Mat S(3, sz, CV_16U, cv::Scalar(0)) ;

    cv::Mat A[MAX_PATHS_PER_SCAN] ;

    for(int path = 0; path < params_.num_paths_; ++path) {
        A[path] = cv::Mat(3, sz, CV_16U, cv::Scalar(0)) ;
    }

//    std::cout << "Smoothing images..." << std::endl;
//    grayscaleGaussianBlur(firstImage, firstImage, BLUR_RADIUS);
//    grayscaleGaussianBlur(secondImage, secondImage, BLUR_RADIUS);

//    std::cout << "Calculating pixel cost for the image..." << std::endl;
    calculatePixelCost(left, right, (int)disp_range, C) ;
 //   if(DEBUG) {
  //      printArray(C, firstImage.rows, firstImage.cols, disparityRange);
 //   }
  //  std::cout << "Aggregating costs..." << std::endl;
    aggregateCosts(rows, cols, disp_range, C, A, S, params_.num_paths_, params_.small_threshold_, params_.large_threshold_);

    cv::Mat disparity_map = cv::Mat(rows, cols, CV_8UC1, cv::Scalar::all(0));

    optimizeDisparity(S, rows, cols, disp_range, disparity_map);

    return disparity_map ;

}


}}


void saveDisparityMap(cv::Mat &disparityMap, int disparityRange, char* outputFile) {
    double factor = 256.0 / disparityRange;
    for (int row = 0; row < disparityMap.rows; ++row) {
        for (int col = 0; col < disparityMap.cols; ++col) {
            disparityMap.at<uchar>(row, col) *= factor;
        }
    }
    cv::imwrite(outputFile, disparityMap);
}

int main(int argc, char** argv) {

    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <left image> <right image> <output image file> <disparity range>" << std::endl;
        return -1;
    }

    char *firstFileName = argv[1];
    char *secondFileName = argv[2];
    char *outFileName = argv[3];

    cv::Mat firstImage;
    cv::Mat secondImage;
    firstImage = cv::imread(firstFileName, CV_LOAD_IMAGE_GRAYSCALE);
    secondImage = cv::imread(secondFileName, CV_LOAD_IMAGE_GRAYSCALE);

    if(!firstImage.data || !secondImage.data) {
        std::cerr <<  "Could not open or find one of the images!" << std::endl;
        return -1;
    }

    unsigned int disparityRange = atoi(argv[4]);


    std::cout << "Smoothing images..." << std::endl;
    grayscaleGaussianBlur(firstImage, firstImage, BLUR_RADIUS);
    grayscaleGaussianBlur(secondImage, secondImage, BLUR_RADIUS);

    using namespace cvx::stereo ;

    SGMStereoMatcher matcher ;
    cv::Mat disp = matcher.computeDisparity(firstImage, secondImage, disparityRange) ;

    saveDisparityMap(disp, disparityRange, outFileName);

    return 0;
}
