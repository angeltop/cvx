#include <certh_core/cvHelpers.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std ;
using namespace certh_core ;

int main(int argc, char *argv[])
{
    cv::Mat im = cv::imread("/tmp/test.png"), gray ;

    cv::cvtColor(im, gray, CV_RGB2GRAY) ;
    cv::threshold(gray, gray, 1, 255, cv::THRESH_BINARY) ;

    cv::Mat_<unsigned int> labels ;

    int nc = connectedComponents(gray, labels) ;

    map<unsigned int, cv::Vec3b> lclrs ;

    cv::Mat_<cv::Vec3b> clr(gray.rows, gray.cols) ;

    for(int i=0 ; i<gray.rows ; i++)
        for(int j=0 ; j<gray.cols ; j++)
        {
            unsigned int lbl = labels[i][j] ;
            if ( lbl == 0 ) clr[i][j] = cv::Vec3b(0, 0, 0) ;
            else
            {
                if ( lclrs.count(lbl) == 0 )
                {
                   lclrs[lbl] = clr[i][j] = cv::Vec3b(rand()%255, rand()%255, rand()%255) ;

                }
                else
                    clr[i][j] = lclrs[lbl] ;
            }

        }

    cv::imwrite("/tmp/labels.png", clr) ;


    cout << findLargestBlob(gray).area() << endl ;

    RegionIterator it(labels) ;

    while ( it )
    {
        cout << it.area() << ' ' << it.rect().tl() << ' ' << it.rect().br() << endl ;

        ++it ;
    }

}



