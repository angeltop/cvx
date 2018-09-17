#include <certh_core/MFunctions.h>
#include <certh_core/Application.h>
#include <certh_core/Calibration.h>
#include <certh_core/Helpers.h>

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

using namespace std ;
using namespace certh_core ;

int main(int argc, char *argv[])
{


    cv::VideoCapture cap(0); // open the default camera
//    if(!cap.isOpened())  // check if we succeeded
//          return -1;

    AprilTagGridPattern::makePattern36H11("/home/malasiot/source/vl/3rdparty/apriltag/tag36h11/",
                                          "/tmp/pattern.svg", cv::Size(4, 4), 0.08, 0.02) ;

    cv::namedWindow("result",1);

    AprilTagDetector pat ;
    AprilTagGridPattern gpat(cv::Size(7, 10), 0.04, 0.01, pat) ;

    for(;;)
    {
          cv::Mat frame, gray;
          cap >> frame; // get a new frame from camera


          vector<cv::Point2f> pts ;
          vector<cv::Point3f> objs ;

          if ( gpat.findPoints(frame, pts, objs) )
              gpat.draw(frame, pts, objs) ;

          cv::imshow("result", frame);
          if( cv::waitKey(30) >= 0) break;
      }

      return 0;

}
