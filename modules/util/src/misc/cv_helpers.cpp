#include <cvx/util/misc/cv_helpers.hpp>
#include <stdarg.h>

using namespace std ;

namespace cvx { namespace util {

void imwritef(const cv::Mat &im, const char *format, ...)
{
    va_list vl;
    va_start(vl, format);
    int nc = vsnprintf(0, 0, format, vl) ;
    va_end(vl) ;

    va_start(vl, format);
    char *buffer = new char [nc+1] ;
    vsnprintf(buffer, nc+1, format, vl) ;
    va_end(vl) ;

    cv::imwrite(buffer, im) ;

    delete [] buffer ;
}

#ifndef SGN
#define SGN(a)          (((a)<0) ? -1 : 1)
#endif

void getScanLine(const cv::Point &p1, const cv::Point &p2, vector<cv::Point> &pts)
{
    int d, x, y, ax, ay, sx, sy, dx, dy;


    int x1 = p1.x ; int y1 = p1.y ;
    int x2 = p2.x ; int y2 = p2.y ;

    dx = x2-x1;  ax = abs(dx)<<1;  sx = SGN(dx);
    dy = y2-y1;  ay = abs(dy)<<1;  sy = SGN(dy);

    x = x1;
    y = y1;

    if (ax>ay)
    {                /* x dominant */
        d = ay-(ax>>1);
        for (;;)
        {
            pts.push_back(cv::Point(x, y)) ;

            if (x==x2) break;
            if (d>=0)
            {
                y += sy;
                d -= ax;
            }
            x += sx;
            d += ay;
        }
    }
    else
    {                      /* y dominant */
        d = ax-(ay>>1);
        for (;;)
        {
            pts.push_back(cv::Point(x, y)) ;

            if (y==y2) break;
            if (d>=0)
            {
                x += sx;
                d -= ay;
            }
            y += sy;
            d += ax;
        }
    }

}

} // namespace util
}
