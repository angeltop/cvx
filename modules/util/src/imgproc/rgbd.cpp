#include <cvx/util/imgproc/rgbd.hpp>

using namespace std ;
using namespace Eigen ;

namespace cvx {

static void hsv2rgb(float h, cv::Vec3i &rgb)
{
    int i ;
    float f, p, q, t, r, g, b ;

    if ( h == 0.0 ) return ;

    // h = 360.0-h ;

    h /= 60.0 ;

    i = (int)h ;
    f = h - i ;
    p = 0  ;
    q = 1-f ;
    t = f ;

    switch (i)
    {
    case 0:
        r = 1 ;
        g = t ;
        b = p ;
        break ;
    case 1:
        r = q ;
        g = 1 ;
        b = p ;
        break ;
    case 2:
        r = p ;
        g = 1 ;
        b = t ;
        break ;
    case 3:
        r = p ;
        g = q ;
        b = 1 ;
        break ;
    case 4:
        r = t ;
        g = p ;
        b = 1 ;
        break ;
    case 5:
        r = 1 ;
        g = p ;
        b = q ;
        break ;
    }

    rgb = cv::Vec3i((int)(255.0*r), (int)(255.0*g), (int)(255.0*b)) ;
}


cv::Mat depthViz(const cv::Mat &depth, ushort minv_, ushort maxv_)
{
    const int nLutColors = 2 << 12 ;
    static cv::Vec3i hsvlut[nLutColors] ;
    static bool initLut = false ;

    assert(depth.type() == CV_16UC1) ;

    int w = depth.cols, h = depth.rows ;

    int nc = nLutColors ;

    if ( !initLut )
    {
        int c ;
        float h, hmax, hstep ;

        hmax = 180 ;
        hstep = hmax/nc ;

        for ( c=0, h=hstep ; c<nc ; c++, h += hstep) hsv2rgb(h, hsvlut[c]) ;
    }

    unsigned short minv, maxv ;
    int i, j ;

    minv = (minv_ == 0 ) ? 0xffff : minv_;
    maxv = (maxv_ == 0 ) ? 0 : maxv_ ;

    uchar *ppl = depth.data ;
    unsigned short *pp = (unsigned short *)ppl ;
    int lw = depth.step ;

    for ( i=0 ; i<h ; i++, ppl += lw )
        for ( j=0, pp = (unsigned short *)ppl ; j<w ; j++, pp++ )
        {
            if ( *pp == 0 ) continue ;
            maxv = std::max(*pp, maxv) ;
            minv = std::min(*pp, minv) ;
        }

    cv::Mat image(h, w, CV_8UC3) ;

    for( i=0 ; i<h ; i++ )
    {
        cv::Vec3b *dst = image.ptr<cv::Vec3b>(i) ;
        unsigned short *src = (unsigned short *)depth.ptr<ushort>(i) ;

        for( j=0 ; j<w ; j++ )
        {
            unsigned short val = *src++ ;

            if ( val == 0 )
            {
                *(cv::Vec3b *)dst = cv::Vec3b(0, 0, 0) ;
                dst ++ ;
                continue ;
            }
            else val = (nc-1)*float((val - minv)/float(maxv - minv)) ;

            const cv::Vec3i &clr = hsvlut[val] ;

            *(cv::Vec3b *)dst = cv::Vec3b(clr[0], clr[1], clr[2]) ;
            dst ++ ;

        }
    }

    return image ;
}

class PtSorter
{
public:
    PtSorter() {}

    bool operator () (const cv::Point &p1, const cv::Point &p2) {
        return p1.x * p1.x + p1.y * p1.y <  p2.x * p2.x + p2.y * p2.y ;
    }
};

bool sampleNearestNonZeroDepth(const cv::Mat &dim, int x, int y, ushort &z, int ws)
{
    assert ( dim.type() == CV_16UC1 ) ;

    static vector<cv::Point> dpts ;

    if ( dpts.empty() )
    {
        for(int i=-ws ; i<=ws ; i++ )
            for(int j=-ws ; j<=ws ; j++ )
                dpts.push_back(cv::Point(j, i))  ;

        PtSorter sorter ;
        std::sort(dpts.begin(), dpts.end(), sorter) ;
    }

    bool found = false ;

    for(int i=0 ; i<dpts.size() ; i++)
    {
        const cv::Point &p = dpts[i] ;

        int x_ = p.x + x ;
        int y_ = p.y + y ;

        if ( x_ < 0 || y_ < 0 || x_ >= dim.cols || y_ >= dim.rows ) continue ;
        if ( ( z = dim.at<ushort>(y_, x_) ) == 0 ) continue ;

        found = true ;
        break ;
    }

    return found ;

}

bool sampleBilinearDepth(const cv::Mat &dim, float x, float y, float &z, int ws)
{
    assert ( dim.type() == CV_16UC1 ) ;

    int ix = x, iy = y ;
    float hx = x - ix, hy = y - iy ;

    if ( ( ix + 1 < 0 || ix + 1 >= dim.cols ) ||
         ( iy + 1 < 0 || iy + 1 >= dim.rows ) ||
         ( ix < 0 ) || ( iy < 0 ) )
    {
        ushort uz ;
        bool res = sampleNearestNonZeroDepth(dim, ix, iy, uz, ws) ;
        z = uz ;
        return res ;
    }

    ushort z1 = dim.at<ushort>(iy, ix) ;
    ushort z2 = dim.at<ushort>(iy, ix+1) ;
    ushort z3 = dim.at<ushort>(iy+1, ix) ;
    ushort z4 = dim.at<ushort>(iy+1, ix+1) ;

    if ( z1 == 0 || z2 == 0 || z3 == 0 || z4 == 0 )
    {
        ushort uz ;
        bool res = sampleNearestNonZeroDepth(dim, ix, iy, uz) ;
        z = uz ;
        return res ;
    }
    else
    {
        float s1 = (1 - hx) * z1 + hx * z2 ;
        float s2 = (1 - hx) * z3 + hx * z4 ;

        z = ( 1 - hy ) * s1 + hy * s2 ;

        return true ;
    }
}


void depthToPointCloud(const cv::Mat &depth, const PinholeCamera &model_, EPointList3f &coords, uint sampling )
{

    float center_x = model_.cx();
    float center_y = model_.cy();

    const double unit_scaling = 0.001 ;

    if ( depth.type() == CV_16UC1 )
    {

        float constant_x = unit_scaling / model_.fx();
        float constant_y = unit_scaling / model_.fy();

        cv::Mat_<ushort> depth_(depth) ;

        for(int i=0 ; i<depth.rows ; i+=sampling)
            for(int j=0 ; j<depth.cols ; j+=sampling)
            {
                ushort val = depth_[i][j] ;

                if ( val == 0 ) continue ;

                coords.push_back(Vector3f((j - center_x) * val * constant_x,
                                          (i - center_y) * val * constant_y,
                                          val * unit_scaling )) ;
            }

    }
    else if ( depth.type() == CV_32FC1 )
    {

        float constant_x = 1.0 / model_.fx();
        float constant_y = 1.0 / model_.fy();

        cv::Mat_<float> depth_(depth) ;

        for(int i=0 ; i<depth.rows ; i+=sampling)
            for(int j=0 ; j<depth.cols ; j+=sampling)
            {
                float val = depth_[i][j] ;

                if ( std::isnan(val) ) continue ;

                coords.push_back(Vector3f((j - center_x) * val * constant_x,
                                          (i - center_y) * val * constant_y,
                                          val * unit_scaling )) ;
            }
    }

}


}
