#include <cvx/util/imgproc/concomp.hpp>

#include <stack>
#include <set>

using namespace std ;

namespace cvx { namespace util {

const int NOLABEL = INT_MAX ;

struct ConRegionInfo
{
    ConRegionInfo(): comp_(NOLABEL) {}

    set<unsigned int> eq_set_ ;
    unsigned int comp_ ;
} ;

typedef vector<ConRegionInfo> EqTable ;

static void dfs(EqTable& G, long v, unsigned int count)
{
    stack<unsigned long> S;

    S.push(v);
    G[v].comp_ = count;

    while (!S.empty())
    {
        v = S.top() ;
        S.pop() ;

        set<unsigned int>::iterator it = G[v].eq_set_.begin() ;

        for( ; it != G[v].eq_set_.end() ; it ++ )
        {
            unsigned int w = *it ;
            if ( G[w].comp_ == NOLABEL )
            {
                G[w].comp_ = count;
                S.push(w);
            }
        }
    }
}

static int eqTableResolve(EqTable &G, unsigned int nreg)
{
    int i ;

    for( i=0 ; i<nreg ; i++ ) G[i].comp_ = NOLABEL ;

    unsigned int count = 0 ;

    for(i=0 ; i<nreg ; i++ )
    {
        if (G[i].comp_ == NOLABEL)
        {
            dfs(G, i, count) ;
            count++ ;
        }
    }

    return count ;
}



unsigned int connectedComponents(const cv::Mat &src, cv::Mat &labelImage, int nc)
{
    assert (nc == 4 || nc == 8) ;

    cv::Mat_<uchar> bmp(src) ;

    int w = src.cols, h = src.rows ;
    cv::Mat_<int> labels(h, w) ;

    unsigned long nlabels = 0 ;

    EqTable eq_table ;

    for(int i=0 ; i<h ; i++ )
        for(int j=0 ; j<w ; j++ )
            labels[i][j] = NOLABEL ;

    // Start merging neighbor pixels

    for( int i=0 ; i<h ; i++ )
        for( int j=0 ; j<w ; j++ )
        {
            if ( !bmp[i][j] ) continue ;

            int &label = labels[i][j] ;

            int nb[8] ;

#define OUT_OF_BOUNDS(x, y) ( y < 0 || y>h-1 || x<0 || x > w-1 )
#define NEIGHB(i, j) ( OUT_OF_BOUNDS(j, i) || labels[i][j] == NOLABEL || !bmp[i][j] ) ? NOLABEL : labels[i][j] ;

            nb[0] = NEIGHB(i-1, j-1) ;
            nb[1] = NEIGHB(i-1, j) ;
            nb[2] = NEIGHB(i-1, j+1) ;
            nb[3] = NEIGHB(i, j-1) ;

            if ( nc == 8 ) {
                nb[4] = NEIGHB(i, j+1) ;
                nb[5] = NEIGHB(i+1, j-1) ;
                nb[6] = NEIGHB(i+1, j) ;
                nb[7] = NEIGHB(i+1, j+1) ;
            }

            unsigned long min_label = w * h ;
            bool first = true ;
            int k ;

            for(  k=0 ; k<nc ; k++ )
            {
                unsigned long p = nb[k] ;

                if ( p == NOLABEL ) continue ;

                first = false ;

                min_label = std::min(min_label, p) ;
            }

            if ( first )
            {
                label = nlabels++ ;
                eq_table.push_back(ConRegionInfo()) ;
            }
            else
            {
                label = min_label ;

                for( k=0 ; k<nc ; k++ )
                {
                    unsigned int p = nb[k] ;

                    if ( p == NOLABEL || p == min_label ) continue ;

                    eq_table[min_label].eq_set_.insert(p) ;
                }
            }
        }

    int ncomp = eqTableResolve(eq_table, nlabels) ;

    for( int i=h-1 ; i>=0 ; i-- )
        for( int j=w-1 ; j>=0 ; j-- )
        {
            int &label = labels[i][j] ;

            if ( label == NOLABEL ) continue ;

            int v = eq_table[label].comp_ ;

            label = v ;
        }

    labelImage = labels ;
    return ncomp ;
}


RegionIterator findLargestBlob(const cv::Mat &src, unsigned int minArea)
{
    cv::Mat labels ;
    if ( connectedComponents(src, labels) )
    {
        unsigned int maxArea = minArea ;

        RegionIterator it(labels), best ;

        while ( it )
        {
            if ( it.area() > maxArea )
            {
                maxArea = it.area() ;
                best = it ;
            }

            ++it ;
        }

        return best ;
    }
    else
        return RegionIterator() ;
}

void RegionIterator::reset(const cv::Mat &src)
{
    data_.reset(new ContainerType ) ;

    int w = src.cols, h = src.rows ;

    // Gather region info

    unsigned long prev_label = NOLABEL ;
    RegionData *prev_reg = NULL, *reg ;

    for( int i=0 ; i<h ; i++ )
        for(int j=0 ; j<w ; j++ )
        {
            unsigned long label = src.at<unsigned int>(i, j) ;

            if ( label == NOLABEL ) continue ;

            // Create or retrieve region with specified label

            if ( label == prev_label ) reg = prev_reg ;
            else
            {
                reg = &((*data_)[label]) ;
                prev_label = label ;
                prev_reg = reg ;
            }

            // Update region info

            reg->label_ = label ;

            if ( reg->rect_.width == 0 )
            {
                reg->rect_ = cv::Rect(cv::Point(j, i), cv::Size(1, 1)) ;
                reg->area_ = 1 ;
            }
            else
            {
                cv::Point tl = reg->rect_.tl(), br = reg->rect_.br() ;

                tl.x = std::min(tl.x, j) ; tl.y = std::min(tl.y, i) ;
                br.x = std::max(br.x, j) ; br.y = std::max(br.y, i) ;

                reg->rect_ = cv::Rect(tl, br) ;

                reg->area_ ++ ;
            }
        }

    it_ = data_->begin() ;
}

cv::Mat RegionIterator::mask() const {
    const cv::Rect &r = it_->second.rect_ ;

    cv::Mat mask = cv::Mat::zeros(src_.size(), CV_8UC1) ;

    for( int y = r.tl().y ; y<=r.br().y ; y++ )
        for( int x=r.tl().x ; x<=r.br().x ; x++ )
        {
            unsigned int label = src_.at<unsigned int>(y, x) ;

            if ( label == it_->second.label_ )
            {
                mask.at<uchar>(y, x) = 255 ;
            }
        }

    return mask ;
}

vector<cv::Point> RegionIterator::contour() const {

    cv::Mat bmp = mask() ;

    vector<vector<cv::Point> > contours ;
    cv::findContours(bmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE) ;

    return contours[0] ;
}

}
}
