#include <cvx/util/imgproc/gabor.hpp>

using namespace std ;

namespace cvx { namespace util {

static unsigned int bestGaborKernelSize(double kf, double sigma, double thresh = 1.0e-5)
{
    unsigned int n ;

    for(  n = 1 ; n<150 ; n += 2  )
    {
        double alpha = kf*kf/sigma/sigma ;
        double bias = exp(-sigma*sigma/2.0) ;

        double sre = alpha * exp(-alpha*n*n) * (cos(n*kf) - exp(-sigma*sigma/2)) ;
        double sim = alpha * exp(-alpha*n*n) * (sin(n*kf)) ;

        if ( sqrt(sre * sre + sim*sim) < thresh ) return n ;
    }

    return 0 ;
}

static void makeGaborKernels(double kf, double phi, double sigma, int n, vector<cv::Mat> &kernels)
{
    cv::Mat krn_real(2*n+1, 2*n+1, CV_32F) ;
    cv::Mat krn_imag(2*n+1, 2*n+1, CV_32F) ;

    double kx = kf * cos(phi) ;
    double ky = kf * sin(phi) ;

    int x, y, k = 0 ;

    double alpha = kf*kf/sigma/sigma ;
    double bias = exp(-sigma*sigma/2.0) ;

    double sum = 0.0 ;

    for( y=-n ; y<=n ; y++ )
    {
        for( x=-n ; x<=n ; x++ )
        {
            double beta = alpha * exp(-alpha*(x*x + y*y)/2.0) ;
            double theta = x * kx + y * ky ;

            double fre = beta * (cos(theta) - bias) ;
            double fim = beta * sin(theta) ;

            krn_real.at<float>(x + n, y + n) = fre ;
            krn_imag.at<float>(x + n, y + n) = fim ;

            sum += fre ;

            k++ ;
        }
    }

    kernels.push_back(krn_real) ;
    kernels.push_back(krn_imag) ;
}

void makeGaborFilterBank(int nOctaves, int nAngles, double sigma, vector<cv::Mat> &kernels)
{
    double oct = 2.0 ;
    int i, j ;
    for ( i=0 ; i<nOctaves ; i++ )
    {
        double theta = 0 ;

        unsigned int n = bestGaborKernelSize(sigma/oct, sigma) ;

        for( j=0 ; j<nAngles ; j++ )
        {
            makeGaborKernels(sigma/oct, theta, sigma, n, kernels) ;

            theta += M_PI/nAngles ;
        }
        oct *= 2 ;
    }
}

void applyGaborFilterBank(const cv::Mat &src, vector<cv::Mat> &kernels, vector<cv::Mat> &responses)
{
    responses.resize(kernels.size()/2) ;

#pragma omp parallel for shared(responses)
    for( int i=0 ; i<kernels.size() ; i+=2 )
    {
        cv::Mat res_real, res_imag, res_norm ;

        cv::filter2D(src, res_real, CV_32F, kernels[i]) ;
        cv::filter2D(src, res_imag, CV_32F, kernels[i+1]) ;

        cv::pow(res_real, 2, res_real);
        cv::pow(res_imag, 2, res_imag);
        cv::add(res_imag, res_real, res_norm);
        cv::pow(res_norm, 0.5, res_norm);
        cv::normalize(res_norm, res_norm, 0, 1, CV_MINMAX, CV_32F);
        responses[i/2] = res_norm ;
    }

}








}
}
