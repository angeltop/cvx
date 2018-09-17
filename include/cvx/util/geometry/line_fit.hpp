#ifndef __LINE_FIT_HPP__
#define __LINE_FIT_HPP__

#include <cvx/util/geometry/line.hpp>

namespace cvx { namespace util {

// Least squares fit of line to list of points

template <class T, int D>
Line<T, D> fitLine(const PointList<T, D> &pts) {

    int N = pts.size() ;

    assert(N >= 1);

    typedef Point<T, D> point_t  ;

    if ( N == 2 ) {
        point_t p = pts[0] ;
        point_t d = pts[1] - pts[0] ;
        d.normalize() ;
        return Line<T, D>(p, d);
    }

    typename PointList<T, D>::matrix_t mat = pts.mat() ;

    Eigen::Matrix<T, 1, D> m = mat.colwise().mean();
    Eigen::Matrix<T, Eigen::Dynamic, D> centered = mat.rowwise() - m;
    Eigen::Matrix<T, D, D> cov = (centered.adjoint() * centered) / T(mat.rows() - 1);

    Eigen::Matrix<T, D, D> U ;
    Eigen::Matrix<T, D, 1> L ;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<T, D, D> > es;
    es.compute(cov) ;

    L = es.eigenvalues() ;
    U = es.eigenvectors() ;

    return Line<T, D>(m, U.col(1)) ;
}

// robust line fitting using RANSAC and re-weighted least squares using Huber influence

template <class T, int D>
Line<T, D> fitLineRobust(const PointList<T, D> &pts,
                               uint n_ransac_iter = 10,         // number of RANSAC iterations (more is better but slower)
                               uint n_ransac_samples = 10,      // number of RANSAC samples used for model estimation
                               const uint n_iwrls_iter = 10,    // maximum number of weighted least squares iterations
                               const T c_dist_thresh = 0.01,    // converge threshold, change of line origin
                               const T c_angle_thresh = 0.01,   // converge threshold, change of line angle
                               const T C = 1.345                // Huber constant (smaller value reduces the influence)
) {
    typedef Point<T, D> point_t  ;

    uint N = pts.size() ;

    assert(N >= 1);

    if ( N == 2 ) {
        point_t p = pts[0] ;
        point_t d = pts[1] - pts[0] ;
        d.normalize() ;
        return Line<T, D>(p, d);
    }

    point_t bm, bu ;
    T perr = std::numeric_limits<T>::max() ;

    for( int r = 0 ; r<n_ransac_iter ; r++ ) {

        // fit line using weighted least squares on a subset of points

        std::vector<T> weight(N, 0.0) ;
        std::vector<T> res(N) ;

        // select subset of samples

        uint n_samples = std::min(n_ransac_samples, N) ;

        std::vector<uint> samples ;
        sample_with_replacement(n_samples, (uint)N, samples) ;

        T wsum = 0 ;
        for( int i=0 ; i<n_samples ; i++ ) {
            int idx = samples[i] ;
            weight[idx] = 1.0 ;
            wsum += 1.0 ;
        }

        point_t pm, pu ; // previous estimates
        for( int iter=0 ; iter<n_iwrls_iter ; iter++ )
        {
            // compute regression line using weighted least sqaures

            point_t m, u ;

            m.setZero() ;

            for( int i=0 ; i<N ; i++ ) {
                point_t p = pts[i] ;
                m += weight[i] * p ;
            }

            m /= wsum ;

            Eigen::Matrix<T, D, D> cov ;
            cov.setZero() ;

            for( int i=0 ; i<N ; i++ ) {
                point_t p = pts[i] ;

                T w = weight[i] ;
                point_t pm = p - m ;

                Eigen::Matrix<T, D, D> U ;

                for( int k=0 ; k<D ; k++)
                    for( int l=0 ; l<D ; l++)
                        cov(k, l) += w * pm[k] * pm[l] ;
            }

            cov /= wsum ;

            Eigen::Matrix<T, D, D> U ;
            Eigen::Matrix<T, D, 1> L ;

            Eigen::SelfAdjointEigenSolver< Eigen::Matrix<T, D, D> > es;
            es.compute(cov) ;

            L = es.eigenvalues() ;
            U = es.eigenvectors() ;

            u = U.col(1) ;

            // compute residuals for given estimate

            T err = 0 ;
            std::vector<T> median(N) ;
            for( int i=0 ; i<N ; i++ )
            {
                point_t p = pts[i] ;

                point_t pm = p - m ;

                double r = pm.squaredNorm() ;
                double ss = pm.dot(u) ;

                r -= ss * ss ;
                r = sqrt(fabs(r)) ;

                weight[i] = r ;
                res[i] = r ;
                err += r ;
                median[i] = r ;
            }

            if ( err < perr ) {
                perr = err ;
                bm = m ;
                bu = u ;
            }

            // test convergence

            if ( iter > 0 ) {
                T pdist = (pm - m).norm() ;
                T pangle = 1 - fabs(pu.dot(u)) ;
                if ( pdist < c_dist_thresh && pangle < c_angle_thresh ) break ;
            }

            pm = m ;
            pu = u ;

            // Estimate MAD of residuals

            sort(median.begin(), median.end()) ;
            T med = median[N/2] ;

            for( int i=0 ; i<N ; i++ )
                weight[i] = fabs(weight[i] - med ) ;

            sort(weight.begin(), weight.end()) ;

            // estimate sigma

            T sigma = weight[N/2]/0.6745 ;

            // Update weights using Hubers scheme

            wsum = 0.0 ;

            for( int i=0 ; i<N ; i++ )
            {

                T r = fabs(res[i])/sigma ;

                if ( r <= C )  weight[i] = 1.0 ;
                else weight[i] = C/r ;

                wsum += weight[i] ;
            }
        }


    }

    return Line<T, D>(bm, bu) ;
}

}
}






#endif
