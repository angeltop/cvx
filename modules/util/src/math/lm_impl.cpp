#include <cvx/util/math/solvers/lm.hpp>

extern "C" {
#include <levmar/levmar.h>
}

namespace cvx {

template<>
int LemvarWrapper<double>::levmar_der(void (*func)(double *p, double *hx, int m, int n, void *adata),
                                      void (*jacf)(double *p, double *j, int m, int n, void *adata),
                                      double *p, double *x, int m, int n, int itmax, double *opts,
                                      double *info, double *work, double *covar, void *adata ) {
    return dlevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata) ;
}

template<>
int LemvarWrapper<float>::levmar_der(void (*func)(float *p, float *hx, int m, int n, void *adata),
                                     void (*jacf)(float *p, float *j, int m, int n, void *adata),
                                     float *p, float *x, int m, int n, int itmax, float *opts,
                                     float *info, float *work, float *covar, void *adata ) {
    return slevmar_der(func, jacf, p, x, m, n, itmax, opts, info, work, covar, adata) ;
}

template<>
int LemvarWrapper<double>::levmar_dif(void (*func)(double *p, double *hx, int m, int n, void *adata),
                                      double *p, double *x, int m, int n, int itmax, double *opts,
                                      double *info, double *work, double *covar, void *adata ) {
    return dlevmar_dif(func, p, x, m, n, itmax, opts, info, work, covar, adata) ;
}

template<>
int LemvarWrapper<float>::levmar_dif(void (*func)(float *p, float *hx, int m, int n, void *adata),
                                     float *p, float *x, int m, int n, int itmax, float *opts,
                                     float *info, float *work, float *covar, void *adata ) {
    return slevmar_dif(func, p, x, m, n, itmax, opts, info, work, covar, adata) ;
}

}
