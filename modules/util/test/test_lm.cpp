#include <iostream>
#include <ostream>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <cvx/util/math/rng.hpp>
#include <cvx/util/math/solvers/lm.hpp>

using namespace std ;
using namespace cvx ;
using namespace Eigen ;


double x33[]={
    8.44E-1, 9.08E-1, 9.32E-1, 9.36E-1, 9.25E-1, 9.08E-1, 8.81E-1,
    8.5E-1, 8.18E-1, 7.84E-1, 7.51E-1, 7.18E-1, 6.85E-1, 6.58E-1,
    6.28E-1, 6.03E-1, 5.8E-1, 5.58E-1, 5.38E-1, 5.22E-1, 5.06E-1,
    4.9E-1, 4.78E-1, 4.67E-1, 4.57E-1, 4.48E-1, 4.38E-1, 4.31E-1,
    4.24E-1, 4.2E-1, 4.14E-1, 4.11E-1, 4.06E-1};


class DemoSolver {
public:

    size_t terms() const { return 33 ; }

    void errors(const VectorXd &p, VectorXd &f) {

        for(int i=0; i<terms(); ++i){
            double t=10*i;
            f[i] = p[0] + p[1]*exp(-p[3]*t) + p[2]*exp(-p[4]*t) - x33[i] ;
        }
    }

    void jacobian(const VectorXd &p, MatrixXd &jac) {
        double t, tmp1, tmp2;

        int i, j ;
        for(i=0; i<terms(); ++i){
            t=10*i;
            tmp1=exp(-p[3]*t);
            tmp2=exp(-p[4]*t);

            j = 0 ;

            jac(i, j++)=1.0;
            jac(i, j++)=tmp1;
            jac(i, j++)=tmp2;
            jac(i, j++) =-p[1]*t*tmp1;
            jac(i, j++) =-p[2]*t*tmp2;
        }

        //    cout << jac << endl ;
    }



};

/* Osborne's problem, minimum at (0.3754, 1.9358, -1.4647, 0.0129, 0.0221) */
void osborne(double *p, double *x, int m, int n, void *data)
{
    register int i;
    double t;

    for(i=0; i<n; ++i){
        t=10*i;
        x[i]=p[0] + p[1]*exp(-p[3]*t) + p[2]*exp(-p[4]*t);

    }

}

void jacosborne(double *p, double *jac, int m, int n, void *data)
{
    register int i, j;
    double t, tmp1, tmp2;

    for(i=j=0; i<n; ++i){
        t=10*i;
        tmp1=exp(-p[3]*t);
        tmp2=exp(-p[4]*t);

        jac[j++]=1.0;
        jac[j++]=tmp1;
        jac[j++]=tmp2;
        jac[j++]=-p[1]*t*tmp1;
        jac[j++]=-p[2]*t*tmp2;

    }
}

int main(int argc, char *argv[]) {

    DemoSolver evaluator ;

    LMSolver<double, DemoSolver> solver ;
    VectorXd X(5) ;
    X << 0.5, 1.5, -1.0, 1.0e-2, 2e-2 ;

    solver.minimizeDiff(evaluator, X) ;

    cout << X << endl ;



}
