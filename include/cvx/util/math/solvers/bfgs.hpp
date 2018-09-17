#ifndef __BFGS_SOLVER_HPP__
#define __BFGS_SOLVER_HPP__

#include <cvx/util/math/solvers/util.hpp>
#include <cvx/util/math/solvers/line_search.hpp>

#include <Eigen/Core>

// Adopted from https://github.com/PatWie/CppNumericalSolvers
// modified to receive parameters
//
// The objective function is a functor of the form
//
// struct ObjFunc {
//    float value(const VectorXf &x) ;
//    void  gradient(const VectorXf &x, VectorXf &grad) ;
// } ;

namespace cvx { namespace util {

template<typename T, typename ObjFunc, typename LS = MoreThuente<T, ObjFunc, 1> >
class BFGSSolver {
public:

    struct Parameters {
        T g_tol_, x_tol_ ;
        uint max_iter_ ;

        LineSearchParams<T> ls_ ;

        Parameters(): g_tol_(1.0e-15), x_tol_(1.0e-7), max_iter_(30) {}
    };

    BFGSSolver() {}
    BFGSSolver(const Parameters &params): params_(params) {}

    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;


    void minimize(ObjFunc &obj_func, Vector &x0) {

        const size_t d = x0.rows();
        size_t iter = 0;

        Matrix H = Matrix::Identity(d, d);

        Vector grad(d) ;
        T grad_norm = 0 ;
        Vector x_old = x0;

        obj_func.gradient(x0, grad);

        do {
            Vector search_dir = -1 * H * grad;

            // check "positive definite"
            T phi = grad.dot(search_dir);

            // positive definit ?
            if (phi > 0) {
                // no, we reset the hessian approximation
                H = Matrix::Identity(d, d);
                search_dir = -1 * grad;
            }

            const double rate = LS::linesearch(params_.ls_, x0, search_dir, obj_func, obj_func.value(x0), grad ) ;

            Vector s = rate * search_dir;

            x0 = x0 + s ; // update solution

            Vector grad_old = grad;
            obj_func.gradient(x0, grad);
            Vector y = grad - grad_old;

            double yDot = y.dot(s);

            if ( yDot!=0 ) {
                const double rho = 1.0 / yDot;
                H = H - rho * (s * (y.transpose() * H) + (H * y) * s.transpose()) + rho * rho * (y.dot(H * y) + 1.0 / rho)
                        * (s * s.transpose());
            }
            grad_norm = grad.template lpNorm<Eigen::Infinity>();
            // std::cout << "iter: "<<iter<< " f = " <<  objFunc.value(x0) << " ||g||_inf "<<grad_norm   << std::endl;
            //std::cout << "-- " << iter <<  " iters -- " << objFunc.value(x0) << " -- " << (T)(x_old-x0).template lpNorm<Eigen::Infinity>()  << " -- " << grad_norm << "\n";
            if ( (x_old - x0).template lpNorm<Eigen::Infinity>() < params_.x_tol_  ) break;
            x_old = x0;
            iter++;

        } while ((grad_norm > params_.g_tol_) && (iter < params_.max_iter_));

        //std::cout << "-- " << iter <<  " iters -- " << objFunc.value(x0) << " -- " << (T)(x_old-x0).template lpNorm<Eigen::Infinity>()  << " -- " << grad_norm << " --";
    }

    Parameters params_ ;


};

}
}
#endif
