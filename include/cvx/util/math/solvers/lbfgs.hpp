#ifndef __LBFGS_SOLVER_HPP__
#define __LBFGS_SOLVER_HPP__

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
class LBFGSSolver {
public:

    struct Parameters {
        T g_tol_, x_tol_ ;
        uint max_iter_ ;

        T M_ ;

        LineSearchParams<T> ls_ ;

        Parameters(): M_(10), g_tol_(1.0e-15), x_tol_(1.0e-7), max_iter_(30) {}
    };

    LBFGSSolver() {}
    LBFGSSolver(const Parameters &params): params_(params) {}

    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Matrix;

    void minimize(ObjFunc &obj_func, Vector &x0) {
           const size_t m = params_.M_ ;
           const size_t d = x0.rows();
           Matrix sVector = Matrix::Zero(d, m);
           Matrix yVector = Matrix::Zero(d, m);
           Eigen::Matrix<T, Eigen::Dynamic, 1> alpha = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(m);

           Vector grad(d), q(d), grad_old(d), s(d), y(d);
           obj_func.gradient(x0, grad);

           Vector x_old = x0;

           size_t iter = 0;
           T H0k = 1;
           T grad_norm = 0 ;

           do {
               const T relative_epsilon = static_cast<T>(0.0001) * std::max(static_cast<T>(1.0), x0.norm());

               if (grad.norm() < relative_epsilon)
                   break;

               //Algorithm 7.4 (L-BFGS two-loop recursion)
               q = grad;
               const int k = std::min(m, iter);

               // for i = k − 1, k − 2, . . . , k − m§
               for (int i = k - 1; i >= 0; i--) {
                   // alpha_i <- rho_i*s_i^T*q
                   const double rho = 1.0 / static_cast<Vector>(sVector.col(i))
                   .dot(static_cast<Vector>(yVector.col(i)));
                   alpha(i) = rho * static_cast<Vector>(sVector.col(i)).dot(q);
                   // q <- q - alpha_i*y_i
                   q = q - alpha(i) * yVector.col(i);
               }
               // r <- H_k^0*q
               q = H0k * q;
               //for i k − m, k − m + 1, . . . , k − 1
               for (int i = 0; i < k; i++) {
                   // beta <- rho_i * y_i^T * r
                   const T rho = 1.0 / static_cast<Vector>(sVector.col(i))
                   .dot(static_cast<Vector>(yVector.col(i)));
                   const T beta = rho * static_cast<Vector>(yVector.col(i)).dot(q);
                   // r <- r + s_i * ( alpha_i - beta)
                   q = q + sVector.col(i) * (alpha(i) - beta);
               }
               // stop with result "H_k*f_f'=q"

               // any issues with the descent direction ?
               T descent = -grad.dot(q);
               T alpha_init =  1.0 / grad.norm();
               if (descent > -0.0001 * relative_epsilon) {
                   q = -1 * grad;
                   iter = 0;
                   alpha_init = 1.0;
               }

               // find steplength
               const T rate = LS::linesearch(params_.ls_, x0, -q,  obj_func, obj_func.value(x0), grad, alpha_init) ;
               // update guess
               x0 = x0 - rate * q;

               grad_old = grad;
               obj_func.gradient(x0, grad);

               s = x0 - x_old;
               y = grad - grad_old;

               // update the history
               if (iter < m) {
                   sVector.col(iter) = s;
                   yVector.col(iter) = y;
               } else {

                   sVector.leftCols(m - 1) = sVector.rightCols(m - 1).eval();
                   sVector.rightCols(1) = s;
                   yVector.leftCols(m - 1) = yVector.rightCols(m - 1).eval();
                   yVector.rightCols(1) = y;
               }
               // update the scaling factor
               H0k = y.dot(s) / static_cast<double>(y.dot(y));

               grad_norm = grad.template lpNorm<Eigen::Infinity>();
               // std::cout << "iter: "<<iter<< " f = " <<  objFunc.value(x0) << " ||g||_inf "<<grad_norm   << std::endl;
               //std::cout << "-- " << iter <<  " iters -- " << objFunc.value(x0) << " -- " << (T)(x_old-x0).template lpNorm<Eigen::Infinity>()  << " -- " << grad_norm << "\n";
               if ( (x_old - x0).template lpNorm<Eigen::Infinity>() < params_.x_tol_  ) break;
               x_old = x0;
               iter++;

           } while ((grad_norm > params_.g_tol_) && (iter < params_.max_iter_)) ;
       }

    Parameters params_ ;

};

}
}
#endif
