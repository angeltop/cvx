#ifndef __LM_SOLVER_HPP__
#define __LM_SOLVER_HPP__

#include <Eigen/Core>

namespace cvx {

template<typename T>
struct LemvarWrapper {
    static int levmar_der( void (*func)(T *p, T *hx, int m, int n, void *adata),
                           void (*jacf)(T *p, T *j, int m, int n, void *adata),
                           T *p, T *x, int m, int n, int itmax, T *opts,
                           T *info, T *work, T *covar, void *adata);

    static int levmar_dif( void (*func)(T *p, T *hx, int m, int n, void *adata),
                           T *p, T *x, int m, int n, int itmax, T *opts,
                           T *info, T *work, T *covar, void *adata);
};

/*  Levenberg-Marquardt non-linear least squares solver
    A wrapper of Lourakis levmar library (www.ics.forth.gr/~lourakis/levmar) suitable for medium size problems
    Template is parameterized with data type i.e. float or double and objective function
    The objective function should be of the form:


    class ObjFunc {
    public:

        size_t terms() const ; // should return the number of terms in least squares summation (for example if it is 2D curve fitting it is 2 times the number of points)

        void values(const VectorXd &p, VectorXd &f) ; // the error computed for all terms given the parameter vector

        void jacobian(const VectorXd &p, MatrixXd &jac) ; // the jacobian of the errors (terms x num_params) if analytic derivates are used
    };

    The data type e.g. VectorXd should match the template type T above (i.e. double in this example).
*/

template<typename T, typename ObjFunc>
class LMSolver {
public:

    struct Parameters {
        T factor_ = (T)1.0e-3 ;
        T g_tol_ = std::numeric_limits<T>::epsilon() ; // ||J^T e||_inf
        T x_tol_ = std::numeric_limits<T>::epsilon() ; // ||Dp||_2
        T f_tol_ = std::numeric_limits<T>::epsilon() ; // ||e||_2
        uint max_iter_ = 100 ;
        T delta_ = 1.0e-6 ;  // step used for finite difference approximation of derivatives
    };

    LMSolver() {}
    LMSolver(const Parameters &params): params_(params) {}

    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> Matrix;

    // minimize objective function with analytic derivatives
    
    void minimizeDer(ObjFunc &obj_func, Vector &x0) {
        opts_[0] = params_.factor_; opts_[1] = params_.g_tol_; opts_[2] = params_.x_tol_ ; opts_[3] = params_.g_tol_;

        LemvarWrapper<T>::levmar_der(feval, fjac, (T *)x0.data(), NULL, x0.rows(), obj_func.terms(), params_.max_iter_, opts_, info_, NULL, NULL, (void *)&obj_func); // with analytic Jacobian
    }

    // minimize objective function with approximated derivatives

    void minimizeDiff(ObjFunc &obj_func, Vector &x0) {
        opts_[0] = params_.factor_; opts_[1] = params_.g_tol_; opts_[2] = params_.x_tol_ ; opts_[3] = params_.g_tol_;
        opts_[4] = params_.delta_ ;

        LemvarWrapper<T>::levmar_dif(feval, (T *)x0.data(), NULL, x0.rows(), obj_func.terms(), params_.max_iter_, opts_, info_, NULL, NULL, (void *)&obj_func); // with computed Jacobiam
    }

    T getMinSquareError() const {
        return info_[1] ;
    }

private:

    static void feval(T *p, T *x, int m, int n, void *data) {
        ObjFunc *obj = (ObjFunc *)data ;

        Eigen::Map<Vector> X(p, m) ;

        Vector F(n) ;
        obj->errors(X, F) ;

        Eigen::Map<Vector> xmap(x, n) ;
        xmap = F ;
    }

    static void fjac(T *p, T *j, int m, int n, void *data) {
        ObjFunc *obj = (ObjFunc *)data ;

        Eigen::Map<Vector> X(p, m) ;

        Matrix J(n, m) ;

        obj->jacobian(X, J) ;

        Eigen::Map<Matrix> jmap(j, m, n) ;
        jmap = J.adjoint() ;
    }


    Parameters params_ ;
    T opts_[5], info_[10];

};


}

#endif

