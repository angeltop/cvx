#ifndef __LINE_SEARCH_H__
#define __LINE_SEARCH_H__

#include <Eigen/Core>

template<typename T>
struct LineSearchParams {
    T x_tol_, f_tol_, g_tol_, stp_min_, stp_max_ ;
    int max_fev_ ;
    bool use_gradients_ ; // the gradient of the objective function is evaluated to check convergence but maybe costly,
                          // if this is set to false then convergence is not tested i.e. search is limited by the number of function evlautions (max_fev_)

    LineSearchParams(): f_tol_(1e-4), g_tol_(1e-2), stp_min_(1e-15), stp_max_(1e15), max_fev_(20), use_gradients_(true) {}
};

template<typename Dtype, typename P, int Ord>
class MoreThuente {

public:

    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> Matrix;


    /**
   * @brief use MoreThuente Rule for (strong) Wolfe conditiions
   * @details [long description]
   *
   * @param search_dir search direction for next update step
   * @param obj_func handle to problem
   *
   * @return step-width
   */

    static Dtype linesearch(const LineSearchParams<Dtype> &params, const Vector &x, const Vector & search_dir, P &obj_func, Dtype fval, const Vector & grad,
                            const  Dtype alpha_init = 1.0) {

        // assume step width
        Dtype ak = alpha_init;

        Vector g = grad.eval();

        Vector s = search_dir.eval();
        Vector xx = x.eval();

        cvsrch(params, obj_func, xx, fval, g, ak, s);

        return ak;
    }

    static int cvsrch(const LineSearchParams<Dtype> &params, P &obj_func, Vector &x, Dtype f, Vector &g, Dtype &stp, Vector &s) {
        // we rewrite this from MIN-LAPACK and some MATLAB code
        int info           = 0;
        int infoc          = 1;
        const Dtype xtol   = params.x_tol_;
        const Dtype ftol   = params.f_tol_;
        const Dtype gtol   = params.g_tol_;
        const Dtype stpmin = params.stp_min_;
        const Dtype stpmax = params.stp_max_;
        const Dtype xtrapf = 4;
        const int maxfev   = params.max_fev_; ///???? originally 20
        int nfev           = 0;

        Dtype dginit = g.dot(s);
        if (dginit >= 0.0) {
            // no descent direction
            // TODO: handle this case
            return -1;
        }

        bool brackt      = false;
        bool stage1      = true;

        Dtype finit      = f;
        Dtype dgtest     = ftol * dginit;
        Dtype width      = stpmax - stpmin;
        Dtype width1     = 2 * width;
        Vector wa = x.eval();

        Dtype stx        = 0.0;
        Dtype fx         = finit;
        Dtype dgx        = dginit;
        Dtype sty        = 0.0;
        Dtype fy         = finit;
        Dtype dgy        = dginit;

        Dtype stmin;
        Dtype stmax;

        while (true) {

            // make sure we stay in the interval when setting min/max-step-width
            if (brackt) {
                stmin = std::min(stx, sty);
                stmax = std::max(stx, sty);
            } else {
                stmin = stx;
                stmax = stp + xtrapf * (stp - stx);
            }

            // Force the step to be within the bounds stpmax and stpmin.
            stp = std::max(stp, stpmin);
            stp = std::min(stp, stpmax);

            // Oops, let us return the last reliable values
            if (
                    (brackt && (stp <= stmin | stp >= stmax))
                    | (nfev >= maxfev - 1 ) | (infoc == 0)
                    | (brackt & (stmax - stmin <= xtol * stmax))) {
                stp = stx;
            }

            // test new point
            x = wa + stp * s;
            f = obj_func.value(x);
            if ( params.use_gradients_ ) obj_func.gradient(x, g);

            nfev++;
            Dtype dg = g.dot(s);
            Dtype ftest1 = finit + stp * dgtest;

            // all possible convergence tests
            if ((brackt & ((stp <= stmin) | (stp >= stmax))) | (infoc == 0))
                info = 6;

            if ((stp == stpmax) & (f <= ftest1) & (dg <= dgtest))
                info = 5;

            if ((stp == stpmin) & ((f > ftest1) | (dg >= dgtest)))
                info = 4;

            if (nfev >= maxfev)
                info = 3;

            if (brackt & (stmax - stmin <= xtol * stmax))
                info = 2;

            if ((f <= ftest1) & (fabs(dg) <= gtol * (-dginit)))
                info = 1;

            // terminate when convergence reached
            if (info != 0)
                return -1;

            if (stage1 & (f <= ftest1) & (dg >= std::min(ftol, gtol)*dginit))
                stage1 = false;

            if (stage1 & (f <= fx) & (f > ftest1)) {
                Dtype fm = f - stp * dgtest;
                Dtype fxm = fx - stx * dgtest;
                Dtype fym = fy - sty * dgtest;
                Dtype dgm = dg - dgtest;
                Dtype dgxm = dgx - dgtest;
                Dtype dgym = dgy - dgtest;

                cstep( stx, fxm, dgxm, sty, fym, dgym, stp, fm, dgm, brackt, stmin, stmax, infoc);

                fx = fxm + stx * dgtest;
                fy = fym + sty * dgtest;
                dgx = dgxm + dgtest;
                dgy = dgym + dgtest;
            } else {
                // this is ugly and some variables should be moved to the class scope
                cstep( stx, fx, dgx, sty, fy, dgy, stp, f, dg, brackt, stmin, stmax, infoc);
            }

            if (brackt) {
                if (fabs(sty - stx) >= 0.66 * width1)
                    stp = stx + 0.5 * (sty - stx);
                width1 = width;
                width = fabs(sty - stx);
            }
        }

        return 0;
    }

    static int cstep(Dtype& stx, Dtype& fx, Dtype& dx, Dtype& sty, Dtype& fy, Dtype& dy, Dtype& stp,
                     Dtype& fp, Dtype& dp, bool& brackt, Dtype& stpmin, Dtype& stpmax, int& info) {
        info = 0;
        bool bound = false;

        // Check the input parameters for errors.
        if ((brackt & ((stp <= std::min(stx, sty) ) | (stp >= std::max(stx, sty)))) | (dx * (stp - stx) >= 0.0)
                | (stpmax < stpmin)) {
            return -1;
        }

        Dtype sgnd = dp * (dx / fabs(dx));

        Dtype stpf = 0;
        Dtype stpc = 0;
        Dtype stpq = 0;

        if (fp > fx) {
            info = 1;
            bound = true;
            Dtype theta = 3. * (fx - fp) / (stp - stx) + dx + dp;
            Dtype s = std::max(theta, std::max(dx, dp));
            Dtype gamma = s * sqrt((theta / s) * (theta / s) - (dx / s) * (dp / s));
            if (stp < stx)
                gamma = -gamma;
            Dtype p = (gamma - dx) + theta;
            Dtype q = ((gamma - dx) + gamma) + dp;
            Dtype r = p / q;
            stpc = stx + r * (stp - stx);
            stpq = stx + ((dx / ((fx - fp) / (stp - stx) + dx)) / 2.) * (stp - stx);
            if (fabs(stpc - stx) < fabs(stpq - stx))
                stpf = stpc;
            else
                stpf = stpc + (stpq - stpc) / 2;
            brackt = true;
        } else if (sgnd < 0.0) {
            info = 2;
            bound = false;
            Dtype theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
            Dtype s = std::max(theta, std::max(dx, dp));
            Dtype gamma = s * sqrt((theta / s) * (theta / s)  - (dx / s) * (dp / s));
            if (stp > stx)
                gamma = -gamma;

            Dtype p = (gamma - dp) + theta;
            Dtype q = ((gamma - dp) + gamma) + dx;
            Dtype r = p / q;
            stpc = stp + r * (stx - stp);
            stpq = stp + (dp / (dp - dx)) * (stx - stp);
            if (fabs(stpc - stp) > fabs(stpq - stp))
                stpf = stpc;
            else
                stpf = stpq;
            brackt = true;
        } else if (fabs(dp) < fabs(dx)) {
            info = 3;
            bound = 1;
            Dtype theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
            Dtype s = std::max(theta, std::max( dx, dp));
            Dtype gamma = s * sqrt(std::max((Dtype)0., (theta / s) * (theta / s) - (dx / s) * (dp / s)));
            if (stp > stx)
                gamma = -gamma;
            Dtype p = (gamma - dp) + theta;
            Dtype q = (gamma + (dx - dp)) + gamma;
            Dtype r = p / q;
            if ((r < 0.0) & (gamma != 0.0)) {
                stpc = stp + r * (stx - stp);
            } else if (stp > stx) {
                stpc = stpmax;
            } else {
                stpc = stpmin;
            }
            stpq = stp + (dp / (dp - dx)) * (stx - stp);
            if (brackt) {
                if (fabs(stp - stpc) < fabs(stp - stpq)) {
                    stpf = stpc;
                } else {
                    stpf = stpq;
                }
            } else {
                if (fabs(stp - stpc) > fabs(stp - stpq)) {
                    stpf = stpc;
                } else {
                    stpf = stpq;
                }

            }
        } else {
            info = 4;
            bound = false;
            if (brackt) {
                Dtype theta = 3 * (fp - fy) / (sty - stp) + dy + dp;
                Dtype s = std::max(theta, std::max(dy, dp));
                Dtype gamma = s * sqrt((theta / s) * (theta / s) - (dy / s) * (dp / s));
                if (stp > sty)
                    gamma = -gamma;

                Dtype p = (gamma - dp) + theta;
                Dtype q = ((gamma - dp) + gamma) + dy;
                Dtype r = p / q;
                stpc = stp + r * (sty - stp);
                stpf = stpc;
            } else if (stp > stx)
                stpf = stpmax;
            else {
                stpf = stpmin;
            }
        }

        if (fp > fx) {
            sty = stp;
            fy = fp;
            dy = dp;
        } else {
            if (sgnd < 0.0) {
                sty = stx;
                fy = fx;
                dy = dx;
            }

            stx = stp;
            fx = fp;
            dx = dp;
        }

        stpf = std::min(stpmax, stpf);
        stpf = std::max(stpmin, stpf);
        stp = stpf;

        if (brackt & bound) {
            if (sty > stx) {
                stp = std::min<Dtype>(stx + 0.66 * (sty - stx), stp);
            } else {
                stp = std::max<Dtype>(stx + 0.66 * (sty - stx), stp);
            }
        }

        return 0;

    }

};


#endif

