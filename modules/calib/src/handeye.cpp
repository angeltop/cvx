#include <cvx/calib/handeye.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>



#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <unsupported/Eigen/NonLinearOptimization>

#include <fstream>
#include <iostream>
#include <set>

#include "DualQuaternion.h"

using namespace std ;
using namespace Eigen ;

//#define EIGEN_NL_SOLVER 0
namespace cvx { namespace camera {
#ifdef EIGEN_NL_SOLVER
// Generic functor
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
};

struct SolverFunctor: public Functor<double>
{

    SolverFunctor(const vector<Affine3d> &A_, const vector<Affine3d> &B_):
        A(A_), B(B_) {}

  int operator()(const VectorXd &x, VectorXd &fvec) const
  {
      Matrix4d X;

      for(int i=0, k=0 ; i<3 ; i++)
          for(int j=0 ; j<4 ; j++, k++)
          {
              X(i, j) = x[k] ;
          }

      int n = A.size() ;

      Matrix3d Rx = X.block<3, 3>(0, 0) ;
      Vector3d Tx = X.block<3, 1>(0, 3) ;

      int k = 0 ;

      for(int r=0 ; r<n ; r++ )
      {
          Matrix3d Ra = A[r].rotation(), Rb = B[r].rotation() ;
          Vector3d Ta = A[r].translation(), Tb = B[r].translation() ;

          Matrix3d C = Ra * Rx - Rx * Rb ;
          Vector3d S = Ra * Tx + Ta - Rx * Tb - Tx ;

          for(int i=0 ; i<3 ; i++ )
              for(int j=0 ; j<3 ; j++ )
                  fvec[k++] = C(i, j) ;

          for(int i=0 ; i<3 ; i++ )
              fvec[k++] = S[i] ;
      }

      const double cfactor = 1.0e6 ;

      Matrix3d Cx = Rx.transpose() * Rx - Matrix3d::Identity() ;

      for(int i=0 ; i<3 ; i++ )
          for(int j=0 ; j<3 ; j++ )
              fvec[k++] = cfactor * Cx(i, j) ;

      return 0;
  }

  int inputs() const { return 12 ; }
  int values() const { return 12*A.size() + 9; } // number of constraints

  const vector<Affine3d> &A,  &B ;

};

// minimize the frobenius norm of residual errors in rotation and translation (according to Dornaika and Horaud)
// uses numerical differentiation for computing Jacobians

bool HandEyeCalibration::solveNonLinear(const vector<Affine3d> &A, const vector<Affine3d> &B,
                  Affine3d &X )
{
    SolverFunctor functor(A, B);
    NumericalDiff<SolverFunctor> numDiff(functor);

    LevenbergMarquardt<NumericalDiff<SolverFunctor>, double> lm(numDiff);

    VectorXd Y(12) ;

    for(int i=0, k=0 ; i<3 ; i++)
        for(int j=0 ; j<4 ; j++, k++)
           Y[k] = X(i, j) ;

    LevenbergMarquardtSpace::Status status = lm.minimizeInit(Y);
    do {
        status = lm.minimizeOneStep(Y);
        double fnorm = lm.fvec.blueNorm();

    } while ( status == LevenbergMarquardtSpace::Running );

    for(int i=0, k=0 ; i<3 ; i++)
        for(int j=0 ; j<4 ; j++, k++)
            X(i, j) = Y[k] ;

    Matrix3d Rx = X.rotation() ;
    Vector3d Tx = X.translation() ;

   // double fnorm = lm.fvec.blueNorm();

    X = Translation3d(Tx) * Rx ;

    return true ;
}
#else
template<typename T>
Eigen::Matrix<T, 3, 3> AngleAxisToRotationMatrix(const Eigen::Matrix<T, 3, 1>& rvec)
{
    T angle = rvec.norm();
    if (angle == T(0))
    {
        return Eigen::Matrix<T, 3, 3>::Identity();
    }

    Eigen::Matrix<T, 3, 1> axis;
    axis = rvec.normalized();

    Eigen::Matrix<T, 3, 3> rmat;
    rmat = Eigen::AngleAxis<T>(angle, axis);

    return rmat;
}

template<typename T>
Eigen::Quaternion<T> AngleAxisToQuaternion(const Eigen::Matrix<T, 3, 1>& rvec)
{
    Eigen::Matrix<T, 3, 3> rmat = AngleAxisToRotationMatrix<T>(rvec);

    return Eigen::Quaternion<T>(rmat);
}

template<typename T>
Eigen::Matrix<T, 3, 1> RotationToAngleAxis(const Eigen::Matrix<T, 3, 3> & rmat)
{
    Eigen::AngleAxis<T> angleaxis;
    angleaxis.fromRotationMatrix(rmat);
    return angleaxis.angle() * angleaxis.axis();

}


class PoseError
{
public:
    PoseError(Eigen::Quaterniond r1, Eigen::Vector3d t1,
              Eigen::Quaterniond r2, Eigen::Vector3d t2)
        : m_rvec1(r1), m_rvec2(r2), m_tvec1(t1), m_tvec2(t2)
    {}

    template<typename T>
    bool operator() (const T* const q4x1, const T* const t3x1, T* residual) const
    {
        Eigen::Quaternion<T> q(q4x1[0], q4x1[1], q4x1[2], q4x1[3]);
        Eigen::Matrix<T, 3, 1> t;
        t << t3x1[0], t3x1[1], t3x1[2];

        DualQuaternion<T> dq(q, t);

        Eigen::Quaternion<T> r1 = m_rvec1.cast<T>();
        Eigen::Matrix<T, 3, 1> t1 = m_tvec1.cast<T>();
        Eigen::Quaternion<T> r2 = m_rvec2.cast<T>();
        Eigen::Matrix<T, 3, 1> t2 = m_tvec2.cast<T>();

        DualQuaternion<T> dq1(r1, t1);
        DualQuaternion<T> dq2(r2, t2);
        DualQuaternion<T> dq1_ = dq * dq2 * dq.inverse();

        DualQuaternion<T> diff = (dq1.inverse() * dq1_).log();
        residual[0] = diff.real().squaredNorm() + diff.dual().squaredNorm();

        return true;
    }

private:
    Eigen::Quaterniond m_rvec1, m_rvec2 ;
    Eigen::Vector3d m_tvec1, m_tvec2;
public:
    /// @see http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class PoseError2
{
public:
    PoseError2(Eigen::Quaterniond r1, Eigen::Vector3d t1,
              Eigen::Quaterniond r2, Eigen::Vector3d t2)
        : m_rvec1(r1), m_rvec2(r2), m_tvec1(t1), m_tvec2(t2)
    {}

    template<typename T>
    bool operator() (const T* const q4x1, const T* const t3x1, T* residual) const
    {
        Eigen::Quaternion<T> q(q4x1[0], q4x1[1], q4x1[2], q4x1[3]);
        Eigen::Matrix<T, 3, 1> Tx;
        Tx << t3x1[0], t3x1[1], t3x1[2];

        Eigen::Matrix<T, 3, 3> Rx = q.toRotationMatrix();

        Eigen::Matrix<T, 3, 3> Ra = m_rvec1.toRotationMatrix().cast<T>();
        Eigen::Matrix<T, 3, 1> Ta = m_tvec1.cast<T>();
        Eigen::Matrix<T, 3, 3> Rb = m_rvec2.toRotationMatrix().cast<T>();
        Eigen::Matrix<T, 3, 1> Tb = m_tvec2.cast<T>();

        Matrix<T, 3, 3> C = Ra * Rx - Rx * Rb ;
        Matrix<T, 3, 1> S = Ra * Tx + Ta - Rx * Tb - Tx ;

        residual[0] = (T)C(0, 0) ;
        residual[1] = (T)C(0, 1) ;
        residual[2] = (T)C(0, 2) ;

        residual[3] = (T)C(1, 0) ;
        residual[4] = (T)C(1, 1) ;
        residual[5] = (T)C(1, 2) ;

        residual[6] = (T)C(2, 0) ;
        residual[7] = (T)C(2, 1) ;
        residual[8] = (T)C(2, 2) ;

        residual[9] = (T)S[0] ;
        residual[10] = (T)S[1] ;
        residual[11] = (T)S[2] ;

        return true;
    }

private:
    Eigen::Quaterniond m_rvec1, m_rvec2 ;
    Eigen::Vector3d m_tvec1, m_tvec2;
public:
    /// @see http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


bool HandEyeCalibration::solveNonLinear(const vector<Affine3d> &A, const vector<Affine3d> &B, Affine3d &X)
{

    Eigen::Matrix4d H = X.matrix();

    Quaterniond q(X.rotation()) ;

    double p[7] = {q.w(), q.x(), q.y(), q.z(),
                   H(0, 3), H(1, 3), H(2, 3)};


    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > rvecs1, rvecs2 ;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tvecs1, tvecs2 ;

    for( uint i=0 ; i<A.size() ; i++ ) {
        rvecs1.push_back(Quaterniond(A[i].rotation())) ;
        tvecs1.push_back(A[i].translation()) ;
        rvecs2.push_back(Quaterniond(B[i].rotation())) ;
        tvecs2.push_back(B[i].translation()) ;
    }

    ceres::Problem problem;
    for (size_t i = 0; i < rvecs1.size(); i++)
    {
        // ceres deletes the objects allocated here for the user
        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<PoseError2, 12, 4, 3>(
                new PoseError2(rvecs1[i], tvecs1[i], rvecs2[i], tvecs2[i]));

        problem.AddResidualBlock(costFunction, NULL, p, p + 4);
    }

    // ceres deletes the object allocated here for the user
    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(p, quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
 //   options.jacobi_scaling = true;
//    options.use_nonmonotonic_steps = true;
//    options.preconditioner_type = ceres::SCHUR_JACOBI;
 //   options.use_inner_iterations = false;
    options.max_num_iterations = 100;
  //  options.function_tolerance = 1.0e-10 ;
  //  options.gradient_tolerance = 1.0e-10 ;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    Eigen::Quaterniond qx(p[0], p[1], p[2], p[3]);
    Eigen::Vector3d t;
    t << p[4], p[5], p[6];

    X = Affine3d(Translation3d(t) * qx.toRotationMatrix()) ;

    return true ;
}
#endif
static void getDualQuaternion(const Affine3d &A, Quaterniond &q, Quaterniond &qp)
{
    AngleAxisd rod(A.rotation()) ;

    double theta = rod.angle() ;
    double hc = cos(theta/2.0) ;
    double hs = sin(theta/2.0) ;
    Vector3d a = rod.axis() ;

    Vector3d as = hs * a ;

    q = Quaterniond(hc, as.x(), as.y(), as.z()) ;

    Vector3d t = A.translation() ;

    double qpw = -t.dot(as) / 2.0 ;

    Vector3d qpv = (t.cross(as) + hc * t) / 2.0 ;

    qp = Quaterniond(qpw, qpv.x(), qpv.y(), qpv.z()) ;
}

static Matrix3d crossprod(const Vector3d &a)
{
    Matrix3d r ;
    r << 0, -a.z(), a.y(), a.z(), 0, -a.x(), -a.y(), a.x(), 0 ;
    return r ;
}

// Method by K. Danielidis

bool HandEyeCalibration::solveHandEyeLinearDualQuaternion(const vector<Affine3d> &A, const vector<Affine3d> &B,
                  Affine3d &X )
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;

    MatrixXd T(6*n, 8) ;

    for(int i=0 ; i<n ; i++)
    {
        Quaterniond qa, qb, qpa, qpb ;

        // compute dual quaternion representations

        getDualQuaternion(A[i], qa, qpa) ;
        getDualQuaternion(B[i], qb, qpb) ;

        // Form the A problem matrix (eq. 31)
        Vector3d s1 = qa.vec() - qb.vec() ;
        Matrix3d s2 = crossprod(qa.vec() + qb.vec()) ;
        Vector3d t1 = qpa.vec() - qpb.vec() ;
        Matrix3d t2 = crossprod(qpa.vec() + qpb.vec()) ;

        T.block<3, 1>(6*i, 0) = s1 ;
        T.block<3, 3>(6*i, 1) = s2 ;
        T.block<3, 1>(6*i, 4) = Vector3d::Zero() ;
        T.block<3, 3>(6*i, 5) = Matrix3d::Zero() ;

        T.block<3, 1>(6*i+3, 0) = t1 ;
        T.block<3, 3>(6*i+3, 1) = t2 ;
        T.block<3, 1>(6*i+3, 4) = s1 ;
        T.block<3, 3>(6*i+3, 5) = s2 ;
    }

    // Solve problem with SVD

    JacobiSVD<MatrixXd> svd(T, ComputeFullV) ;

    const MatrixXd V = svd.matrixV();
    const VectorXd S = svd.singularValues() ;

    // The last two singular values should be ideally zero

    const double singValThresh = 1.0e-1 ;

  //  if ( S[6] > singValThresh || S[7] > singValThresh ) return false ;

    // obtain right eigen-vectors spanning the solution space

    VectorXd v7 = V.col(6) ;
    VectorXd v8 = V.col(7) ;

    // find lambda1, lambda2 so that lambda1*v7 + lambda2*v8 = [q^T ;qprime^T]
    // Form the quadratic eq. 35 and solve it to obtain lamba1, lambda2

    Vector4d u1 = v7.segment<4>(0);
    Vector4d v1 = v7.segment<4>(4);

    Vector4d u2 = v8.segment<4>(0);
    Vector4d v2 = v8.segment<4>(4);

    double a = u1.dot(v1) ;
    double b = u1.dot(v2) + u2.dot(v1) ;
    double c = u2.dot(v2) ;

    // solve for s = lambda1/lambda2

    double det = sqrt(b * b - 4 * a * c) ;
    double s1 = (-b + det)/2/a ;
    double s2 = (-b - det)/2/a ;

    double a_ = u1.dot(u1) ;
    double b_ = u1.dot(u2) ;
    double c_ = u2.dot(u2) ;
    double s, val ;

    double val1 = s1 * s1 * a_ + 2 * s1 * b_ + c_ ;
    double val2 = s2 * s2 * a_ + 2 * s2 * b_ + c_ ;

    if ( val1 > val2 )  {
        s = s1 ;
        val = val1 ;
    }
    else  {
        s = s2 ;
        val = val2 ;
    }

    double lambda2 = sqrt(1/val) ;
    double lambda1 = s * lambda2 ;

    // compute the solution

    VectorXd sol = lambda1 * v7 + lambda2 * v8 ;

    Quaterniond q(sol[0], sol[1], sol[2], sol[3]) ;
    Quaterniond qp(sol[4], sol[5], sol[6], sol[7]) ;

    // obtain rotation and translation from dual quaternion

    Matrix3d rot = q.toRotationMatrix() ;
    Vector3d trans = 2 * (qp * q.conjugate()).vec() ;

    X = Translation3d(trans) * rot ;

    return true ;
}

bool HandEyeCalibration::solveHandEyeLinearTsai(const vector<Affine3d> &A, const vector<Affine3d> &B, Affine3d &X)
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;

    assert(n>2) ;

    MatrixXd A_(3*n, 3) ;
    VectorXd B_(3*n) ;

    for(int i=0 ; i<n ; i++)
    {
        AngleAxisd rg(A[i].rotation()) ;
        AngleAxisd rc(B[i].rotation()) ;

        double theta_g = rg.angle() ;
        double theta_c = rc.angle() ;

        Vector3d rng = rg.axis() ;
        Vector3d rnc = rc.axis() ;

        Vector3d Pg = 2*sin(theta_g/2)*rng;
        Vector3d Pc = 2*sin(theta_c/2)*rnc;

        A_.block<3, 3>(3*i, 0) = crossprod(Pg + Pc);
        B_.segment<3>(3*i) = Pc - Pg;
    }

    // Solve problem with SVD

    JacobiSVD<MatrixXd> svdR(A_, ComputeThinU | ComputeThinV) ;

    // compute rotation

    VectorXd Pcg_prime = svdR.solve(B_) ;
    double err = (A_* Pcg_prime - B_).norm()/n ;

    VectorXd Pcg = 2*Pcg_prime/(sqrt(1+Pcg_prime.squaredNorm()));
    MatrixXd Rcg = (1-Pcg.squaredNorm()/2)*Matrix3d::Identity() +
            0.5*(Pcg*Pcg.adjoint() + sqrt(4 - Pcg.squaredNorm())*crossprod(Pcg));

    // compute translation

    for(int i=0 ; i<n ; i++)
    {
        A_.block<3, 3>(3*i, 0) = A[i].rotation() - Matrix3d::Identity() ;
        B_.segment<3>(3*i) = Rcg * B[i].translation() - A[i].translation() ;
    }

    JacobiSVD<MatrixXd> svdT(A_, ComputeThinU | ComputeThinV) ;

    VectorXd Tcg = svdT.solve(B_) ;
    err = (A_* Tcg - B_).norm()/n ;

    X = Translation3d(Tcg) * Rcg ;

    return true ;
}

// Method by Horaud and Dornaika

bool HandEyeCalibration::solveHandEyeLinearHD(const vector<Affine3d> &A, const vector<Affine3d> &B, Affine3d &X)
{
    assert(A.size() == B.size()) ;

    int n = A.size() ;

    // Solve for rotation

    MatrixXd A_(4*n, 4) ;

    for(int i=0 ; i<n ; i++)
    {
        Quaterniond qa, qb ;
        qa = Quaterniond(A[i].rotation()) ;
        qb = Quaterniond(B[i].rotation()) ;

        // compute  (Q-W)'(Q-W)

        Matrix4d Q, W, QW ;

        Q <<     qa.w(), -qa.x(), -qa.y(), -qa.z(),
                 qa.x(),  qa.w(), -qa.z(),  qa.y(),
                 qa.y(),  qa.z(),  qa.w(), -qa.x(),
                 qa.z(), -qa.y(),  qa.x(),  qa.w();

        W <<     qb.w(), -qb.x(), -qb.y(), -qb.z(),
                 qb.x(),  qb.w(),  qb.z(), -qb.y(),
                 qb.y(), -qb.z(),  qb.w(),  qb.x(),
                 qb.z(),  qb.y(), -qb.x(),  qb.w();

        A_.block<4, 4>(4*i, 0) = Q - W ;

    }

    // Perform SVD to find rotation

    JacobiSVD<MatrixXd> svd(A_, ComputeFullV) ;

    const MatrixXd V = svd.matrixV();

    Vector4d vq = V.col(3) ;

    Quaterniond q(vq[0], vq[1], vq[2], vq[3]) ;

    Matrix3d R = q.toRotationMatrix() ;

    // Solve for translation

    MatrixXd MA(3*n, 3) ;
    VectorXd MB(3*n) ;

    for(int i=0 ; i<n ; i++)
    {
        MA.block<3, 3>(3*i, 0) = A[i].rotation() - Matrix3d::Identity() ;
        MB.segment<3>(3*i) = R * B[i].translation() - A[i].translation() ;
    }

    Vector3d T = MA.jacobiSvd(ComputeThinU | ComputeThinV).solve(MB) ;

    X = Translation3d(T) * R ;

    return true ;
}

void saveMotionsToFile(const vector<Affine3d> &motions, const string &fileName)
{
    ofstream strm(fileName.c_str()) ;

    for( uint i=0 ; i<motions.size() ; i++ )
    {
        Matrix4d m = motions[i].matrix() ;

        for(int r=0 ; r<4 ; r++ )
            for(int c=0 ; c<4 ; c++ )
                strm << m(c, r) << endl ;
    }
}

void HandEyeCalibration::sortStationMovements(vector<Affine3d> &gripper_to_base, vector<Affine3d> &target_to_sensor)
{
    int nFrames = gripper_to_base.size() ;

    vector<Affine3d> gripper_to_base_, target_to_sensor_ ;

    double dist ;

    int i=0 ;

    set<int> checked ;

    while ( 1 )
    {
        checked.insert(i) ;
        gripper_to_base_.push_back(gripper_to_base[i]) ;
        target_to_sensor_.push_back(target_to_sensor[i]) ;

        double maxAngle = 0.0 ;
        int bestj = -1 ;

        for( int j = 0 ; j<gripper_to_base.size() ; j++ )
        {
            if ( checked.count(j) == 0 )
            {
                Quaterniond qi(gripper_to_base[i].rotation()) ;
                Quaterniond qj(gripper_to_base[j].rotation()) ;

                double angle = qi.angularDistance(qj) ;

                if ( angle > maxAngle )
                {
                    maxAngle = angle ;
                    bestj = j ;
                }

            }
        }

        if ( bestj < 0 ) break ;

        i = bestj ;
    }


    gripper_to_base = gripper_to_base_ ;
    target_to_sensor = target_to_sensor_ ;

}

bool HandEyeCalibration::solveFixed(const vector<Affine3d> &gripper_to_base, const vector<Affine3d> &target_to_sensor,
                  Affine3d &sensor_to_base)
{
    vector<Affine3d> A, B ;
/*
    vector<Affine3d> gripper_to_base_(gripper_to_base), target_to_sensor_(target_to_sensor) ;

   sortStationMovements(gripper_to_base_, target_to_sensor_) ;

    for( uint i=0 ; i<target_to_sensor_.size()-1 ; i++)
    {
        A.push_back(gripper_to_base_[i+1] * gripper_to_base_[i].inverse() ) ;
        B.push_back(target_to_sensor_[i+1] * target_to_sensor_[i].inverse() ) ;
    }
    */

    for( uint i=0 ; i<target_to_sensor.size() ; i++ )
        for( uint j=i+1 ; j<gripper_to_base.size() ; j++ ) {
            Quaterniond qi(gripper_to_base[i].rotation()) ;
            Quaterniond qj(gripper_to_base[j].rotation()) ;

            double angle = qi.angularDistance(qj) ;

            if ( angle > M_PI * 10 / 180.0 ) {
                A.push_back(gripper_to_base[j] * gripper_to_base[i].inverse() ) ;
                B.push_back(target_to_sensor[j] * target_to_sensor[i].inverse() ) ;
            }

        }

    bool res ;

    if ( params_.method_ == Horaud )
        res = solveHandEyeLinearHD(A, B, sensor_to_base) ;
    else if ( params_.method_ == Tsai )
        res = solveHandEyeLinearTsai(A, B, sensor_to_base) ;
    else
        res = solveHandEyeLinearDualQuaternion(A, B, sensor_to_base) ;

    if ( !res ) return false ;

    if ( params_.refine_ )
        res = solveNonLinear(A, B, sensor_to_base) ;
/*
    Vector3d avg_t(0, 0, 0) ;
    Vector4d avg_q(0, 0, 0, 0) ;

    for( uint i=0 ; i<target_to_sensor.size() ; i++)
    {
        Affine3d target_to_gripper_ = (target_to_sensor[i] * sensor_to_base) * gripper_to_base[i].inverse() ;

        avg_t += target_to_gripper_.translation() ;
        Quaterniond q(target_to_gripper_.rotation()) ;

        avg_q += Vector4d(q.w(), q.x(), q.y(), q.z()) ;

    }

    avg_t /= target_to_sensor_.size() ;
    avg_q /= target_to_sensor_.size() ;

    avg_q.normalize() ;
*/

    return true ;

}

bool HandEyeCalibration::solveMoving(const vector<Affine3d> &gripper_to_base, const vector<Affine3d> &target_to_sensor,
                  Affine3d &sensor_to_gripper )
{
    vector<Affine3d> A, B ;

    vector<Affine3d> gripper_to_base_(gripper_to_base), target_to_sensor_(target_to_sensor) ;

    sortStationMovements(target_to_sensor_, gripper_to_base_) ;

    for( uint i=0 ; i<target_to_sensor_.size()-1 ; i++)
    {
        A.push_back(gripper_to_base_[i+1].inverse() * gripper_to_base_[i] ) ;
        B.push_back(target_to_sensor_[i+1] * target_to_sensor_[i].inverse() ) ;
    }

    bool res ;

    if ( params_.method_ == Horaud )
        res = solveHandEyeLinearHD(A, B, sensor_to_gripper) ;
    else if ( params_.method_ == Tsai )
        res = solveHandEyeLinearTsai(A, B, sensor_to_gripper) ;
    else
        res = solveHandEyeLinearDualQuaternion(A, B, sensor_to_gripper) ;

    if ( !res ) return false ;

    if ( params_.refine_ )
        res = solveNonLinear(A, B, sensor_to_gripper) ;

    return true ;

}


}}
