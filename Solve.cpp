//
// Created by 0-0 mashuo on 2023/3/9.
//

#include "Solve.h"


LsRes LeastSquare(Matrix H,Matrix l,Matrix P){
    Matrix x[4];
    Matrix dx;
    x[1] = (H.T()*P*H).inv();   // Q
    Matrix W = H.T()*P*l;
    dx = x[1]*W;
    x[0] = dx;                 // x
    x[3] = H*x[0] - l;         // V
    l = l - x[3];              // 更新l
    double sigma2 = (x[3].T()*P*x[3])(1,1)/(H.row-H.col);  // sigma2
    return {x[0],x[1],x[3],sigma2};
}

LsRes::LsRes() = default ;

LsRes::LsRes(Matrix a, Matrix b, Matrix c, double d) {
    dx = a;
    Qxx = b;
    V = c;
    sigma2 = d;
}


ExtendedKalmanFilter::ExtendedKalmanFilter(const Matrix & x, const Matrix & P) {
    m_stateX = x;
    m_varianceP = P;
}

void ExtendedKalmanFilter::state_change(const Matrix & x, const Matrix & P) {
    m_stateX.release();
    m_varianceP.release();
    m_stateX = x;
    m_varianceP = P;
}

void
ExtendedKalmanFilter::state_update(Matrix & F, Matrix & Q, Matrix & R, Matrix & H, Matrix & h,Matrix & L) {
    Matrix estimate_x = F * m_stateX;
    Matrix estimate_P = F * m_varianceP * F.T() + Q;
    Matrix K = estimate_P * H.T() * (H * estimate_P *H.T() + R).inv();
    m_stateX = estimate_x + K * (L - h);
    Matrix m = K * H;
    Matrix I = eye(m.row);
    m_varianceP = (I - m) * estimate_P * (I - m).T() + K * R *K.T();
}
