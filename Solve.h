//
// Created by 0-0 mashuo on 2023/3/9.
//

#ifndef RTK_SOLVE_H
#define RTK_SOLVE_H

#include "Matrix.h"


class LsRes;   // 最小二乘结果类
class ExtendedKalmanFilter; //卡尔曼滤波类
// 最小二乘法
/* H 几何构型矩阵
 * l 残差矩阵
 * P 权矩阵
 * n 变量个数
 * 输出：LeRes
 */
LsRes LeastSquare(Matrix H,Matrix l,Matrix P);

// 卡尔曼滤波

// 最小二乘法定位结果类
class LsRes{
public:
    Matrix Qxx;
    Matrix dx;
    double sigma2;
    Matrix V;

    LsRes();

    LsRes(Matrix, Matrix, Matrix , double);
};



class ExtendedKalmanFilter{
public:
    Matrix m_stateX;
    Matrix m_varianceP;

    // 构造函数
    ExtendedKalmanFilter(const Matrix &,const Matrix &);
    // 状态向量改变函数
    void state_change(const Matrix &,const Matrix &);
    // 状态更新函数
    void state_update(Matrix &,Matrix &,Matrix &,Matrix &,Matrix &,Matrix &);
};


#endif //RTK_SOLVE_H
