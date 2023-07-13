//
// Created by 0-0 mashuo on 2023/3/6.
//

#ifndef RTK_MATRIX_H
#define RTK_MATRIX_H

#include "vector"

int MatrixInv(int n,const double a[], double b[]);  // 矩阵求逆函数
// 考虑到卫星导航的数据结构，矩阵类只设计了double类型
struct Matrix {

public:
    //矩阵的行、列数
    int col{};
    int row{};
    double *p{};

public:
    Matrix();

    ~Matrix();  //折构函数

    void release();

    Matrix(int, int, const double[]);     //数组初始化函数

    Matrix(int, int,std::vector<double>); //vector初始化函数

    Matrix(const Matrix &); //拷贝折构函数

    double operator()(int, int) const; //矩阵索引运算符重载，只能获取矩阵在该索引处的值   索引从1开始

    void assign(int, int, double); //改变矩阵在索引处的值  索引从1开始

    Matrix &operator=(const Matrix &); //矩阵赋值运算符重载,仅允许空矩阵或者类型相同矩阵赋值

    friend Matrix operator+(const Matrix &, const Matrix &); //矩阵相加
    friend Matrix operator+(double, const Matrix &); //矩阵加常数
    friend Matrix operator+(const Matrix &, double); //矩阵加常数

    friend Matrix operator-(const Matrix &, const Matrix &); //矩阵相减
    friend Matrix operator-(double, const Matrix &); //矩阵减常数
    friend Matrix operator-(const Matrix &, double); //矩阵减常数

    friend Matrix operator*(const Matrix &, const Matrix &); //矩阵相乘

    friend Matrix operator^(const Matrix &, const Matrix &); //矩阵点乘
    friend Matrix operator^(double, const Matrix &); //矩阵数乘
    friend Matrix operator^(const Matrix &, double); //矩阵数乘

    Matrix inv() const; //矩阵求逆

    Matrix T() const; //矩阵转置

    double det() const; //求出矩阵的行列式值

    Matrix swaprow(int, int); //行交换(实参位置可变化)

    Matrix multirow(int row1, int row2, int n, double multiple); //将row1行元素经倍乘-multiple加到row2行元素，使得row2行元素中第n列元素值为0

    Matrix m_acmatrix(int, int) const;  // 返回i行j列的代数余子式

    double tr() const;  // 求出矩阵的迹

    Matrix min_matrix(int, int, int, int) const;   // 构建矩阵的一个分块矩阵

    void print() const; //打印矩阵
};

Matrix eye(int); //生成单位矩阵

Matrix zero(int,int); //生成a行b列的全零矩阵

Matrix horizontal_stack(Matrix m1,Matrix m2);

Matrix vertical_stack(Matrix m1,Matrix m2);

Matrix horizontal_stack_array(Matrix * , int);

Matrix vertical_stack_array(Matrix *, int);

Matrix diag(double *,int);

Matrix diag(Matrix *,int);






#endif //RTK_MATRIX_H
