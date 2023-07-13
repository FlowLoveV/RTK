//
// Created by 0-0 mashuo on 2023/3/6.
//

#include "string"
#include "iostream"
#include "typeinfo"
#include "cmath"
#include "Matrix.h"


Matrix::Matrix() {
    //默认构造函数
    this->col = 0;
    this->row = 0;
    this->p = nullptr;
}

Matrix::Matrix(const int row, const int col, const double t[]) {
    //初始化函数
    this->col = col;
    this->row = row;
    p = new double[col * row];
    for (int i = 0; i < row * col; ++i) {
        *(p + i) = *(t + i);
    }
}

Matrix::Matrix(const Matrix &b) {
    // 赋值
    row = b.row;
    col = b.col;
    p = new double[b.row * b.col];
    memcpy(p, b.p, b.col * b.row * sizeof(double));
}

void Matrix::print() const{
    std::cout << this->row << '\t' << this->col << '\n';
    for (int i = 0; i < this->row; ++i) {
        for (int j = 0; j < this->col; ++j) {
            std::cout << *(this->p + i * this->col + j) << "    ";
        }
        std::cout << '\n';
    }
}

double Matrix::operator()(int m, int n) const {
    if (m > this->row || n > this->col) {
        std::cerr << "Error in operator ():matrix index exceed limit" << '\n';
    } else if (m <= 0 || n <= 0) {
        std::cerr << "Error in operator ():matrix index must be positive" << '\n';
    } else {
        return *(this->p + (m - 1) * col + n - 1);
    }
}

void Matrix::assign(int i, int j, double value) {
    if (i > this->row || j > this->col) {
        std::cerr << "Error in assign():matrix index exceed limit" << '\n';
    } else if (i <= 0 || j <= 0) {
        std::cerr << "Error in assign():matrix index must be positive" << '\n';
    }
    if ((*this)(i, j) == value) {}
    else {
        *(this->p + (i - 1) * this->col + (j - 1)) = value;
    }
}

Matrix &Matrix::operator=(const Matrix &b) {
    if (this == &b) return *this;   // 相等时，不用赋值
    if (p == nullptr)  // p为空指针时
    {
        p=new double[b.row*b.col];
        row = b.row;
        col = b.col;
        memcpy(p, b.p, b.col * b.row * sizeof(double));
        return *this;
    }
    if(col == b.col && row == b.row)
    {
        for (int i = 0; i <col*row; ++i) {
            *(p + i) = b.p[i];
        }
        return *this;
    }
    // 不符合上述情况时
    std::cerr<<"矩阵大小不相等，无法进行赋值操作";
}

Matrix operator+(const Matrix &a, const Matrix &b) {
    // 非相同大小矩阵无法相加
    if (a.row != b.row || a.col != b.col) {
        std::cerr << "Error in operator +:matrices of different sizes can't be added" << '\n';
    }
    auto *p = new double[a.row * a.col];
    for (int i = 0; i < a.row * a.col; ++i) {
        *(p + i) = *(a.p + i) + *(b.p + i);
    }
    Matrix m(a.row, a.col, p);
    return m;
}

Matrix operator+(const double a, const Matrix &b) {
    auto *p = new double[b.row * b.col];
    for (int i = 0; i < b.row * b.col; ++i) {
        *(p + i) = a + *(b.p + i);
    }
    Matrix m(b.row, b.col, p);
    return m;
}

Matrix operator+(const Matrix &b, const double a) {
    auto *p = new double[b.row * b.col];
    for (int i = 0; i < b.row * b.col; ++i) {
        *(p + i) = a + *(b.p + i);
    }
    Matrix m(b.row, b.col, p);
    return m;
}

Matrix operator-(const Matrix &a, const Matrix &b) {
    // 非相同大小矩阵无法相加
    if (a.row != b.row || a.col != b.col) {
        std::cerr << "Error in operator -:matrices of different sizes can't minus" << '\n';
    }
    auto *p = new double[a.row * a.col];
    for (int i = 0; i < a.row * a.col; ++i) {
        *(p + i) = *(a.p + i) - *(b.p + i);
    }
    Matrix m(a.row, a.col, p);
    return m;
}

Matrix operator-(const double a, const Matrix &b) {
    auto *p = new double[b.row * b.col];
    for (int i = 0; i < b.row * b.col; ++i) {
        *(p + i) = a - *(b.p + i);
    }
    Matrix m(b.row, b.col, p);
    return m;
}

Matrix operator-(const Matrix &b, const double a) {
    auto *p = new double[b.row * b.col];
    for (int i = 0; i < b.row * b.col; ++i) {
        *(p + i) = -a + *(b.p + i);
    }
    Matrix m(b.row, b.col, p);
    return m;
}

Matrix operator*(const Matrix &a, const Matrix &b) {
    if (a.col != b.row) std::cerr << "Error in operator *:不满足矩阵相乘的条件!" << '\n';
    Matrix m;
    m.row = a.row;
    m.col = b.col;
    m.p = new double[a.row * b.col];
    for (int i = 1; i < a.row + 1; ++i) {
        for (int j = 1; j < b.col + 1; ++j) {
            double value = 0;
            for (int k = 1; k < a.col + 1; ++k) {
                value = value + (*(a.p + (i - 1) * a.col + k - 1)) * (*(b.p + (k - 1) * b.col + j - 1));
            }
            m.assign(i, j, value);
        }
    }
    return m;
}

Matrix operator^(const Matrix &a, const Matrix &b) {
    // 前列等后行
    if (a.row != b.row || a.col != b.col) {
        std::cerr << "Error in operator ^:matrices of different sizes can't .*" << '\n';
    }
    auto *p = new double[a.row * a.col];
    for (int i = 0; i < a.row * a.col; ++i) {
        *(p + i) = *(a.p + i) * *(b.p + i);
    }
    Matrix m(a.row, a.col, p);
    return m;
}

Matrix operator^(const double a, const Matrix &b) {
    if(b.p == nullptr) return Matrix();
    auto *p = new double[b.row * b.col];
    for (int i = 0; i < b.row * b.col; ++i) {
        *(p + i) = a * *(b.p + i);
    }
    Matrix m(b.row, b.col, p);
    return m;
}

Matrix operator^(const Matrix &b, const double a) {
    if(b.p == nullptr) return Matrix();
    auto *p = new double[b.row * b.col];
    for (int i = 0; i < b.row * b.col; ++i) {
        *(p + i) = a * *(b.p + i);
    }
    Matrix m(b.row, b.col, p);
    return m;
}

Matrix Matrix::T() const{
    double temp;
    Matrix m;
    m.row = col;
    m.col = row;
    m.p = new double[col * row];
    for (int i = 1; i < row + 1; ++i) {
        for (int j = 1; j < col + 1; ++j) {
            m.assign(j, i, (*this)(i, j));
        }
    }
    return m;
}

Matrix eye(const int n) {
    if(n<=0) return Matrix();
    Matrix m;
    m.col = m.row = n;
    m.p = new double[n * n];
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) { m.assign(i + 1, j + 1, 1); }
            else { m.assign(i + 1, j + 1, 0); }
        }
    }
    return m;
}

Matrix Matrix::swaprow(const int n1, const int n2) {
    if ((n1 > row || n1 <= 0) && (n2 > row || n2 <= 0)) {
        std::cerr << "Error in swaprow(): index exceed or index is negative!" << '\n';
    } else {
        double temp;
        for (int i = 1; i < col + 1; ++i) {
            temp = (*this)(n1, i);
            (*this).assign(n1, i, (*this)(n2, i));
            (*this).assign(n2, i, temp);
        }
    }
    return *this;
}

Matrix Matrix::inv() const{
    Matrix copy(*this);
    if (col != row) { std::cerr << "Error in inv():非方阵没有逆矩阵！" << '\n'; }
    MatrixInv(col, this->p, copy.p);
    return copy;
}

Matrix Matrix::multirow(const int row1, const int row2, const int n, const double multiple) {
    // 如果(row1,n)处元素值为0或者(row2,n）处元素等于0，则无需操作，直接返回原矩阵
    if ((*this)(row1, n) == 0 || (*this)(row2, n) == 0) { return *this; }
    // n的限制
    if (n <= 0 || n > col) { std::cerr << "Error in multirow(): n的值必须在1到col之间" << '\n'; }
    double temp;
    for (int i = 1; i < row + 1; ++i) {
        temp = (*this)(row2, i);
        (*this).assign(row2, i, temp - multiple * (*this)(row1, i));
    }
    return *this;
}

Matrix::~Matrix() {
    release();
}

void Matrix::release() {
    delete[]p;
    p = nullptr;
    row = 0;
    col = 0;
}

double Matrix::det() const{
    if(row!=col){
        std::cerr<<"error in det():非方阵无法求矩阵行列式的值";
    }
    double value = 0;
    if(row==1) return *p;
    if(row==2){
        value = (*this)(1,1) * (*this)(2,2) - (*this)(1,2) * (*this)(2,1);
    }
    else{
        for (int i = 1; i < row+1; ++i) {
            Matrix ac_matrix = this->m_acmatrix(1, i);
            value += (*this)(1, i)*ac_matrix.det()*pow(-1,1+i);
        }
    }
    return value;
}

Matrix Matrix::m_acmatrix(int i, int j) const{
    std::vector<double> acp;
    for (int k = 0; k < row*col; ++k) {
        if( k / row != i-1 && k % row != j-1){
            acp.push_back(*(p+k));
        }
    }
    auto *newp = new double[(row-1)*(row-1)];
    newp=&acp[0];
    return {row-1,row-1,newp};
}

double Matrix::tr() const{
    // 容错处理
    if(row!=col){
        std::cerr<<"error in Matrix::tr():非方阵不能求迹";
    }
    double a=0;
    for (int i = 1; i < row+1; ++i) {
        a += (*this)(i,i);
    }
    return a;
}

Matrix::Matrix(int row, int col, std::vector<double> t) {
    //初始化函数
    this->col = col;
    this->row = row;
    p = new double[col * row];
    for (int i = 0; i < row * col; ++i) {
        *(p + i) = t[i];
    }
}

/* 返回矩阵为
 * 行：row0 - row1
 * 列：col0 - col1
 */
Matrix Matrix::min_matrix(int row0, int row1, int col0, int col1) const{
    // 错误判断
    if(row0<=0 ||  row1<row0  || row1>row || col0<=0 || col0>col1 || col1>col){
        std::cerr<<"error in Matrix::min_matrix():矩阵索引越界或索引顺序相反";
    }
    std::vector<double> v;
    for (int i = row0; i <= row1; ++i) {
        for (int j = col0; j <= col1; ++j) {
            v.push_back((*this)(i,j));
        }
    }
    return {row1-row0+1,col1-col0+1,v};
}


Matrix horizontal_stack(Matrix m1, Matrix m2) {
    // 兼容处理
    if(m1.row == 0 && m2.row == 0) return Matrix();
    if(m1.row == 0) return m2;
    if(m2.row == 0) return m1;
    // 报错处理
    if(m1.row!=m2.row){
        std::cerr<<"error in horizontal_stack():Matrix1 and Matrix2 don't have matching row!";
    }
    // 正常处理
    int row=m1.row,col=m1.col+m2.col;
    auto *p = new double[row*col];
    Matrix m(row,col,p);
    for (int i = 1; i < row+1; ++i) {
        for (int j = 1; j < m1.col+1; ++j) {
            m.assign(i,j,m1(i,j));
        }
        for (int k = m1.col+1; k <col+1 ; ++k) {
            m.assign(i,k,m2(i,k-m1.col));
        }
    }
    return m;
}

Matrix vertical_stack(Matrix m1,Matrix m2){
    // 兼容处理
    if(m1.col == 0 && m2.col == 0) return Matrix();
    if(m1.col == 0) return m2;
    if(m2.col == 0) return m1;
    // 报错处理
    if(m1.col!=m2.col){
        std::cerr<<"error in vertical_stack():Matrix1 and Matrix2 don't have matching col";
    }
    // 正常处理
    int row=m1.row+m2.row,col=m1.col;
    auto p = new double[row*col];
    Matrix m(row,col,p);
    for (int i = 1; i < col+1; ++i) {
        for (int j = 1; j < m1.row+1; ++j) {
            m.assign(j,i,m1(j,i));
        }
        for (int k = m1.row+1; k<row+1 ; ++k) {
            m.assign(k,i,m2(k-m1.row,i));
        }
    }
    return m;
}


int MatrixInv(int n,const double a[], double b[]) {
    int i, j, k, l, u, v, is[n], js[n];   /* matrix dimension <= 10 */
    double d, p;

    if (n <= 0) {
        printf("Error dimension in MatrixInv!\n");
        exit(EXIT_FAILURE);
    }
    /* 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变 */
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            b[i * n + j] = a[i * n + j];
        }
    }
    for (k = 0; k < n; k++) {
        d = 0.0;
        for (i = k; i < n; i++)   /* 查找右下角方阵中主元素的位置 */
        {
            for (j = k; j < n; j++) {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d) {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }
        if (d < 1e-16)   /* 主元素接近于0，矩阵不可逆 */
        {
            printf("Divided by 0 in MatrixInv!\n");
            exit(EXIT_FAILURE);
        }

        if (is[k] != k)  /* 对主元素所在的行与右下角方阵的首行进行调换 */
        {
            for (j = 0; j < n; j++) {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (js[k] != k)  /* 对主元素所在的列与右下角方阵的首列进行调换 */
        {
            for (i = 0; i < n; i++) {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        l = k * n + k;
        b[l] = 1.0 / b[l];  /* 初等行变换 */
        for (j = 0; j < n; j++) {
            if (j != k) {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++) {
            if (i != k) {
                for (j = 0; j < n; j++) {
                    if (j != k) {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++) {
            if (i != k) {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }

    for (k = n - 1; k >= 0; k--)  /* 将上面的行列调换重新恢复 */
    {
        if (js[k] != k) {
            for (j = 0; j < n; j++) {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k) {
            for (i = 0; i < n; i++) {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }

    return (1);
}

Matrix zero(int a, int b) {
    std::vector<double> v(a*b,0);
    auto *p = new double[a*b];
    p=&v[0];
    return {a,b,p};
}


Matrix horizontal_stack_array(Matrix *m,int n){
    // 容错处理
    for (int i = 0; i < n-1; ++i) {
        if(m[i].row != m[i+1].row && !m[i].row && !m[i+1].row){
            std::cerr<<"error in horizontal_stack_array(): not all matrix in matrix array have the same row!";
        }
    }
    // 矩阵连接
    std::vector<Matrix> v={horizontal_stack(m[0],m[1])};
    for (int i = 0; i < n-2; ++i) {
        v.push_back(horizontal_stack(v[i],m[i+2]));
    }
    return v[n-2];
}


Matrix vertical_stack_array(Matrix *m,int n){
    // 容错处理
    for (int i = 0; i < n-1; ++i) {
        if(m[i].col != m[i+1].col && !m[i].row && !m[i+1].row){
            std::cerr<<"error in vertical_stack_array(): not all matrix in matrix array have the same col!";
        }
    }
    // 矩阵连接
    std::vector<Matrix> v={vertical_stack(m[0],m[1])};
    for (int i = 0; i < n-2; ++i) {
        v.push_back(vertical_stack(v[i],m[i+2]));
    }
    return v[n-2];
}

Matrix diag(double *p,int n){
    std::vector<double> v;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <n; ++j) {
            if(i==j){
                v.push_back(p[i]);
            }
            v.push_back(0);
        }
    }
    auto new_p = new double[n*n];
    new_p = &v[0];
    return {n,n,new_p};
}

Matrix diag(Matrix *m,int n){
    // 错误处理
    for (int i = 0; i < n; ++i) {
        if(m[i].col != m[i].row){
            std::cerr<<"error in diag(): m数组中的矩阵必须均为方阵";
        }
    }
    int col = 0;   // 总列数
    for (int i = 0; i < n; ++i) {
        col += m[i].col;
    }
    Matrix m_out = zero(col,col);
    int row = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = row; j < row + m[i].row; ++j) {
            for (int k = row; k < row + m[i].row; ++k) {
                m_out.assign(j+1,k+1,m[i](j-row+1,k-row+1));
            }
        }
        row += m[i].row;
    }
    return m_out;
}


