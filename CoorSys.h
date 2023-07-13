//
// Created by 0-0 mashuo on 2023/3/6.
//

#ifndef RTK_COORSYS_H
#define RTK_COORSYS_H

// 常量定义
#define R_WGS84  6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84   */
#define R_CGS2K  6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K  1.0/298.257222101  /* Flattening; CGCS2000   */
#define pi 3.1415926535897932384626433832795028841971

#include <iostream>
#include "Matrix.h"

double rad(double deg);

double deg(double rad);

double DMS2deg(double deg,double minute,double second);

struct XYZ {
    double X;
    double Y;
    double Z;

    // 构造函数
    XYZ() ;

    XYZ(const double *p) ;

    XYZ(double X, double Y, double Z) ;

    XYZ &operator=(const XYZ &b) ;



    void print() ;
};

struct BLH {
    // rad
    double lat;
    double lon;
    double H;

    // 构造函数
    BLH() ;

    BLH(const double *p);

    BLH(const double B, const double L, const double H) ;

    BLH &operator=(const BLH &b) ;

    void print() ;
};

struct ENU
{
    double E;
    double N;
    double U;

    // 构造函数
    ENU();
    ENU(const double *p);

    ENU(const double e,const double n,const double u);

    // 赋值运算符重载
    ENU& operator=(const ENU &b);


    // 打印
    void print();
};

struct GAUSS
{
    double x,y;     // 高斯平面坐标                  [m]
    double lon;     // 高斯投影带的中央子午线大地经度   [rad]
    double h;

    // 构造函数
    GAUSS();
    GAUSS(const double *p) ;
    GAUSS(double X, double Y, double H,double L) ;

    // 打印
    void print()  ;

};

// 求距离函数

double Distance(XYZ xyz1,XYZ xyz2);

double Distance(ENU c1,ENU c2);

// BLH->XYZ
XYZ BLH2XYZ(const BLH cor, const std::string &str);

// XYZ->BLH
BLH XYZ2BLH(const XYZ cor, const std::string &str);

// XYZ->ENU
ENU XYZ2ENU(const XYZ xyz0,const XYZ xyz,const std::string &str);

// 大地坐标转高斯投影坐标
GAUSS BLH2GAUSS(BLH blh, double lon,const std::string& type);

// 求出卫星和测站之间视线向量
/* Input : Satellite Coordinate , Receiver Coordinate
 * Output: Line of sight Matrix (3,1)
 */
Matrix LOSVector(XYZ Satellite,XYZ Receiver);


#endif //RTK_COORSYS_H
