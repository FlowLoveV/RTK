//
// Created by 0-0 mashuo on 2023/3/6.
//

#include <cmath>
#include <iostream>
#include <iomanip>
#include "CoorSys.h"


// deg转rad函数
double rad(const double deg) {
    return deg / 180.0 * pi;
}

// rad转deg函数
double deg(const double rad) {
    return rad / pi * 180;
}

// 度分秒转deg函数
double DMS2deg(const double deg,const double minute,const double second) {
    return deg + minute/60.0 + second/3600.0;
}

XYZ::XYZ() : X(R_WGS84), Y(0), Z(0) {}

XYZ::XYZ(const double *p) {
    X = *p;
    Y = *(p + 1);
    Z = *(p + 2);
}

XYZ::XYZ(const double X, const double Y, const double Z) {
    this->X = X;
    this->Y = Y;
    this->Z = Z;
}

XYZ &XYZ::operator=(const XYZ &b) {
    if (this == &b) return *this;
    this->X = b.X;
    this->Y = b.Y;
    this->Z = b.Z;
    return *this;
}

double Distance(XYZ xyz1, XYZ xyz2) {
    double dx=xyz1.X-xyz2.X;
    double dy=xyz1.Y-xyz2.Y;
    double dz=xyz1.Z-xyz2.Z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

void XYZ::print() {
    std::cout << "XYZ:" << std::setprecision(16) << X << "    " << Y << "    " << Z << '\n';
}

BLH::BLH() : lat(0), lon(0), H(0) {}

BLH::BLH(const double *p)  {
    lat = *p;
    lon = *(p + 1);
    H = *(p + 2);
}

BLH::BLH(const double B, const double L, const double H) {
    lat = B;
    lon = L;
    this->H = H;
}

BLH &BLH::operator=(const BLH &b) {
    if (this == &b) return *this;
    this->lat = b.lat;
    this->lon = b.lon;
    this->H = b.H;
    return *this;
}

void BLH::print() {
    std::cout << "BLH:" << std::setprecision(16) << lat / pi * 180 << "    " << lon / pi * 180 << "    " << H << '\n';
}

ENU::ENU() :E(0),N(0),U(0) {}

ENU::ENU(const double *p) {
    E=*(p);
    N=*(p+1);
    U=*(p+2);
}

ENU::ENU(const double e, const double n, const double u) {
    E=e;
    N=n;
    U=u;
}

ENU &ENU::operator=(const ENU &b) {
    if(this == &b) return *this;
    E=b.E;
    N=b.N;
    U=b.U;
    return *this;
}

double Distance(ENU c1, ENU c2) {
    double de=c1.E-c2.E;
    double dn=c1.N-c2.N;
    double du=c1.U-c2.U;
    return sqrt(de*de + dn*dn + du*du);
}

void ENU::print() {
    printf("%4.3f %4.3f %4.3f\n",E,N,U);
}

GAUSS::GAUSS() : x(0), y(0), lon(rad(114)), h(0) {}

GAUSS::GAUSS(const double *p) { x=*(p); y=*(p+1); h=*(p+2); lon=*(p+3);}

GAUSS::GAUSS(double X, double Y, double H, double L) : x(X),y(Y),lon(L),h(H) {}

void GAUSS::print() {printf("%3.2f %8.8f %8.8f %4.5f\n",deg(lon),x,y,h);}

// BLH->XYZ
XYZ BLH2XYZ(const BLH cor, const std::string &str) {
    double a, f;
    if (str == "WGS84") {
        a = R_WGS84;
        f = F_WGS84;
    } else if (str == "CGCS2000") {
        a = R_CGS2K;
        f = F_CGS2K;
    } else {
        std::cerr << "没有这种坐标框架!";
    }
    double e2 = f * (2 - f);
    double N = a / sqrt(1 - e2 * sin(cor.lat) * sin(cor.lat));
    double X = (N + cor.H) * cos(cor.lat) * cos(cor.lon);
    double Y = (N + cor.H) * cos(cor.lat) * sin(cor.lon);
    double Z = (N * (1 - e2) + cor.H) * sin(cor.lat);
    XYZ corout(X, Y, Z);
    return corout;
}

// XYZ->BLH
BLH XYZ2BLH(const XYZ cor, const std::string &str) {
    double a, f;
    if (str == "WGS84") {
        a = R_WGS84;
        f = F_WGS84;
    } else if(str == "CGCS2000"){
        a = R_CGS2K;
        f = F_CGS2K;
    }
    else {std::cerr<<"没有这种坐标框架！";BLH cor;return cor;}
    int Iterator;
    double e2, dZ, rho2, dZ_new, SinPhi, B, L, H;
    double ZdZ, Nh, N;
    N = 0.0;
    e2 = f * (2.0 - f);
    rho2 = cor.X * cor.X + cor.Y * cor.Y;
    dZ = e2 * cor.Z;
    dZ_new = dZ;
    Iterator = 0;
    do {
        dZ = dZ_new;
        ZdZ = cor.Z + dZ;
        Nh = sqrt(rho2 + ZdZ * ZdZ);
        if (Nh < 1.0)
        {
            B = L = 0.0;
            H = -a;
            break;
        }
        SinPhi = ZdZ / Nh;
        N = a / sqrt(1.0 - e2 * SinPhi * SinPhi);
        dZ_new = N * e2 * SinPhi;
        Iterator = Iterator + 1;
    } while ((fabs(dZ - dZ_new) > 1E-8) && (Iterator < 10));
    L = atan2(cor.Y, cor.X);
    B = atan2(ZdZ, sqrt(rho2));
    H = Nh - N;
    BLH corout(B, L, H);
    return corout;
}

// XYZ->ENU
ENU XYZ2ENU(const XYZ xyz0,const XYZ xyz,const std::string &str)
{
    BLH blh= XYZ2BLH(xyz0,str);
    double Cen[9];
    double sinp=sin(blh.lat),cosp=cos(blh.lat),sinl=sin(blh.lon),cosl=cos(blh.lon);
    Cen[0]=-sinl,  Cen[1]=cosl,  Cen[2]=0.0,
    Cen[3]=-sinp*cosl,Cen[4]=-sinp*sinl,Cen[5]=cosp,
    Cen[6]=cosp*cosl,Cen[7]=cosp*sinl,Cen[8]=sinp;
    Matrix C(3,3,Cen);
    double r[3];
    r[0]=xyz.X-xyz0.X,r[1]=xyz.Y-xyz0.Y,r[2]=xyz.Z-xyz0.Z;
    Matrix dr(3,1,r);
    Matrix enu=C*dr;
    ENU enu_out(enu(1,1),enu(2,1),enu(3,1));
    return enu_out;
}

// 大地坐标转高斯投影坐标
GAUSS BLH2GAUSS(BLH blh, double lon, const std::string &type) {
    // 首先选择坐标框架
    double a=0, f=0;
    if (type == "WGS84") {
        a = R_WGS84;
        f = F_WGS84;
    } else if (type == "CGCS2000") {
        a = R_CGS2K;
        f = F_CGS2K;
    } else {
        std::cerr << "没有这种坐标框架!";
    }
    double B = blh.lat, l = blh.lon - rad(lon);
    double sinB = sin(B), cosB = cos(B), cosB3 = cosB * cosB * cosB, cosB5 = cosB3 * cosB * cosB;
    double b = a * (1 - f);
    double e2 = f*(2-f), e4 = e2 * e2, e6 = e4 * e2, e8 = e6 * e2, e10 = e8 * e2;
    double e_dot2 = (a*a-b*b)/(b*b);
    double t = tan(B), t2 = t * t, t4 = t2 * t2;
    double eta2 = e_dot2 * cosB * cosB,  eta4 = eta2 * eta2;
    double N = a / sqrt(1 - e2 * sinB * sinB);

    double A1 = 1, A2 = 3.0 / 4.0, A3 = 45.0 / 64.0, A4 = 175.0 / 256.0, A5 = 11025.0 / 16384.0, A6 = 43659.0 / 65536.0;
    double xA = A1 + A2 * e2 + A3 * e4 + A4 * e6 + A5 * e8 + A6 * e10;
    double B2 = 3.0 / 4.0, B3 = 15.0 / 16.0, B4 = 525.0 / 512.0, B5 = 2205.0 / 2048.0, B6 = 72756.0 / 65536.0;
    double xB = B2 * e2 + B3 * e4 + B4 * e6 + B5 * e8 + B6 * e10;
    double C3 = 15.0 / 64.0, C4 = 105.0 / 256.0, C5 = 2205.0 / 4096.0, C6 = 10395.0 / 16384.0;
    double xC = C3 * e4 + C4 * e6 + C5 * e8 + C6 * e10;
    double D4 = 35.0 / 512.0, D5 = 315.0 / 2048.0, D6 = 31185.0 / 131072.0;
    double xD = D4 * e6 + D5 * e8 + D6 * e10;
    double E5 = 315.0 / 16384.0, E6 = 3645.0 / 65536.0;
    double xE = E5 * e8 + E6 * e10;

    double X0 = a * (1 - e2) *
                (xA * B - xB / 2 * sin(2 * B) + xC / 4 * sin(4 * B) - xD / 6 * sin(6 * B) + xE / 8 * sin(8 * B));

    double x = X0 + N / 2 * sinB * cosB * l * l + N / 24 * sinB * cosB3 * (5 - t2 + 9 * eta2 + 4 * eta4) * pow(l,4)
               + N / 720 * sinB * cosB5 * (61 - 58 * t2 + t4) * pow(l, 6);

    double y = N * cosB * l + N / 6 * cosB3 * (1 - t2 + eta2) * pow(l, 3) + N / 120 * cosB5
                                                                            * (5 - 18 * t2 + t4 + 14 * eta2 - 58 * eta2 * t2) * pow(l, 5);
    GAUSS gauss(x,y,blh.H,rad(lon));
    return gauss;
}

Matrix LOSVector(XYZ Satellite,XYZ Receiver){
    double dis = Distance(Satellite,Receiver);
    std::vector<double> v(3);
    v[0] = Satellite.X - Receiver.X;
    v[1] = Satellite.Y - Receiver.Y;
    v[2] = Satellite.Z - Receiver.Z;
    return {3,1,v};
}

