//
// Created by 0-0 mashuo on 2023/3/6.
//

#ifndef RTK_GNSSDATA_H
#define RTK_GNSSDATA_H

#define GPSMAXPRN 32
#define BDSMAXPEN 63
/* General constants */
#define C_Light 299792458.0      /* Speed of light  [m/s]; IAU 1976  */
#define G_fL1   1575.42e6        /* GPS L1 frequency   [hz]          */
#define G_fL2   1227.60e6        /* GPS L2 frequency   [hz]          */
#define G_fL3   1176.45e6        /* GPS L3 frequency   [hz]          */
#define B_f1I    1561.098e6       /* BDS B1-2 frequency [hz]          */
#define B_f3I    1268.52e6        /* BDS B3 frequency   [hz]          */

#define Omega_WGS 7.2921151467e-5  /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9     /* [m^3/s^2]; WGS-84 */
#define GM_JGM3   398600.4415e+9   /* [m^3/s^2]; JGM3  */
#define Omega_BDS 7.2921150e-5     /*[rad/s], the earth rotation rate */
#define GM_BDS   398600.4418e+9    /* [m^3/s^2]; CGCS2000  */

#define G_WL1  C_Light/G_fL1       /* GPS L1 WaveLength [m]         */
#define G_WL2  C_Light/G_fL2       /* GPS L2 WaveLength [m]         */
#define G_WL3  C_Light/G_fL3       /* GPS L3 WaveLength [m]         */
#define B_W1I  C_Light/B_f1I       /* GPS B1I WaveLength [m]         */
#define B_W3I  C_Light/B_f3I       /* GPS B3I WaveLength [m]         */

#define f(x,y) x*x/(x*x-y*y)

#include <cstdint>
#include "TimeSys.h"
#include "CoorSys.h"
#include "Matrix.h"
#include "setting_map.h"

struct GNSSINFO;

// 单个卫星星历结构体
struct GPSEPH
{
    uint32_t PRN;        /* Satellite PRN               */
    double tow;          /* 子帧时间戳  [s]               */
    uint32_t IODE1;
    uint32_t IODE2;
    uint32_t week;       /* GPS week                    */
    uint32_t z_week;
    double toe;          /* 卫星星历参考时刻 [s]           */
    double A;            /* 卫星轨道长半轴      [m]        */
    double detN;         /* 卫星平均角速度改正项 [rad/s]    */
    double M0;           /* 参考时刻卫星平近点角 [rad]      */
    double ecc;          /* 卫星轨道偏心率                 */
    double omega;        /* 参考时刻近地点角距             */
    double cuc;          /* 升交角距的余弦调和改正项振幅[rad]*/
    double cus;          /* 升交角距的正弦调和改正项振幅[rad]*/
    double crc;          /* 卫星到地心距离余弦改正项振幅[m]  */
    double crs;          /* 卫星到地心距离正弦改正项振幅[m]  */
    double cic;          /* 轨道倾角余弦改正项振幅[rad]     */
    double cis;          /* 轨道倾角正弦改正项振幅[rad]     */
    double I0;           /* 参考时刻轨道倾角 [rad]         */
    double det_IO;       /* 轨道倾角变化率 [rad/s]         */
    double omega_0;      /* 升交点赤经    [rad]           */
    double det_omega;    /* 升交点赤经变化率    [rad/s]    */
    uint32_t iodc;
    double toc;          /* 卫星钟差参考时刻  [s]          */
    double tgd;          /* 信号群延差       [s]          */
    double a0;           /* toc时刻钟偏参数  [s]          */
    double a1;           /* toc时刻钟漂参数               */
    double a2;           /* toc时刻频漂参数   [s-1]       */
    double N;            /* 改正后的平均角速度 [rad/s]     */
    double a[4];         // klobuchar模型的四个参数
    bool flag= false;    // 判断该卫星的星历是否存在
};

struct BDSEPH
{
    uint32_t PRN;      /* Satellite PRN               */
    uint32_t week;     /* BDS week                    */
    double tgd1;       /* B1信号群延差                  */
    double tgd2;       /* B2信号群延差                  */
    uint32_t AODC;
    uint32_t toc;      /* 卫星钟差参考时刻  [s]          */
    double a0;         /* toc时刻钟偏参数  [s]          */
    double a1;         /* toc时刻钟漂参数               */
    double a2;         /* toc时刻频漂参数   [s-1]       */
    uint32_t AODE;
    uint32_t toe;      /* 卫星星历参考时刻 [s]           */
    double RootA;      /* 卫星轨道长半轴开根号 [sqrt(m)]  */
    double ecc;        /* 卫星轨道偏心率                 */
    double omega;      /* 参考时刻近地点角距             */
    double detN;       /* 卫星平均角速度改正项 [rad/s]    */
    double M0;         /* 参考时刻卫星平近点角 [rad]      */
    double Omega0;     /* 升交点赤经    [rad]           */
    double det_Omega;  /* 升交点赤经变化率    [rad/s]    */
    double i0;         /* 参考时刻轨道倾角 [rad]         */
    double IDOT;       /* 轨道倾角变化率 [rad/s]         */
    double cuc;        /* 升交角距的余弦调和改正项振幅[rad]*/
    double cus;        /* 升交角距的正弦调和改正项振幅[rad]*/
    double crc;        /* 卫星到地心距离余弦改正项振幅[m]  */
    double crs;        /* 卫星到地心距离正弦改正项振幅[m]  */
    double cic;        /* 轨道倾角余弦改正项振幅[rad]     */
    double cis;        /* 轨道倾角正弦改正项振幅[rad]     */
    double b[4];         // klobuchar模型的四个参数
    bool flag= false;  // 判断该卫星的星历是否存在
};

// 每颗卫星观测数据，记录卫星双频观测量
struct Stalite {
    double C[2]{}; // 伪距
    double L[2]{}; // 载波
    float D[2]{}; // 多普勒
    float S[2]{}; // 信噪比

    Stalite();

    void Formatting();   // 将所有观测值置0
};

// GNSS观测数据结构体
struct OBS
{
    GPST t;      //观测时刻
    int nums=0;    //观测卫星数目
    Stalite GPS[GPSMAXPRN]; // gps卫星观测数据
    Stalite BDS[BDSMAXPEN]; // bds卫星观测数据
    double S_limit=SNR_LIMIT;      // 信噪比阈值(参数配置）
    int G_num=0,C_num=0;    // 记录可用卫星数目
    int *G_prn,*C_prn;      // 记录可用卫星PRN号
    // 计算双频消电离层组合伪距观测值
    void get_GPS_PIF(const int prn[],int num,double pif[]);
    void get_BDS_PIF(const int prn[],int num,double pif[],BDSEPH *bdseph,GNSSINFO *info);
    // 剔除伪距粗差、统计双频伪距观测值可用卫星(双频信噪比大于35的卫星可用)
    void find_error_P(GPSEPH *gpseph,BDSEPH *bdseph);
    // 判断某卫星观测值是否完整
    bool is_gps_complete_L1(int);
    bool is_gps_complete_L2(int);
    bool is_bds_complete_I1(int);
    bool is_bds_complete_I3(int);
    // 判断载波信噪比是否超限
    bool is_SNR_over_bds_I1(int);
    bool is_SNR_over_bds_I3(int);
    bool is_SNR_over_gps_L1(int);
    bool is_SNR_over_gps_L2(int);
    // 释放空间
    void release();
    // 格式化观测值结构体
    void Formatting();
};

// 二进制头文件结构体
struct HeaderB
{
    uint8_t len;   // 文件头长度
    uint16_t MsgId; // 数据ID
    uint16_t MsgLen; //数据体长度
    GPST t;
};

// 卫星信息结构体
struct SatelliteINFO
{
    XYZ Coordinate;        // m
    double Vx,Vy,Vz;       // m/s
    double clock_error;    // s 目前未改正dtr、tgd,后续可加入
    double v_clock_error;  // s/s 加入了rate_dtr
    double E=0,A=0;            // 高度角、方位角 [rad]
    bool flag= false;      // 位置是否有效
    SatelliteINFO();
};

struct GNSSINFO
{
    SatelliteINFO GPS[GPSMAXPRN];
    SatelliteINFO BDS[BDSMAXPEN];
    int G_num=0,C_num=0;         // 记录已知卫星位置的卫星数目
    int *G_prn,*C_prn;           // 记录已知卫星位置的prn
    double E_limit=rad(ELEVATION_LIMIT);   // 卫星高度角限制10deg（参数配置）

    // 根据观测时刻计算卫星发射时刻的卫星位置、卫星钟差(定义见Positioning.h)
    void calculate_tr(OBS *obs,GPSEPH *gpseph,BDSEPH *bdseph);
    // 根据接收机时钟钟差  求出地球自转改正
    void calculate_rotation(double G_dtr,double C_dtr,const double G_pif[],const double C_pif[]);

    // 得到测站观测到且已知位置、且高度角大于10°的卫星数目,并给出PRN号
    void getGPS(double limit);

    void getBDS(double limit);

    // 根据测站坐标计算卫星高度角、方位角
    void calculate_EA(XYZ cor);

    // 释放开辟的动态内存
    void release();

};

// 定位结果结构体
struct ReciverPos
{
    GPST t;          //观测时间
    XYZ Coordinate;  //xyz(m)  ECEF
    ENU enu;         //相对于参考坐标的站心坐标
    double dis=0;    //定位误差m
    double Vx=0,Vy=0,Vz=0; //速度(m/s)
    double PDOP=0;
    double sigma0=0;  //中误差
    int G_num=0,C_num=0;
    double G_clk=0,C_clk=0;
    Matrix Qxx;       //协因数阵
    double v_sigma0=0; // 速度参数的中误差
    Matrix v_Qxx;      // 速度参数的协因数阵
    bool known = false;

    void print();

    void calculate_enu(XYZ xyz);

};



#endif //RTK_GNSSDATA_H
