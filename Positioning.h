//
// Created by 0-0 mashuo on 2023/3/6.
//

#ifndef RTK_POSITIONING_H
#define RTK_POSITIONING_H

// Hopefield模型标准气象元素
#define Hopefield_H0 0
#define Hopefield_T0 288.16
#define Hopefield_p0 1013.25
#define Hopefield_RH0 0.5

#include "TimeSys.h"
#include "CoorSys.h"
#include "GnssData.h"

// GPS卫星位置、速度、钟差钟速计算
int GPS_POS(GNSSINFO *info, GPSEPH *eph, GPST time[]);

// BDS卫星位置、速度、钟差钟速计算
int BDS_POS(GNSSINFO *info, BDSEPH *eph, GPST time[]);

// GPS f1载波 对流层延迟误差改正算法   H[m] E[radians] 输出对流层延迟误差(m)
double Hopefiled(double H, double E);

// GPS f1载波 电离层延迟误差改正算法  输出电离层延迟时间改正
/* alpha 0,1,2,3 GPS navigation telegram
 * beta  0,1,2,3 GPS navigation telegram
 * BLH 用户大地坐标                 [radians]
 * E-观测卫星高度角、A-观测卫星方位角  [radians]
 * GPST 观测时间
 */
double KlobucharG(const double alpha[], const double beta[], GPST time, BLH blh, double E, double A);

// BDS BI1 电离层延迟改正算法 输出电离层延迟时间改正
/* alpha 0,1,2,3 BDS navigation telegram
 * beta  0,1,2,3 BDS navigation telegram
 * BLH 用户大地坐标                 [radians]
 * E-观测卫星高度角、A-观测卫星方位角  [radians]
 * GPST 观测时间
 */
double KlobucharB(const double alpha[], const double beta[], BDST time, BLH blh, double E, double A);

// 双频伪距无电离层观测值单点定位算法实现
int PIF_location(OBS *obs, GPSEPH *gpseph, BDSEPH *bdseph, ReciverPos *Pos, GNSSINFO *info);

// 计算卫星发射时刻
GPST calculate_ts(GPST tr, double P, double dts);

// 双频单点定位数学模型
void calculate_pos(GNSSINFO *info, double G_pif[], double C_pif[], ReciverPos *Pos);

// 单点定速基本算法
void calculate_v(GNSSINFO *info, OBS *obs, ReciverPos *Pos);

void print_info(GNSSINFO *info);



#endif //RTK_POSITIONING_H
