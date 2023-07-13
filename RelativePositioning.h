//
// Created by 0-0 mashuo on 2023/3/8.
//

#ifndef RTK_RELATIVEPOSITIONING_H
#define RTK_RELATIVEPOSITIONING_H

#include "GnssData.h"
#include "vector"
#include "Solve.h"


class Station;
class RelativeStation;


// RTK全局配置变量
extern uint8_t V_FUNC;
extern uint8_t P_FUNC;
extern uint8_t H_FUNC;
extern uint8_t SC_NUM;  // 载波数目
extern uint8_t SYS_NUM; // 参与解算GNSS系统数目

// 测站定义
class Station{
public:
    OBS obs;        // 观测数据                  /* 这里的数据并没有采用指针的方式，因为
    GPSEPH gps_eph; // GPS卫星星历                  在RTK中，我们只是使用测站的数据，不
    BDSEPH bds_eph; // BDS卫星星历                  和单点定位一样需要改变这些数据的信息。
    GNSSINFO info;  // 卫星结构信息                  在不需要改变这些信息的情况下，这种做法
    ReciverPos pos; // 测站单点定位结果               更方便且安全                     */

    bool gps_complete_obs[GPSMAXPRN] = {false};
    bool bds_complete_obs[BDSMAXPEN] = {false};
    std::vector<int> gps_complete_prn;
    std::vector<int> bds_complete_prn;
    int gps_complete_num = 0;
    int bds_complete_num = 0;
    // 构造函数
    Station(OBS p_obs,GPSEPH p_gps_eph,BDSEPH p_bds_eph,GNSSINFO p_info,ReciverPos p_pos);
    Station();
    // 统计双频四观测可用卫星数目及PRN号
    int get_gps_complete();
    int get_bds_complete();
    // 获得原始观测值矩阵
    Matrix get_gps_obs() const;
    Matrix get_bds_obs() const;
    // 获得组合观测值
    Matrix GL_GPS(const Matrix&) const;
    Matrix GL_BDS(const Matrix&) const;

};

// 相对定位测站类
class RelativeStation{

public:
    // 输入两站信息
    Station base;
    Station mobile;

    // RTK中间量
    bool gps_public_view[GPSMAXPRN] = {false};    // GPS共视卫星
    bool bds_public_view[BDSMAXPEN] = {false};    // BDS共视卫星
    int gps_view_num=0;                           // gps共视卫星数目
    int bds_view_num=0;                           // bds共视卫星数目
    std::vector<int> gps_view_prn;                // gps共视卫星prn号
    std::vector<int> bds_view_prn;                // bds共视卫星prn号
    short gps_reference_star = 0;                 // gps双差参考卫星序号(按prn号排列的序号)
    short bds_reference_star = 0;                 // bds双差参考卫星序号
    std::vector<double> gps_base_rho0;            // 基站和参考GPS卫星间的距离（SPP求得的近似距离）
    std::vector<double> bds_base_rho0;            // 基站和参考BDS卫星间的距离（SPP求得的近似距离）
    std::vector<double> gps_mobile_rho0;          // 移动站站和共视GPS卫星间的距离（SPP求得的近似距离）
    std::vector<double> bds_mobile_rho0;          // 移动站站和共视BDS卫星间的距离（SPP求得的近似距离）

    // 基线解相关
    double baseline_xyz[3] = {0};                  // 基线x,y,z分量
    double baseline_neu[3] = {0};                  // 基线n,e,u分量
    double ratio = 0;                              // 模糊度固定ratio值
    double sigma2 = 0;                             // 单位权中误差
    double m_xyz[3] = {0};                         // 基线x,y,z分量中误差
    double m_neu[3] = {0};                         // 基线e,n,u方向分量中误差
    double S_length = 0;                           // 基线长
    double m_S = 0;                                // 基线长中误差
    double RDOP = 0;                               // 基线DOP值
    double RMS = 0;                                // 观测值残差
    bool fixed = false;                            // 解是否固定


    RelativeStation(Station Base,Station Mobile);
    // 共视卫星选取并统计数目
    void get_public_view();
    // 基准星选取
    int select_bds_reference_satellite();
    int select_gps_reference_satellite();

    // 计算基准战到GPS、BDS各共视卫星的近似距离
    void get_base_rho0();

    // 双差权阵构建
    Matrix Build_D_P();
    Matrix Build_D_P_GPSL1_L();
    Matrix Build_D_P_GPSL2_L();
    Matrix Build_D_P_BDSBI1_L();
    Matrix Build_D_P_BDSBI3_L();

    // 最终H矩阵构建
    Matrix BuildFinalH();
    // 最小H矩阵构建，也即B矩阵构建(同时求得了移动站到GPS、BDS各共视卫星的距离)  因此，求解时，要先构建H矩阵，然后构建V矩阵
    Matrix BuildGPSMinH();
    Matrix BuildBDSMinH();

    // 单差残差向量构建（均按照PRN号从小到大排列）
    Matrix BuildSingleResidual();
    Matrix Build_S_V_GPSL1_C();
    Matrix Build_S_V_GPSL1_L();
    Matrix Build_S_V_GPSL2_C();
    Matrix Build_S_V_GPSL2_L();
    Matrix Build_S_V_BDSBI1_C();
    Matrix Build_S_V_BDSBI1_L();
    Matrix Build_S_V_BDSBI3_C();
    Matrix Build_S_V_BDSBI3_L();

    // 双差残差向量构建（均按照PRN号从小到大排列）
    Matrix BuildDoubleResidual();
    Matrix Build_D_V_GPSL1_C();
    Matrix Build_D_V_GPSL1_L();
    Matrix Build_D_V_GPSL2_C();
    Matrix Build_D_V_GPSL2_L();
    Matrix Build_D_V_BDSBI1_C();
    Matrix Build_D_V_BDSBI1_L();
    Matrix Build_D_V_BDSBI3_C();
    Matrix Build_D_V_BDSBI3_L();

    // 最小二乘法RTK
    bool LS();

    // 精度评定函数
    void accuracy_evaluation(double sig2,Matrix Qbb,Matrix V);

    // 将精度评定结果打印出来
    void print_accuracy_evaluation();

};


// 基准战、流动站时间对齐函数
bool TimeAlignment(GPST t1,GPST t2);

// 根据高度角生成权值
double GetWeightByElevation(double E);

// 根据信噪比生成权值
double GetWeightBySNR(double SNR);

// 周跳检测
bool is_slip(double,double);

// 解算标识
/* H_FUNC   bit0-GPS:{0=无，1=有}    bit1-BDS:{0=无，1=有}
 * P_FUNC   bit0-GL1  bit1-GL2  bit2-BL1  bit3-BL3
 * V_FUNC   bit0-GL1  bit1-GL2  bit2-BL1  bit3-BL3
 * 这里的解算默认为伪距载波一起参与解算
 */
// 进行RTK配置
void dispose_RTK();

// 输出RTK结果






//

#endif //RTK_RELATIVEPOSITIONING_H
