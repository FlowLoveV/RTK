//
// Created by 0-0 mashuo on 2023/3/8.
//

#include "RelativePositioning.h"
#include <cmath>
#include "setting_map.h"
#include <utility>
#include "CoorSys.h"
#include "BitOperation.h"
#include "lambda.h"
#include "ostream"

Station::Station(OBS p_obs, GPSEPH p_gps_eph, BDSEPH p_bds_eph, GNSSINFO p_info, ReciverPos p_pos) {
    obs=p_obs;  gps_eph=p_gps_eph;  bds_eph=p_bds_eph;  info=p_info;  pos=std::move(p_pos);
}


int Station::get_gps_complete() {
    int num = 0;
    for (int i = 0; i < GPSMAXPRN; ++i) {
        gps_complete_obs[i] = obs.is_gps_complete_L1(i) && obs.is_gps_complete_L2(i);
        if (gps_complete_obs[i]){
            gps_complete_prn.push_back(i);
            num += 1;
        }
    }
    gps_complete_num = num;
    return num;
}

int Station::get_bds_complete() {
    int num = 0;
    for (int i = 0; i < BDSMAXPEN; ++i) {
        bds_complete_obs[i] = obs.is_bds_complete_I1(i) && obs.is_bds_complete_I3(i);
        if (bds_complete_obs[i]){
            bds_complete_prn.push_back(i);
            num += 1;
        }
    }
    bds_complete_num = 0;
    return num;
}

Matrix Station::get_gps_obs() const {
    std::vector<double> v(gps_complete_num*8,0);
    Matrix m(gps_complete_num,8,v);
    for (int i = 1; i < gps_complete_num+1; ++i) {
        int index = gps_complete_prn[i];
        m.assign(i,1,obs.GPS[index].C[0]);
        m.assign(i,2,obs.GPS[index].C[1]);
        m.assign(i,3,obs.GPS[index].L[0]);
        m.assign(i,4,obs.GPS[index].L[1]);
        m.assign(i,5,obs.GPS[index].D[0]);
        m.assign(i,6,obs.GPS[index].D[1]);
        m.assign(i,7,obs.GPS[index].S[0]);
        m.assign(i,8,obs.GPS[index].S[1]);
    }
    return m;
}

Matrix Station::get_bds_obs() const {
    std::vector<double> v(bds_complete_num*8);
    Matrix m(bds_complete_num,8,v);
    for (int i = 1; i < bds_complete_num+1; ++i) {
        int index = bds_complete_prn[i];
        m.assign(i,1,obs.BDS[index].C[0]);
        m.assign(i,2,obs.BDS[index].C[1]);
        m.assign(i,3,obs.BDS[index].L[0]);
        m.assign(i,4,obs.BDS[index].L[1]);
        m.assign(i,5,obs.BDS[index].D[0]);
        m.assign(i,6,obs.BDS[index].D[1]);
        m.assign(i,7,obs.BDS[index].S[0]);
        m.assign(i,8,obs.BDS[index].S[1]);
    }
    return m;
}

Matrix Station::GL_GPS(const Matrix & m) const {
    std::vector<double> v(gps_complete_num);
    Matrix GL(gps_complete_num,1,v);
    for (int i = 1; i < gps_complete_num+1; ++i) {
        double gl = G_WL1 * m(i,3) - G_WL2 * m(i,4);
        GL.assign(i,1,gl);
    }
    return GL;
}

Matrix Station::GL_BDS(const Matrix & m) const {
    std::vector<double> v(bds_complete_num);
    Matrix GL(bds_complete_num,1,v);
    for (int i = 1; i < bds_complete_num+1; ++i) {
        double gl = B_W1I * m(i,3) - B_W3I * m(i,4);
        GL.assign(i,1,gl);
    }
    return GL;
}

Station::Station() = default;


bool TimeAlignment(GPST t1,GPST t2) {
    if(abs(t1-t2) < 0.001) return true;
    else return false;
}

double GetWeightByElevation(double E) {
    double sinE = sin(E);
    return 0.004*0.004+0.003*0.003/(sinE*sinE);
}

double GetWeightBySNR(double SNR) {
    return 0.0164*pow(10,-SNR/10);
}

void RelativeStation::get_public_view() {
    bool b[4] = {(V_FUNC >> 0 & 1) == 0,(V_FUNC >> 1 & 1) == 0,(V_FUNC >> 2 & 1) == 0,(V_FUNC >> 3 & 1) == 0};
    // 统计GPS共视卫星
    for (int i = 0; i < GPSMAXPRN; ++i) {
        // 双频观测是否完整判断
        bool GL1 = b[0] || (base.obs.is_gps_complete_L1(i) && mobile.obs.is_gps_complete_L1(i));
        bool GL2 = b[1] || (base.obs.is_gps_complete_L2(i) && mobile.obs.is_gps_complete_L2(i));
        // 卫星高度角判断
        if(base.info.GPS[i].flag && mobile.info.GPS[i].flag && GL1 && GL2
          && base.info.GPS[i].E>=base.info.E_limit
          && mobile.info.GPS[i].E>=base.info.E_limit)
        {gps_public_view[i]= true;  gps_view_prn.push_back(i);}
        else gps_public_view[i]= false;
    }
    // 统计BDS共视卫星
    for (int i = 0; i < BDSMAXPEN; ++i) {
        bool CBI1 = b[2] || (base.obs.is_bds_complete_I1(i) && mobile.obs.is_bds_complete_I1(i));
        bool CBI3 = b[3] || (base.obs.is_bds_complete_I3(i) && mobile.obs.is_bds_complete_I3(i));
        // 卫星高度角判断
        if(base.info.BDS[i].flag && mobile.info.BDS[i].flag && CBI1 && CBI3
          && base.info.BDS[i].E>=base.info.E_limit
          && mobile.info.BDS[i].E>=base.info.E_limit)
        {bds_public_view[i]= true; bds_view_prn.push_back(i);}
        else bds_public_view[i]= false;
    }
    gps_view_num = (H_FUNC >> 0 & 1) ? gps_view_prn.size() : 0;
    bds_view_num = (H_FUNC >> 1 & 1) ? bds_view_prn.size() : 0;
}

int RelativeStation::select_bds_reference_satellite() {
    double maxE=0;
    short maxIndex=0;
    for (int i = 0; i < bds_view_num; ++i) {
        // 避免选取GEO卫星
        int index = bds_view_prn[i];
        if(index == 0 || index == 1 || index == 2 || index == 3 || index == 4 || index == 58 || index == 59 || index == 60)    continue;
        double E = base.info.BDS[index].E + mobile.info.BDS[index].E;
        if (maxE < E) { maxE=E; maxIndex = i;}
    }
    bds_reference_star = maxIndex;
    return maxIndex;
}

int RelativeStation::select_gps_reference_satellite() {
    // 根据高度角选取参考卫星
    double maxE=0;
    short maxIndex=0;
    for (int i = 0; i < gps_view_num; ++i) {
        int index = gps_view_prn[i];
        double E = base.info.GPS[index].E + mobile.info.GPS[index].E;
        if (maxE < E) { maxE=E; maxIndex = i;}
    }
    gps_reference_star = maxIndex;
    return maxIndex;
}

RelativeStation::RelativeStation(Station Base, Station Mobile) {
    base=std::move(Base);
    mobile=std::move(Mobile);
}

Matrix RelativeStation::BuildSingleResidual() {
    return Matrix();
}

Matrix RelativeStation::BuildDoubleResidual() {
    Matrix DV_matrix[SC_NUM*2];  // 因为每个波段都用到载波伪距两种观测值
    int i = -1;
    // GL1
    if(GET_BIT(V_FUNC,0)){
        i += 1;
        DV_matrix[i] = Build_D_V_GPSL1_L();
        DV_matrix[i+SC_NUM] = Build_D_V_GPSL1_C();
    }
    // GL2
    if(GET_BIT(V_FUNC,1)){
        i += 1;
        DV_matrix[i] = Build_D_V_GPSL2_L();
        DV_matrix[i+SC_NUM] = Build_D_V_GPSL2_C();
    }
    // CL1
    if(GET_BIT(V_FUNC,2)){
        i += 1;
        DV_matrix[i] = Build_D_V_BDSBI1_L();
        DV_matrix[i+SC_NUM] = Build_D_V_BDSBI1_C();
    }
    // CL3
    if(GET_BIT(V_FUNC,3)){
        i += 1;
        DV_matrix[i] = Build_D_V_BDSBI3_L();
        DV_matrix[i+SC_NUM] = Build_D_V_BDSBI3_C();
    }
    // 将观测值矩阵组合起来
    Matrix D_V= vertical_stack_array(DV_matrix,SC_NUM*2);
    return D_V;
}

Matrix RelativeStation::Build_S_V_GPSL1_C() {
    double p[gps_view_num];
    for (int i = 0; i < gps_view_num; ++i) {
        int index = gps_view_prn[i];
        *(p+i) = mobile.obs.GPS[index].C[0] - base.obs.GPS[index].C[0];
        // 单差方程线性化
        *(p+i) += (gps_base_rho0[i] - gps_mobile_rho0[i]);
    }
    return {gps_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_GPSL1_L() {
    double p[gps_view_num];
    for (int i = 0; i < gps_view_num; ++i) {
        int index = gps_view_prn[i];
        *(p+i) = mobile.obs.GPS[index].L[0] - base.obs.GPS[index].L[0];
        *(p+i) *= G_WL1;    // 载波乘以波长
        // 单差方程线性化
        *(p+i) += (gps_base_rho0[i] - gps_mobile_rho0[i]);
    }
    return {gps_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_GPSL2_C() {
    double p[gps_view_num];
    for (int i = 0; i < gps_view_num; ++i) {
        int index = gps_view_prn[i];
        *(p+i) = mobile.obs.GPS[index].C[1] - base.obs.GPS[index].C[1];
        // 单差方程线性化
        *(p+i) += (gps_base_rho0[i] - gps_mobile_rho0[i]);
    }
    return {gps_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_GPSL2_L() {
    double p[gps_view_num];
    for (int i = 0; i < gps_view_num; ++i) {
        int index = gps_view_prn[i];
        *(p+i) = mobile.obs.GPS[index].L[1] - base.obs.GPS[index].L[1];
        *(p+i) *= G_WL2;    // 载波乘以波长
        // 单差方程线性化
        *(p+i) += (gps_base_rho0[i] - gps_mobile_rho0[i]);
    }
    return {gps_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_BDSBI1_C() {
    double p[bds_view_num];
    for (int i = 0; i < bds_view_num; ++i) {
        int index = bds_view_prn[i];
        *(p+i) = mobile.obs.BDS[index].C[0] - base.obs.BDS[index].C[0];
        // 单差方程线性化
        *(p+i) += (bds_base_rho0[i] - bds_mobile_rho0[i]);
    }
    return {bds_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_BDSBI1_L() {
    double p[bds_view_num];
    for (int i = 0; i < bds_view_num; ++i) {
        int index = bds_view_prn[i];
        *(p+i) = mobile.obs.BDS[index].L[0] - base.obs.BDS[index].L[0];
        *(p+i) *= B_W1I;    // 载波乘以波长
        // 单差方程线性化
        *(p+i) += (bds_base_rho0[i] - bds_mobile_rho0[i]);
    }
    return {bds_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_BDSBI3_C() {
    double p[bds_view_num];
    for (int i = 0; i < bds_view_num; ++i) {
        int index = bds_view_prn[i];
        *(p+i) = mobile.obs.BDS[index].C[1] - base.obs.BDS[index].C[1];
        // 单差方程线性化
        *(p+i) += (bds_base_rho0[i] - bds_mobile_rho0[i]);
    }
    return {bds_view_num,1,p};
}

Matrix RelativeStation::Build_S_V_BDSBI3_L() {
    double p[bds_view_num];
    for (int i = 0; i < bds_view_num; ++i) {
        int index = bds_view_prn[i];
        *(p+i) = mobile.obs.BDS[index].L[1] - base.obs.BDS[index].L[1];
        *(p+i) *= B_W3I;    // 载波乘以波长
        // 单差方程线性化
        *(p+i) += (bds_base_rho0[i] - bds_mobile_rho0[i]);
    }
    return {bds_view_num,1,p};
}

Matrix RelativeStation::Build_D_V_GPSL1_C() {
    if(gps_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v=this->Build_S_V_GPSL1_C();
    for (int i = 0; i < gps_view_num; ++i) {
        if(i != gps_reference_star){
            v.push_back(single_v(i+1,1)-single_v(gps_reference_star+1,1));
        }
    }
    return {gps_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_GPSL1_L() {
    if(gps_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v=this->Build_S_V_GPSL1_L();
    for (int i = 0; i < gps_view_num; ++i) {
        if(i != gps_reference_star){
            v.push_back(single_v(i+1,1)-single_v(gps_reference_star+1,1));
        }
    }
    return {gps_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_GPSL2_C() {
    if(gps_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v=this->Build_S_V_GPSL2_C();
    for (int i = 0; i < gps_view_num; ++i) {
        if(i != gps_reference_star){
            v.push_back(single_v(i+1,1)-single_v(gps_reference_star+1,1));
        }
    }
    return {gps_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_GPSL2_L() {
    if(gps_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v=this->Build_S_V_GPSL2_L();
    for (int i = 0; i < gps_view_num; ++i) {
        if(i != gps_reference_star){
            v.push_back(single_v(i+1,1)-single_v(gps_reference_star+1,1));
        }
    }
    return {gps_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_BDSBI1_C() {
    if(bds_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v = this->Build_S_V_BDSBI1_C();
    for (int i = 0; i < bds_view_num; ++i) {
        if(i != bds_reference_star){
            v.push_back(single_v(i+1,1)-single_v(bds_reference_star+1,1));
        }
    }
    return {bds_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_BDSBI1_L() {
    if(bds_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v = this->Build_S_V_BDSBI1_L();
    for (int i = 0; i < bds_view_num; ++i) {
        if(i != bds_reference_star){
            v.push_back(single_v(i+1,1)-single_v(bds_reference_star+1,1));
        }
    }
    return {bds_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_BDSBI3_C() {
    if(bds_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v = this->Build_S_V_BDSBI3_C();
    for (int i = 0; i < bds_view_num; ++i) {
        if(i != bds_reference_star){
            v.push_back(single_v(i+1,1)-single_v(bds_reference_star+1,1));
        }
    }
    return {bds_view_num-1,1,v};
}

Matrix RelativeStation::Build_D_V_BDSBI3_L() {
    if(bds_view_num<2) return Matrix();
    std::vector<double> v;
    Matrix single_v = this->Build_S_V_BDSBI3_L();
    for (int i = 0; i < bds_view_num; ++i) {
        if(i != bds_reference_star){
            v.push_back(single_v(i+1,1)-single_v(bds_reference_star+1,1));
        }
    }
    return {bds_view_num-1,1,v};
}

Matrix RelativeStation::BuildGPSMinH() {
    if(gps_view_num<2) return Matrix();
    std::vector<double> v;
    int index_satellite_prn = gps_view_prn[gps_reference_star];
    double dX1 = mobile.info.GPS[index_satellite_prn].Coordinate.X - mobile.pos.Coordinate.X;
    double dY1 = mobile.info.GPS[index_satellite_prn].Coordinate.Y - mobile.pos.Coordinate.Y;
    double dZ1 = mobile.info.GPS[index_satellite_prn].Coordinate.Z - mobile.pos.Coordinate.Z;
    double dis1 = sqrt(dX1*dX1 + dY1*dY1 + dZ1*dZ1);
    double detX1=dX1/dis1,detY1=dY1/dis1,detZ1=dZ1/dis1;
    for (int i = 0; i < gps_view_num; ++i) {
        if(i!=gps_reference_star){
            double dx = -mobile.info.GPS[gps_view_prn[i]].Coordinate.X + mobile.pos.Coordinate.X;
            double dy = -mobile.info.GPS[gps_view_prn[i]].Coordinate.Y + mobile.pos.Coordinate.Y;
            double dz = -mobile.info.GPS[gps_view_prn[i]].Coordinate.Z + mobile.pos.Coordinate.Z;
            double dis = sqrt(dx*dx + dy*dy + dz*dz);
            dx=dx/dis,dy=dy/dis,dz=dz/dis;
            double alpha_x = dx+detX1,alpha_y=dy+detY1,alpha_z=dz+detZ1;
            v.push_back(alpha_x);
            v.push_back(alpha_y);
            v.push_back(alpha_z);
            // 存储GPS mobile近似伪距
            gps_mobile_rho0.push_back(dis);
        }
        else gps_mobile_rho0.push_back(dis1);
    }
    return {gps_view_num-1,3,v};
}

Matrix RelativeStation::BuildBDSMinH() {
    if(bds_view_num<2) return Matrix();
    std::vector<double> v;
    int index_satellite_prn = bds_view_prn[bds_reference_star];
    double dX1 = mobile.info.BDS[index_satellite_prn].Coordinate.X - mobile.pos.Coordinate.X;
    double dY1 = mobile.info.BDS[index_satellite_prn].Coordinate.Y - mobile.pos.Coordinate.Y;
    double dZ1 = mobile.info.BDS[index_satellite_prn].Coordinate.Z - mobile.pos.Coordinate.Z;
    double dis1 = sqrt(dX1*dX1 + dY1*dY1 + dZ1*dZ1);
    double detX1=dX1/dis1,detY1=dY1/dis1,detZ1=dZ1/dis1;
    for (int i = 0; i < bds_view_num; ++i) {
        if(i!=bds_reference_star){
            double dx = -mobile.info.BDS[bds_view_prn[i]].Coordinate.X + mobile.pos.Coordinate.X;
            double dy = -mobile.info.BDS[bds_view_prn[i]].Coordinate.Y + mobile.pos.Coordinate.Y;
            double dz = -mobile.info.BDS[bds_view_prn[i]].Coordinate.Z + mobile.pos.Coordinate.Z;
            double dis = sqrt(dx*dx + dy*dy + dz*dz);
            dx=dx/dis,dy=dy/dis,dz=dz/dis;
            double alpha_x = dx+detX1,alpha_y=dy+detY1,alpha_z=dz+detZ1;
            v.push_back(alpha_x);
            v.push_back(alpha_y);
            v.push_back(alpha_z);
            // 存储BDS mobile 近似伪距
            bds_mobile_rho0.push_back(dis);
        }
        else bds_mobile_rho0.push_back(dis1);
    }
    return {bds_view_num-1,3,v};
}


Matrix RelativeStation::Build_D_P_GPSL1_L() {
    if(gps_view_num<2) return Matrix();
    if(Variance_model ==3){
        // 等权模型
        Matrix m = 2 + (2^eye(gps_view_num-1));
        return m.inv();
    }
    double single_d[gps_view_num];
    // 函数指针,根据方差模型不同指向不同函数
    double (*func)(double);
    if(Variance_model==1) {
        func=GetWeightByElevation;
        for (int i = 0; i < gps_view_num; ++i) {
            single_d[i]=(func(base.info.GPS[gps_view_prn[i]].E) + func(mobile.info.GPS[gps_view_prn[i]].E));
        }
    }
    if(Variance_model==2) {
        func=GetWeightBySNR;
        for (int i = 0; i < gps_view_num; ++i) {
            single_d[i]=(func(base.obs.GPS[gps_view_prn[i]].S[0]) + func(mobile.obs.GPS[gps_view_prn[i]].S[0]));
        }
    }
    // 计算双差方差阵
    Matrix double_d = zero(gps_view_num-1,gps_view_num-1) + single_d[gps_reference_star];
    int k=1;
    for (int i = 0; i < gps_view_num; ++i) {
        if(i!=gps_reference_star){
            // 信噪比超过限定，则赋值
            if(base.obs.is_SNR_over_gps_L1(i) && mobile.obs.is_SNR_over_gps_L1(i)) double_d.assign(k,k,double_d(k,k)+single_d[i]);
            else double_d.assign(k,k,(double_d(k,k)+single_d[i])*RATIO_LIMIT);
            k+=1;
        }
    }
    return double_d.inv();
}


Matrix RelativeStation::Build_D_P_GPSL2_L() {
    if(gps_view_num<2) return Matrix();
    if(Variance_model ==3){
        // 等权模型
        Matrix m = 2 + (2^eye(gps_view_num-1));
        return m.inv();
    }
    double single_d[gps_view_num];
    // 函数指针,根据方差模型不同指向不同函数
    double (*func)(double);
    if(Variance_model==1) {
        func=GetWeightByElevation;
        for (int i = 0; i < gps_view_num; ++i) {
            single_d[i]=(func(base.info.GPS[gps_view_prn[i]].E) + func(mobile.info.GPS[gps_view_prn[i]].E));
        }
    }
    if(Variance_model==2) {
        func=GetWeightBySNR;
        for (int i = 0; i < gps_view_num; ++i) {
            single_d[i]=(func(base.obs.GPS[gps_view_prn[i]].S[1]) + func(mobile.obs.GPS[gps_view_prn[i]].S[1]));
        }
    }
    // 计算双差方差阵
    Matrix double_d = zero(gps_view_num-1,gps_view_num-1) + single_d[gps_reference_star];
    int k=1;
    for (int i = 0; i < gps_view_num; ++i) {
        if(i!=gps_reference_star){
            // 信噪比超过限定，则赋值
            if(base.obs.is_SNR_over_gps_L2(i) && mobile.obs.is_SNR_over_gps_L2(i)) double_d.assign(k,k,double_d(k,k)+single_d[i]);
            else double_d.assign(k,k,(double_d(k,k)+single_d[i])*RATIO_LIMIT);
            k+=1;
        }
    }
    return double_d.inv();
}

Matrix RelativeStation::Build_D_P_BDSBI1_L() {
    if(bds_view_num<2) return Matrix();
    if(Variance_model ==3){
        // 等权模型
        Matrix m = 2 + (2^eye(bds_view_num-1));
        return m.inv();
    }
    double single_d[bds_view_num];
    // 函数指针,根据方差模型不同指向不同函数
    double (*func)(double);
    if(Variance_model==1) {
        func=GetWeightByElevation;
        for (int i = 0; i < bds_view_num; ++i) {
            single_d[i]=(func(base.info.BDS[bds_view_prn[i]].E) + func(mobile.info.BDS[bds_view_prn[i]].E));
        }
    }
    if(Variance_model==2) {
        func=GetWeightBySNR;
        for (int i = 0; i < bds_view_num; ++i) {
            single_d[i]=(func(base.obs.BDS[bds_view_prn[i]].S[0]) + func(mobile.obs.BDS[bds_view_prn[i]].S[0]));
        }
    }
    // 计算双差方差阵
    Matrix double_d = zero(bds_view_num-1,bds_view_num-1) + single_d[bds_reference_star];
    int k=1;
    for (int i = 0; i < bds_view_num; ++i) {
        if(i!=bds_reference_star){
            // 信噪比超过限定，则赋值
            if(base.obs.is_SNR_over_bds_I1(i) && mobile.obs.is_SNR_over_bds_I1(i)) double_d.assign(k,k,double_d(k,k)+single_d[i]);
            else double_d.assign(k,k,(double_d(k,k)+single_d[i])*RATIO_LIMIT);
            k+=1;
        }
    }
    return double_d.inv();
}

Matrix RelativeStation::Build_D_P_BDSBI3_L() {
    if(bds_view_num<2) return Matrix();
    if(Variance_model ==3){
        // 等权模型
        Matrix m = 2 + (2^eye(bds_view_num-1));
        return m.inv();
    }
    double single_d[bds_view_num];
    // 函数指针,根据方差模型不同指向不同函数
    double (*func)(double);
    if(Variance_model==1) {
        func=GetWeightByElevation;
        for (int i = 0; i < bds_view_num; ++i) {
            single_d[i]=(func(base.info.BDS[bds_view_prn[i]].E) + func(mobile.info.BDS[bds_view_prn[i]].E));
        }
    }
    if(Variance_model==2) {
        func=GetWeightBySNR;
        for (int i = 0; i < bds_view_num; ++i) {
            single_d[i]=(func(base.obs.BDS[bds_view_prn[i]].S[1]) + func(mobile.obs.BDS[bds_view_prn[i]].S[1]));
        }
    }
    // 计算双差方差阵
    Matrix double_d = zero(bds_view_num-1,bds_view_num-1) + single_d[bds_reference_star];
    int k=1;
    for (int i = 0; i < bds_view_num; ++i) {
        if(i!=bds_reference_star){
            if(base.obs.is_SNR_over_bds_I3(i) && mobile.obs.is_SNR_over_bds_I3(i)) double_d.assign(k,k,double_d(k,k)+single_d[i]);
            else double_d.assign(k,k,(double_d(k,k)+single_d[i])*RATIO_LIMIT);
            k+=1;
        }
    }
    return double_d.inv();
}

void RelativeStation::get_base_rho0() {
    // gps
    for (int i = 0; i < gps_view_num; ++i) {
        gps_base_rho0.push_back(Distance(base.pos.Coordinate,base.info.GPS[gps_view_prn[i]].Coordinate));
    }
    // bds
    for (int i = 0; i < bds_view_num; ++i) {
        bds_base_rho0.push_back(Distance(base.pos.Coordinate,base.info.BDS[bds_view_prn[i]].Coordinate));
    }
}

Matrix RelativeStation::BuildFinalH() {
    Matrix B_GPS,B_BDS;
    Matrix I_GPS,I_BDS;
    if(H_FUNC & 1){
        B_GPS = BuildGPSMinH();
        I_GPS = eye(gps_view_num-1);
    }
    if((H_FUNC>>1) & 1){
        B_BDS = BuildBDSMinH();
        I_BDS = eye(bds_view_num-1);
    }
    Matrix lambda_m[SC_NUM];  //  波长矩阵
    Matrix b_m[SC_NUM*2];
    int i = -1;
    // GL1
    if(GET_BIT(V_FUNC,0)){
        i += 1;
        // 波长矩阵构建
        lambda_m[i] = G_WL1^I_GPS;
        // B矩阵构建
        b_m[i] = B_GPS;
        b_m[i+SC_NUM] = B_GPS;
    }
    // GL2
    if(GET_BIT(V_FUNC,1)){
        i += 1;
        // 波长矩阵构建
        lambda_m[i] = G_WL2^I_GPS;
        // B矩阵构建
        b_m[i] = B_GPS;
        b_m[i+SC_NUM] = B_GPS;
    }
    // CL1
    if(GET_BIT(V_FUNC,2)){
        i += 1;
        // 波长矩阵构建
        lambda_m[i] = B_W1I^I_BDS;
        // B矩阵构建
        b_m[i] = B_BDS;
        b_m[i+SC_NUM] = B_BDS;
    }
    // CL3
    if(GET_BIT(V_FUNC,3)){
        i += 1;
        // 波长矩阵构建
        lambda_m[i] = B_W3I^I_BDS;
        // B矩阵构建
        b_m[i] = B_BDS;
        b_m[i+SC_NUM] = B_BDS;
    }
    Matrix left = vertical_stack_array(b_m,SC_NUM*2);
    Matrix right_up = diag(lambda_m,SC_NUM);
    Matrix right_down = zero(right_up.row,right_up.col);
    Matrix right = vertical_stack(right_up,right_down);
    return horizontal_stack(left,right);
}

Matrix RelativeStation::Build_D_P() {
    Matrix p_m[SC_NUM*2];
    int i = -1;
    // GL1
    if(GET_BIT(V_FUNC,0)){
        i+=1;
        p_m[i] = Build_D_P_GPSL1_L();
        p_m[i+SC_NUM] = p_m[i]^(1.0/Variance_ratio);
    }
    // GL2
    if(GET_BIT(V_FUNC,1)){
        i+=1;
        p_m[i] = Build_D_P_GPSL2_L();
        p_m[i+SC_NUM] = p_m[i]^(1.0/Variance_ratio);
    }
    // BL1
    if(GET_BIT(V_FUNC,2)){
        i+=1;
        p_m[i] = Build_D_P_BDSBI1_L();
        p_m[i+SC_NUM] = p_m[i]^(1.0/Variance_ratio);
    }
    // BL3
    if(GET_BIT(V_FUNC,3)){
        i+=1;
        p_m[i] = Build_D_P_BDSBI3_L();
        p_m[i+SC_NUM] = p_m[i]^(1.0/Variance_ratio);
    }
    return diag(p_m,SC_NUM*2);
}

bool RelativeStation::LS() {
    // 选取共视卫星
    get_public_view();
    if(gps_view_num + bds_view_num <4) {std::cout<<"共视卫星数目少于4颗，无法进行相对定位!\n";return false;} // 必要观测保证
    // 选取基准星
    select_gps_reference_satellite();
    select_bds_reference_satellite();
    get_base_rho0();
    LsRes res;
    Matrix H,L,P;
    while(1){
        H = BuildFinalH();
        L = BuildDoubleResidual();
        P = Build_D_P();
        res = LeastSquare(H,L,P);
        // 更新
        mobile.pos.Coordinate.X += res.dx(1,1);
        mobile.pos.Coordinate.Y += res.dx(2,1);
        mobile.pos.Coordinate.Z += res.dx(3,1);
        // 判断迭代是否完成
        if(res.dx(1,1)<1e-6 && res.dx(2,1)<1e-6 && res.dx(3,1)<1e-6){
            break;
        }else{
            // 迭代未完成，清空流动站到各卫星距离数组，后续会重新计算
            std::vector<double>().swap(gps_mobile_rho0);
            std::vector<double>().swap(bds_mobile_rho0);
        }
    }
    // 模糊度固定
    int n = res.dx.row - 3;
    int m = 2;
    double *a = res.dx.p+3;
    // 从Qxx中将模糊度的信息提取出来，这里注意lambda函数输入的是协方差矩阵
    Matrix K1 = horizontal_stack(zero(n,3),eye(n));
    Matrix K2 = horizontal_stack(eye(3), zero(3,n));
    // 代码纠错
    double float_baseline[] = {mobile.pos.Coordinate.X - base.pos.Coordinate.X,
                               mobile.pos.Coordinate.Y - base.pos.Coordinate.Y,
                               mobile.pos.Coordinate.Z - base.pos.Coordinate.Z};

    Matrix float_b(3,1,float_baseline);
    Matrix float_Qbb = K2*res.Qxx*K2.T();
    Matrix float_Qaa = K1*res.Qxx*K1.T();
    double *Q = float_Qaa.p;
    double F[n*m];
    double s[m];
    int solved = lambda(n,m,a,Q,F,s);
    ratio = s[1] / s[0];

    // 如果ratio的值高于阈值且lambda方法求解成功，求解固定后的基线向量
    if(ratio>=RATIO_LIMIT && solved==0 ){
        fixed = true;
        Matrix fixed_a(n,1,F);  // 第一行即残差最小行
        Matrix float_a(n,1,a);
        Matrix Qaa(n,n,Q);
        Matrix float_Qba = K2*res.Qxx*K1.T();
        Matrix fixed_b = float_b - float_Qba * Qaa.inv() * (float_a - fixed_a);
        baseline_xyz[0] = fixed_b.p[0];
        baseline_xyz[1] = fixed_b.p[1];
        baseline_xyz[2] = fixed_b.p[2];
        Matrix fixed_Qbb = float_Qbb - float_Qba * Qaa.inv() * float_Qba.T();
        // 求出固定解后更新流动站坐标
        mobile.pos.Coordinate.X += fixed_b(1,1);
        mobile.pos.Coordinate.Y += fixed_b(2,1);
        mobile.pos.Coordinate.Z += fixed_b(3,1);
        accuracy_evaluation(res.sigma2,fixed_Qbb,res.V);
    }
    else{
        // 输出基线结果并进行精度评定
        accuracy_evaluation(res.sigma2,float_Qbb,res.V);
    }
    print_accuracy_evaluation();
    return true;
}

void RelativeStation::accuracy_evaluation(double sig2,  Matrix Qbb, Matrix V) {
    sigma2 = sig2;
    // 当解未固定时，需给基线向量赋值
    if(!fixed){
        baseline_xyz[0] = mobile.pos.Coordinate.X - base.pos.Coordinate.X;
        baseline_xyz[1] = mobile.pos.Coordinate.Y - base.pos.Coordinate.Y;
        baseline_xyz[2] = mobile.pos.Coordinate.Z - base.pos.Coordinate.Z;
    }
    // 基线分量精度评定
    double sigma = sqrt(sig2);
    // x,y,z分量精度评定
    m_xyz[0] = sigma * sqrt(Qbb(1,1));
    m_xyz[1] = sigma * sqrt(Qbb(2,2));
    m_xyz[2] = sigma * sqrt(Qbb(3,3));
    // n,e,u分量精度评定
    BLH blh = XYZ2BLH(base.pos.Coordinate,"WGS84");
    double B = blh.lat , L=blh.lon;
    double sinB = sin(B),cosB = cos(B),sinL = sin(L), cosL = cos(L);
    double p[9] = {-sinB*cosL, -sinB*sinL, cosB,
                   -sinL     , cosL      , 0   ,
                   cosB*cosL , cosB*sinL , sinB};
    Matrix K(3,3,p);
    Matrix xyz(3,1,baseline_xyz);
    Matrix neu = K*xyz;
    baseline_neu[0] = neu(1,1);
    baseline_neu[1] = neu(2,1);
    baseline_neu[2] = neu(3,1);
    Matrix Q_neu = K*Qbb*K.T();
    m_neu[0] = sigma * sqrt(Q_neu(1,1));
    m_neu[1] = sigma * sqrt(Q_neu(2,2));
    m_neu[2] = sigma * sqrt(Q_neu(3,3));
    //
    // 静态基线长度计算及精度评定
    double S0 = Distance(base.pos.Coordinate,mobile.pos.Coordinate);
    double fp[3] = {baseline_xyz[0]/S0, baseline_xyz[1]/S0, baseline_xyz[2]/S0};
    Matrix kf(1,3,fp);
    S_length = S0;
    m_S = sigma * sqrt((kf*Qbb*kf.T())(1,1));
    // DOP值
    RDOP = Qbb.tr();
    // RMS值
    RMS = sqrt((V.T()*V)(1,1)/V.row);
}

void RelativeStation::print_accuracy_evaluation() {
    printf("GPST:%d %5.3f  State:",base.obs.t.weeks,base.obs.t.second);
    if(fixed) std::cout<<"fixed";
    else std::cout<<"float";
    std::cout<<" GPSnum:"<<gps_view_num<<" "<<"BDSnum:"<<bds_view_num;
    for (int i = 0; i < gps_view_num; ++i) {
        printf(" G%02d",gps_view_prn[i]+1);
    }
    for (int i = 0; i < bds_view_num; ++i) {
        printf(" C%02d",bds_view_prn[i]+1);
    }
    std::cout<<"\n";
    printf("xyz:%.5f  %.5f  %.5f  neu:%.5f  %.5f  %.5f  S:%.5f  sigma2:%.3f  RMS:%.5f\n m :%.5f  %.5f  %.5f   m :%.5f  %.5f  %.5f  m:%.5f  ratio:%3.3f\n",
           baseline_xyz[0],baseline_xyz[1],baseline_xyz[2],baseline_neu[0],baseline_neu[1],baseline_neu[2],S_length,
           sigma2,RMS,m_xyz[0],m_xyz[1],m_xyz[2],m_neu[0],m_neu[1],m_neu[2],m_S,ratio);
    std::cout<<"\n";
}


extern uint8_t V_FUNC=0;
extern uint8_t P_FUNC=0;
extern uint8_t H_FUNC=0;
extern uint8_t SC_NUM=0;
extern uint8_t SYS_NUM=0;


void dispose_RTK(){
    // 目前配置只含GL1,GL2,CL1,CL3
    /* 双差观测值矩阵从上往下排列的顺序为：
     * GPS在上 BDS在下
     * L1在上 L2在下(或者BI1在上，BI3在下)
     * 载波在上，伪距在下
     */

    if(strstr(Satellite_system_frequency,"G")){
        H_FUNC = SET_BIT1(H_FUNC,0); // bit0设置为1
        SYS_NUM +=1;
        if(strstr(Satellite_system_frequency,"GL1")){
            // bit0 设置为1
            P_FUNC = SET_BIT1(P_FUNC,0);
            V_FUNC = SET_BIT1(V_FUNC,0);
            SC_NUM += 1;
        }
        if(strstr(Satellite_system_frequency,"GL2")){
            // bit1 设置为1
            P_FUNC = SET_BIT1(P_FUNC,1);
            V_FUNC = SET_BIT1(V_FUNC,1);
            SC_NUM += 1;
        }
    }
    if(strstr(Satellite_system_frequency,"C")){
        H_FUNC = SET_BIT1(H_FUNC,1); // bit1 设置为1
        SYS_NUM +=1;
        if(strstr(Satellite_system_frequency,"CL1")){
            // bit2 设置为1
            P_FUNC = SET_BIT1(P_FUNC,2);
            V_FUNC = SET_BIT1(V_FUNC,2);
            SC_NUM += 1;
        }
        if(strstr(Satellite_system_frequency,"CL3")){
            // bit3 设置为1
            P_FUNC = SET_BIT1(P_FUNC,3);
            V_FUNC = SET_BIT1(V_FUNC,3);
            SC_NUM += 1;
        }
    }
}

bool is_slip(double GL1,double GL2){
    if(abs(GL2 - GL1) > 0.0131)  return true;
    return false;
}









