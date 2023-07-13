//
// Created by 0-0 mashuo on 2023/3/6.
//

#include <cmath>
#include <cstdio>
#include "GnssData.h"


Stalite::Stalite() {
    C[0]=C[1]=L[0]=L[1]=D[0]=D[1]=S[0]=S[1]=0;
}

void Stalite::Formatting() {
    C[0]=C[1]=L[0]=L[1]=S[0]=S[1]=D[0]=D[1]=0;
}

void OBS::get_GPS_PIF(const int *prn, int num, double *pif) {
    for (int i = 0; i < num; ++i) {
        pif[prn[i]]=f(G_fL1,G_fL2)*GPS[prn[i]].C[0]+f(G_fL2,G_fL1)*GPS[prn[i]].C[1];
    }
}

void OBS::get_BDS_PIF(const int prn[],int num,double pif[],BDSEPH *bdseph,GNSSINFO *info)
{
    double k13f=B_f1I*B_f1I/(B_f3I*B_f3I);
    for (int i = 0; i < num; ++i)
    {
        // BDS卫星需要改正tgd
        double tgd1=bdseph[prn[i]].tgd1;
        double d_tgd=C_Light*k13f*tgd1/(1-k13f);
        double clk=C_Light*info->BDS[prn[i]].clock_error;
        double pr1=BDS[prn[i]].C[0],pr3=BDS[prn[i]].C[1];
        double pr=f(B_f1I,B_f3I)*BDS[prn[i]].C[0]+f(B_f3I,B_f1I)*BDS[prn[i]].C[1];
        pif[prn[i]]=pr+d_tgd;
    }
}

void OBS::find_error_P(GPSEPH *gpseph, BDSEPH *bdseph)
{
    // 重置
    G_num=0,C_num=0;
    G_prn = new int[GPSMAXPRN];
    C_prn = new int[BDSMAXPEN];
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if(GPS[i].C[0]!=0 && GPS[i].C[1]!=0 && gpseph[i].flag)
        {
            // 可在此添加伪距粗差探测代码
            G_prn[G_num]=i;
            G_num+=1;
        }
        //else gpseph[i].flag= false; //非双频观测星历设置为无效
    }
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if(BDS[i].C[0]!=0 && BDS[i].C[1]!=0 && bdseph[i].flag)
        {
            // 可在此添加伪距粗差探测代码

            C_prn[C_num]=i;
            C_num+=1;
        }
        //else bdseph[i].flag= false;
    }
}

void OBS::release() {
    delete []G_prn;
    G_prn = nullptr;
    delete []C_prn;
    C_prn = nullptr;
}

void OBS::Formatting() {
    G_num=C_num=0;
    // 避免多次释放内存
    if( G_prn!= nullptr && C_prn!= nullptr){
        release();
    }
    for (auto & i : GPS) {
        i.Formatting();
    }
    for (auto & i: BDS)  {
        i.Formatting();
    }
    nums=0;
}

bool OBS::is_gps_complete_L1(int prn) {
    if (prn<0 || prn>=GPSMAXPRN){
        std::cerr<<"error in OBS::is_gps_complete(int prn):prn超过索引允许范围!";
    }
    if (GPS[prn].C[0]!=0 && GPS[prn].D[0]!=0 &&
        GPS[prn].L[0]!=0 && GPS[prn].S[0]!=0 ){
        return true;
    }
    return false;
}

bool OBS::is_bds_complete_I1(int prn) {
    if (prn<0 || prn>=BDSMAXPEN){
        std::cerr<<"error in OBS::is_bds_complete(int prn):prn超过索引允许范围!";
    }
    if (BDS[prn].C[0]!=0 && BDS[prn].D[0]!=0 &&
        BDS[prn].L[0]!=0 && BDS[prn].S[0]!=0 ){
        return true;
    }
    return false;
}

bool OBS::is_SNR_over_bds_I1(int i) {
    if (i<0 || i>=BDSMAXPEN){
        std::cerr<<"error in OBS::is_bds_complete(int prn):prn超过索引允许范围!";
    }
    if(BDS[i].S[0] >= S_limit) return true;
    return false;
}

bool OBS::is_SNR_over_bds_I3(int i) {
    if (i<0 || i>=BDSMAXPEN){
        std::cerr<<"error in OBS::is_bds_complete(int prn):prn超过索引允许范围!";
    }
    if(BDS[i].S[1] >= S_limit) return true;
    return false;
}

bool OBS::is_SNR_over_gps_L1(int i) {
    if (i<0 || i>=GPSMAXPRN){
        std::cerr<<"error in OBS::is_gps_complete(int prn):prn超过索引允许范围!";
    }
    if(GPS[i].S[0] >= S_limit) return true;
    return false;
}

bool OBS::is_SNR_over_gps_L2(int i) {
    if (i<0 || i>=GPSMAXPRN){
        std::cerr<<"error in OBS::is_gps_complete(int prn):prn超过索引允许范围!";
    }
    if(GPS[i].S[1] >= S_limit) return true;
    return false;
}

bool OBS::is_gps_complete_L2(int prn) {
    if (prn<0 || prn>=GPSMAXPRN){
        std::cerr<<"error in OBS::is_gps_complete(int prn):prn超过索引允许范围!";
    }
    if (GPS[prn].C[1]!=0 && GPS[prn].D[1]!=0 &&
        GPS[prn].L[1]!=0 && GPS[prn].S[1]!=0 ){
        return true;
    }
    return false;
}

bool OBS::is_bds_complete_I3(int prn) {
    if (prn<0 || prn>=BDSMAXPEN){
        std::cerr<<"error in OBS::is_bds_complete(int prn):prn超过索引允许范围!";
    }
    if (BDS[prn].C[1]!=0 && BDS[prn].D[1]!=0 &&
        BDS[prn].L[1]!=0 && BDS[prn].S[1]!=0 ){
        return true;
    }
    return false;
}

SatelliteINFO::SatelliteINFO() : Vx(0.0),Vy(0.0),Vz(0.0),clock_error(0.0),v_clock_error(0.0),Coordinate(0,0,0) {}

void GNSSINFO::getGPS(double limit) {
    G_prn=new int[GPSMAXPRN];
    int num=0;
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if(this->GPS[i].flag && this->GPS[i].E>=limit)
        {
            G_prn[num]=i;
            num+=1;
        }
    }
    G_num=num;
}

void GNSSINFO::getBDS(double limit) {
    C_prn=new int[BDSMAXPEN];
    int num=0;
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if(this->BDS[i].flag && this->BDS[i].E>=limit)
        {
            C_prn[num]=i;
            num+=1;
        }
    }
    C_num=num;
}

void GNSSINFO::calculate_EA(XYZ cor) {
    // 计算位置有效GPS卫星的高度角、方位角
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if(GPS[i].flag)
        {
            ENU enu = XYZ2ENU(cor,GPS[i].Coordinate,"WGS84");
            double e=enu.E,n=enu.N,u=enu.U;
            GPS[i].E=asin(sqrt((u*u)/(e*e+n*n+u*u)));
            GPS[i].A=atan2(e,n);
        }
    }
    // 计算位置有效BDS卫星的高度角、方位角
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if(BDS[i].flag)
        {
            ENU enu= XYZ2ENU(cor,BDS[i].Coordinate,"CGCS2000");
            double e=enu.E,n=enu.N,u=enu.U;
            BDS[i].E=asin(sqrt((u*u)/(e*e+n*n+u*u)));
            BDS[i].A=atan2(e,n);
        }
    }
}

void GNSSINFO::release() {
    delete []G_prn;
    delete []C_prn;
}

void ReciverPos::print() {
    CommonTime ct= GPST2Common(t);
    printf("%u %6.3f %u %02u %02u %02u %02u %04.1f %.3f %.3f %.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f  %4.2f  %5.3f %6.3f %6.3f %02u %02u\n",
           t.weeks,t.second,ct.year,ct.month,ct.day,ct.hour,ct.minute,ct.second,
           Coordinate.X,Coordinate.Y,Coordinate.Z,enu.E,enu.N,enu.U,dis,Vx,Vy,Vz,PDOP,sigma0,G_clk,C_clk,G_num,C_num);
}

void ReciverPos::calculate_enu(XYZ xyz) {
    enu= XYZ2ENU(xyz,Coordinate,"WGS84");
    dis=sqrt(enu.E*enu.E + enu.N*enu.N + enu.U*enu.U);
}




