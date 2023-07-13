//
// Created by 0-0 mashuo on 2023/3/6.
//

#include <iostream>
#include <cmath>
#include <iomanip>
#include "Positioning.h"
#include "Matrix.h"

int PIF_location(OBS *obs, GPSEPH *gpseph, BDSEPH *bdseph, ReciverPos *Pos,GNSSINFO *info) {
    // 剔除粗差、统计可用卫星
    obs->find_error_P(gpseph, bdseph);
    if (obs->C_num + obs->G_num < 5)
    {
        std::cout<<"GPS、BDS双系统可观测卫星少于五个，无法进行定位\n";
        Pos->known= false;
        return 0;
    } // 可用卫星少于五个、或者单系统返回-1
    // 计算卫星发射时刻
    info->calculate_tr(obs, gpseph, bdseph);
    double limit=info->E_limit;
    if(Pos->Coordinate.X ==R_WGS84 && Pos->Coordinate.Y==0 && Pos->Coordinate.Z==0) {limit=0;} // 如果初始状态位置位置，则不剔除高度角小于15°卫星
    else {info->calculate_EA(Pos->Coordinate);}  // 如果已知初始坐标，则计算卫星高度角，并剔除高度角小于15°的卫星
    info->getBDS(limit);
    info->getGPS(limit);
    if (info->C_num + info->G_num < 5)
    {
        std::cout<<"可计算坐标的双系统卫星少于五个，无法进行定位\n";
        Pos->known= false;
        return 0;
    } // 可用卫星少于五个、或者单系统返回-1
    // 单点定位
    double G_pif[GPSMAXPRN] ={0};
    double C_pif[BDSMAXPEN] ={0};

    obs->get_GPS_PIF(info->G_prn, info->G_num, G_pif);
    obs->get_BDS_PIF(info->C_prn, info->C_num, C_pif, bdseph, info);
    Pos->t = obs->t; // 定位时间
    Pos->G_num=info->G_num, Pos->C_num=info->C_num; // 卫星数目
    calculate_pos(info, G_pif, C_pif, Pos);
    XYZ xyz0(-2267804.5263, 5009342.3723, 3220991.8632); //测站参考坐标
    Pos->calculate_enu(xyz0);
    // 单点测速
    calculate_v(info,obs,Pos);
    return 1;
}

void calculate_pos(GNSSINFO *info, double G_pif[], double C_pif[], ReciverPos *Pos) {
    // 复制可用卫星数目、PRN号
    int G_num = info->G_num, C_num = info->C_num;
    int *G_prn = info->G_prn, *C_prn = info->C_prn;
    // 参数（GPS接收机钟差、BDS接收机钟差初始均设为0）
    double x_array[5] = {Pos->Coordinate.X, Pos->Coordinate.Y, Pos->Coordinate.Z, 0, 0};
    int n = 0;
    if(G_num==0 || C_num ==0) n=4;
    else n=5;
    Matrix x(n, 1, x_array);
    Matrix dx, Qxx, v;
    double sigma0 = 100;
    auto *G_Trop = new double[G_num]; // 对流层误差
    for (int i = 0; i < G_num; ++i) {
        G_Trop[i] = 0;
    }
    auto *C_Trop = new double[C_num]; // 对流层误差
    for (int i = 0; i < C_num; ++i) {
        C_Trop[i] = 0;
    }
    int k = 0; //迭代次数
    // 构建B、W矩阵
    double B_array[(G_num + C_num) * n];
    Matrix B(G_num + C_num, n, B_array);
    double W_array[G_num + C_num];
    Matrix W(G_num + C_num, 1, W_array);
    // 构建权阵P
    Matrix P = eye(G_num + C_num);
    bool Trop_done = false; // 记录对流层误差是否改正

    // 迭代定位计算
    LOOP:
    do {
        // 给B、W矩阵赋值
        for (int i = 1; i <= G_num; ++i) {
            double row0 = Distance(Pos->Coordinate, info->GPS[G_prn[i - 1]].Coordinate);
            B.assign(i, 1, (Pos->Coordinate.X - info->GPS[G_prn[i - 1]].Coordinate.X) / row0);
            B.assign(i, 2, (Pos->Coordinate.Y - info->GPS[G_prn[i - 1]].Coordinate.Y) / row0);
            B.assign(i, 3, (Pos->Coordinate.Z - info->GPS[G_prn[i - 1]].Coordinate.Z) / row0);
            if(C_num==0) B.assign(i, 4, 1);
            else {B.assign(i, 4, 1);B.assign(i, 5, 0);}
            W.assign(i, 1, G_pif[G_prn[i - 1]] -
                           (row0 + x(4, 1) - C_Light * info->GPS[G_prn[i - 1]].clock_error + G_Trop[i-1]));
        }
        for (int i = 1 + G_num; i <= C_num + G_num; ++i) {
            double row0 = Distance(Pos->Coordinate, info->BDS[C_prn[i - 1 - G_num]].Coordinate);
            B.assign(i, 1, (Pos->Coordinate.X - info->BDS[C_prn[i - 1 - G_num]].Coordinate.X) / row0);
            B.assign(i, 2, (Pos->Coordinate.Y - info->BDS[C_prn[i - 1 - G_num]].Coordinate.Y) / row0);
            B.assign(i, 3, (Pos->Coordinate.Z - info->BDS[C_prn[i - 1 - G_num]].Coordinate.Z) / row0);
            if(G_num==0) B.assign(i,4,1);
            else {B.assign(i, 4, 0);B.assign(i, 5, 1);}
            W.assign(i, 1, C_pif[C_prn[i - 1 - G_num]] -
                           (row0 + x(n, 1) - C_Light * info->BDS[C_prn[i - 1 - G_num]].clock_error +
                            C_Trop[i-1-G_num]));
        }
        dx = (B.T() * P * B).inv() * B.T() * P * W;
        x = x + dx;
        v = B * dx - W;
        Matrix v2 = v.T() * P * v;
        if (C_num + G_num > 5) sigma0 = sqrt(v2(1, 1) / (G_num + C_num - 5));
        Qxx = (B.T() * P * B).inv();
        // 更新定位结果信息
        Pos->Coordinate.X = x(1, 1);
        Pos->Coordinate.Y = x(2, 1);
        Pos->Coordinate.Z = x(3, 1);
        Pos->sigma0 = sigma0;
        Pos->Qxx.release();
        Pos->Qxx = Qxx;
        if(G_num == 0) Pos->C_clk = x(4,1);
        if(C_num == 0) Pos->G_clk = x(4,1);
        if(n==5) {Pos->G_clk = x(4,1);Pos->C_clk = x(5,1);}
        Pos->PDOP = Qxx(1, 1) + Qxx(2, 2) + Qxx(3, 3);
        Pos->known= true;

        // 定位精度足够时，进行对流层改正
        if (dx(1, 1) <= 1e-6 && dx(2, 1) <= 1e-6 && dx(3, 1) <= 1e-6  && !Trop_done) {
            // 当测站坐标基本稳定时，根据测站坐标和卫星坐标计算对流层延迟
            info->calculate_EA(Pos->Coordinate); //计算卫星高度角、方位角
            double H = XYZ2BLH(Pos->Coordinate, "WGS84").H; //计算测站大地高
            for (int i = 0; i < G_num; ++i) {
                double E = info->GPS[G_prn[i]].E;
                G_Trop[i] = Hopefiled(H, E);
            }
            for (int i = 0; i < C_num; ++i) {
                double E = info->BDS[C_prn[i]].E;
                C_Trop[i] = Hopefiled(H, E);
            }
            info->calculate_rotation(0, 0, G_pif, C_pif);
            //根据接收机钟差计算卫星位置地球自转改正(忽略接收机钟差影响)
            Trop_done = true;
            goto LOOP; // 这里使用了LOOP语句，用于选择合适时机改正对流层误差
        }
        if ( ( abs(dx(1,1)) <=1e-6 && abs(dx(2,1)) <= 1e-6 && abs(dx(3,1)) <= 1e-6 ) || k>9){
            break;
        }
    } while (1);
    // delete
    delete []G_Trop;
    delete []C_Trop;
}

void calculate_v(GNSSINFO *info, OBS *obs, ReciverPos *Pos)
{
    // 获取单频点多普勒观测值（GPS选用L1频点，BDS选用B1I频点）
    int G_num=info->G_num,C_num=info->C_num;
    int *G_prn=info->G_prn,*C_prn=info->C_prn;
    double D[G_num+C_num];
    for (int i = 0; i < G_num; ++i) {
        D[i]=obs->GPS[G_prn[i]].D[0]*G_WL1;
    }
    for (int i = G_num; i <G_num+C_num ; ++i) {
        D[i]=obs->BDS[C_prn[i-G_num]].D[0]*B_W1I;
    }
    // 构建B矩阵、W矩阵
    double B_array[(G_num+C_num)*4],W_array[G_num+C_num];
    Matrix B(G_num+C_num,4,B_array),W(G_num+C_num,1,W_array);
    for (int i = 1; i <= G_num; ++i) {
        double row0 = Distance(Pos->Coordinate,info->GPS[G_prn[i-1]].Coordinate);
        double dx = Pos->Coordinate.X - info->GPS[G_prn[i-1]].Coordinate.X;
        double dy = Pos->Coordinate.Y - info->GPS[G_prn[i-1]].Coordinate.Y;
        double dz = Pos->Coordinate.Z - info->GPS[G_prn[i-1]].Coordinate.Z;
        double rate_row0 = -(dx * info->GPS[G_prn[i-1]].Vx + dy * info->GPS[G_prn[i-1]].Vy +
                             dz * info->GPS[G_prn[i-1]].Vz) / row0;
        B.assign(i,1,dx/row0);
        B.assign(i,2,dy/row0);
        B.assign(i,3,dz/row0);
        B.assign(i,4,1);
        W.assign(i,1,D[i-1]+rate_row0+C_Light*info->GPS[G_prn[i-1]].v_clock_error);
    }
    for (int i = G_num+1; i <=C_num+G_num; ++i) {
        int index = C_prn[i-1-G_num];
        double row0 = Distance(Pos->Coordinate,info->BDS[index].Coordinate);
        double dx = Pos->Coordinate.X - info->BDS[index].Coordinate.X;
        double dy = Pos->Coordinate.Y - info->BDS[index].Coordinate.Y;
        double dz = Pos->Coordinate.Z - info->BDS[index].Coordinate.Z;
        double rate_row0 = -(dx * info->BDS[index].Vx + dy * info->BDS[index].Vy +
                             dz * info->BDS[index].Vz) / row0;
        B.assign(i,1,dx/row0);
        B.assign(i,2,dy/row0);
        B.assign(i,3,dz/row0);
        B.assign(i,4,1);
        W.assign(i,1,D[i-1]+rate_row0+C_Light*info->BDS[index].v_clock_error);
    }
    // 单点测速
    double x_array[4]={0,0,0,0};
    Matrix x(4,1,x_array),dx,v;
    Matrix P=eye(G_num+C_num);
    Matrix v_Qxx =(B.T()*P*B).inv();
    dx = v_Qxx*B.T()*P*W;
    v = B*dx - W;
    Matrix v2=v.T()*P*v;
    double sigma0 = (G_num + C_num)>4 ? sqrt(v2(1,1)/(G_num+C_num-4)) : 100;
    x = x + dx;
    // 记录
    Pos->Vx=x(1,1);
    Pos->Vy=x(2,1);
    Pos->Vz=x(3,1);
    Pos->v_sigma0=sigma0;
    Pos->v_Qxx=v_Qxx;
}

double KlobucharB(const double alpha[], const double beta[], BDST time, BLH blh, double E, double A) {
    double R = 6378, h = 375; //地球半径、电离层高度
    double psi = pi / 2 - E - asin(R / (R + h) * cos(E)); //rad
    double phi = asin(sin(blh.lat) * cos(psi) + cos(blh.lat) * sin(psi) * cos(A));
    double lambda = blh.lon + asin(sin(psi) * sin(A) / cos(phi));
    double A2 = alpha[0] + alpha[1] * phi / pi + alpha[2] * pow(phi / pi, 2) + alpha[3] * pow(phi / pi, 3);
    if (A2 < 0) A2 = 0;
    double A4 = beta[0] + beta[1] * phi / pi + beta[2] * pow(phi / pi, 2) + alpha[3] * pow(phi / pi, 3);
    if (A4 >= 172800) A4 = 172800;
    if (A4 < 72000) A4 = 72000;
    double t = remainder((time.second + lambda * 43200 / pi), 86400); //浮点数取模
    double Iz;
    if (abs(t - 50400) < A4 / 4) Iz = 5 * 1e-9 + A2 * cos(2 * pi * (t - 50400) / A4);
    else Iz = 5 * 1e-9;
    return Iz / sqrt(1 - pow(R / (R + h) * cos(E), 2));
}

double KlobucharG(const double alpha[], const double beta[], GPST time, BLH blh, double E, double A) {
    double EA = 445 / (deg(E) + 20) - 4;  //deg
    double phi_p = deg(blh.lat) + EA * cos(A);  //deg
    double lambda_p = deg(blh.lon) + EA * sin(A) / cos(blh.lat);  //deg
    // 观测时间UT
    CommonTime ct = GPST2Common(time);
    double t = ct.hour + lambda_p / 15;
    t >= 24 ? t -= 24 : t = t;
    // P处地磁纬度
    double phi_m = phi_p + 10.07 * cos(rad(lambda_p - 288.04));  //deg
    // 振幅和周期
    double A0 = alpha[0] + alpha[1] * phi_m + alpha[2] * pow(phi_m, 2) + alpha[3] * pow(phi_m, 3);
    double P = beta[0] + beta[1] * phi_m + beta[2] * pow(phi_m, 2) + beta[3] * pow(phi_m, 3);
    double Tg;
    double secZ = 1 + 2 * pow((96 - deg(E)) / 90, 3);
    if (t > 6 && t < 22) {
        Tg = 5 * 1e-9 + A0 * cos(2 * pi / P * (t - 14));
        Tg = Tg * secZ;
    } else { Tg = 5 * 1e-9 * secZ; }
    return Tg;
}

double Hopefiled(double H, double E) {
    if (H > 50000) { return 0; }
    double T = Hopefield_T0 - 0.0065 * (H - Hopefield_H0);
    double p = Hopefield_p0 * pow((1 - 0.0000226 * (H - Hopefield_H0)), 5.225);
    double RH = Hopefield_RH0 * exp(-0.0006396 * (H - Hopefield_H0));
    double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    double hd = 40136 + 148.72 * (Hopefield_T0 - 273.16);
    double hw = 11000;
    double Kd = 155.2e-7 * p / T * (hd - H);
    double Kw = 155.2e-7 * 4810 / T / T * e * (hw - H);
    return Kd / sin(rad(sqrt(pow(deg(E), 2) + 6.25))) + Kw / sin(rad(sqrt(pow(deg(E), 2) + 2.25))) ;
}

int GPS_POS(GNSSINFO *info, GPSEPH *eph, GPST time[]) {
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if (!eph[i].flag) {
            info->GPS[i].flag = false;
            continue;
        } //没有星历的卫星不计算位置,并将位置状态设为未知
        double e = eph[i].ecc; //偏心率
        // 判断星历是否有效
        double week = time[i].weeks - eph[i].week;
        double tk = time[i].second - eph[i].toe;
        if (abs(week * 604800 + tk) >= 7200) {
            info->GPS[i].flag = false;
            continue;
        } //星历无效的卫星不计算位置，并将位置状态设为未知
        // 计算tk
        if (tk > 302400) tk = tk - 604800;
        else if (tk < -302400) tk = tk + 604800;
        double n0 = sqrt(GM_Earth / pow(eph[i].A, 3.0));
        double n = n0 + eph[i].detN;
        double Mk = eph[i].M0 + n * tk;  // 平近点角
        // 迭代计算偏近点角
        double Ek = 0, Ek0;
        do {
            Ek0 = Ek;
            Ek = Mk + e * sin(Ek);
        } while (abs(Ek0 - Ek) >= 1e-8);
        // 计算真近点角
        double Vk = atan2(sqrt(1 - e * e) * sin(Ek) / (1 - e * cos(Ek)), (cos(Ek) - e) / (1 - e * cos(Ek)));
        // 升交角距
        double Phik = Vk + eph[i].omega;
        // 计算二阶调和改正项
        double det_uk = eph[i].cus * sin(2 * Phik) + eph[i].cuc * cos(2 * Phik);
        double det_rk = eph[i].crs * sin(2 * Phik) + eph[i].crc * cos(2 * Phik);
        double det_ik = eph[i].cis * sin(2 * Phik) + eph[i].cic * cos(2 * Phik);
        double uk = Phik + det_uk;
        double rk = eph[i].A * (1 - e * cos(Ek)) + det_rk;
        double ik = eph[i].I0 + det_ik + eph[i].det_IO * tk;
        // 卫星在轨道平面上的位置
        double _xk = rk * cos(uk), _yk = rk * sin(uk);
        // 改正后的升交点经度
        double Omegak = eph[i].omega_0 + (eph[i].det_omega - Omega_WGS) * tk - Omega_WGS * eph[i].toe;
        // 计算出GPS卫星在地固坐标系下的位置
        double xk = _xk * cos(Omegak) - _yk * cos(ik) * sin(Omegak);
        double yk = _xk * sin(Omegak) + _yk * cos(ik) * cos(Omegak);
        double zk = _yk * sin(ik);
        // GPS卫星运动速度计算
        double rate_Ek = n / (1 - e * cos(Ek));
        double rate_Phik = sqrt((1 + e) / (1 - e)) * pow(cos(Vk / 2) / cos(Ek / 2), 2) * rate_Ek;
        double rate_uk = 2 * (eph[i].cus * cos(2 * Phik) - eph[i].cuc * sin(2 * Phik)) * rate_Phik + rate_Phik;
        double rate_rk = eph[i].A * e * sin(Ek) * rate_Ek +
                         2 * (eph[i].crs * cos(2 * Phik) - eph[i].crc * sin(2 * Phik)) * rate_Phik;
        double rate_ik = eph[i].det_IO + 2 * (eph[i].cis * cos(2 * Phik) - eph[i].cic * sin(2 * Phik)) * rate_Phik;
        double rate_Omegak = eph[i].det_omega - Omega_WGS;
        double R[12] = {cos(Omegak), -sin(Omegak) * cos(ik), -(_xk * sin(Omegak) + _yk * cos(Omegak) * cos(ik)),
                        _yk * sin(Omegak) * sin(ik), sin(Omegak), cos(Omegak) * cos(ik),
                        _xk * cos(Omegak) - _yk * sin(Omegak) * cos(ik), _yk * cos(Omegak) * sin(ik),
                        0, sin(ik), 0, _yk * cos(ik)};
        Matrix rate_R(3, 4, R);
        double rateof_xk = rate_rk * cos(uk) - rk * rate_uk * sin(uk);
        double rateof_yk = rate_rk * sin(uk) + rk * rate_uk * cos(uk);
        double m_right[4] = {rateof_xk, rateof_yk, rate_Omegak, rate_ik};
        Matrix right(4, 1, m_right);
        Matrix rate_xyz = rate_R * right;
        //GPS 卫星钟差、种速计算
        double F = -2 * sqrt(GM_Earth) / pow(C_Light, 2.0);
        double dt = time[i].second - eph[i].toc;
        if (dt > 302400) dt = dt - 604800;
        else if (dt < -302400) dt = dt + 604800;
        double dtr = F * e * sqrt(eph[i].A) * sin(Ek);
        double Clock_error = eph[i].a0 + eph[i].a1 * dt + eph[i].a2 * dt * dt + dtr; //相对论效应改正、对于双频组合观测值而言，不需要群延差改正
        double rate_dtr = F * e * sqrt(eph[i].A) * cos(Ek) * rate_Ek;
        double v_clock_error = eph[i].a1 + 2 * eph[i].a2 * dt + rate_dtr;
        // 赋值
        info->GPS[i].Coordinate.X = xk;
        info->GPS[i].Coordinate.Y = yk;
        info->GPS[i].Coordinate.Z = zk;
        info->GPS[i].Vx = rate_xyz(1, 1);
        info->GPS[i].Vy = rate_xyz(2, 1);
        info->GPS[i].Vz = rate_xyz(3, 1);
        info->GPS[i].clock_error = Clock_error;
        info->GPS[i].v_clock_error = v_clock_error;
        info->GPS[i].flag = true;
    }
    return 1;
}

int BDS_POS(GNSSINFO *info, BDSEPH *eph, GPST time[]) {
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if (!eph[i].flag) {
            info->BDS[i].flag = false;
            continue;
        } //未收到星历的卫星位置不计算，并将卫星位置设为未知
        double e = eph[i].ecc, A = pow(eph[i].RootA, 2.0);
        double n0 = sqrt(GM_BDS / pow(A, 3.0));
        // 转换时间
        BDST bt = GPST2BDST(time[i]);
        double week = bt.weeks - eph[i].week;
        double tk = bt.second - eph[i].toe;
        if (abs(week * 604800 + tk) >= 3600) {
            info->BDS[i].flag = false;
            continue;
        } //星历无效的卫星位置不计算，并将卫星位置设为未知
        // 时间差tk计算
        if (tk > 302400) tk -= 604800;
        else if (tk < -302400) tk += 604800;
        double n = n0 + eph[i].detN;
        double Mk = eph[i].M0 + n * tk;
        // 迭代计算偏近点角
        double Ek = 0, Ek0;
        do {
            Ek0 = Ek;
            Ek = Mk + e * sin(Ek);
        } while (abs(Ek0 - Ek) >= 1e-8);
        // 计算真近点角
        double Vk = atan2(sqrt(1 - e * e) * sin(Ek) / (1 - e * cos(Ek)), (cos(Ek) - e) / (1 - e * cos(Ek)));
        double Phik = Vk + eph[i].omega; //升交角距
        double cus = eph[i].cus, cuc = eph[i].cuc;
        double crs = eph[i].crs, crc = eph[i].crc;
        double cis = eph[i].cis, cic = eph[i].cic;
        // 改正项
        double det_uk = cus * sin(2 * Phik) + cuc * cos(2 * Phik);
        double det_rk = crs * sin(2 * Phik) + crc * cos(2 * Phik);
        double det_ik = cis * sin(2 * Phik) + cic * cos(2 * Phik);
        double uk = Phik + det_uk;
        double rk = A * (1 - e * cos(Ek)) + det_rk;
        double ik = eph[i].i0 + eph[i].IDOT * tk + det_ik;
        // 卫星在轨道面内的坐标
        double _xk = rk * cos(uk), _yk = rk * sin(uk);
        // 旋转坐标系到BDSC
        double xk, yk, zk, Omegak, rate_Ek;
        Matrix rate_xyz;
        if (i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 58 || i == 59 || i == 60) {// GEO卫星坐标求法
            Omegak = eph[i].Omega0 + eph[i].det_Omega * tk - Omega_BDS * eph[i].toe;
            double Xk = _xk * cos(Omegak) - _yk * cos(ik) * sin(Omegak);
            double Yk = _xk * sin(Omegak) + _yk * cos(ik) * cos(Omegak);
            double Zk = _yk * sin(ik);
            double array_Rx[9] = {1, 0, 0, 0, cos(rad(-5.0)),
                                  sin(rad(-5.0)), 0, -sin(rad(-5.0)), cos(rad(-5.0))};
            double theta = Omega_BDS * tk;
            double array_Rz[9] = {cos(theta), sin(theta), 0,
                                  -sin(theta), cos(theta), 0,
                                  0, 0, 1};
            Matrix Rx(3, 3, array_Rx), Rz(3, 3, array_Rz);
            double right[3] = {Xk, Yk, Zk};
            Matrix R_right(3, 1, right);
            Matrix xyz = Rz * Rx * R_right;
            xk = xyz(1, 1);
            yk = xyz(2, 1);
            zk = xyz(3, 1);
            // 计算卫星速度
            rate_Ek = n / (1 - e * cos(Ek));
            double rate_Phik = sqrt(1 - e * e) * rate_Ek / (1 - e * cos(Ek));
            double rate_uk = 2 * (eph[i].cus * cos(2 * Phik) - eph[i].cuc * sin(2 * Phik)) * rate_Phik + rate_Phik;
            double rate_rk = A * e * sin(Ek) * rate_Ek +
                             2 * (eph[i].crs * cos(2 * Phik) - eph[i].crc * sin(2 * Phik)) * rate_Phik;
            double rate_ik = eph[i].IDOT + 2 * (eph[i].cis * cos(2 * Phik) - eph[i].cic * sin(2 * Phik)) * rate_Phik;
            double rate_Omegak = eph[i].det_Omega;

            double rateof_xk = rate_rk * cos(uk) - rk * rate_uk * sin(uk);
            double rateof_yk = rate_rk * sin(uk) + rk * rate_uk * cos(uk);
            double rate_Xk =
                    -yk * rate_Omegak - (rateof_yk * cos(ik) - zk * rate_ik) * sin(Omegak) + rateof_xk * cos(Omegak);
            double rate_Yk =
                    xk * rate_Omegak + (rateof_yk * cos(ik) - zk * rate_ik) * cos(Omegak) + rateof_xk * sin(Omegak);
            double rate_Zk = rateof_yk * sin(ik) + yk * rate_ik * cos(ik);
            double XYZ_right[3] = {rate_Xk, rate_Yk, rate_Zk};
            Matrix new_right(3, 1, XYZ_right);
            double m_right[4] = {rateof_xk, rateof_yk, rate_Omegak, rate_ik};
            Matrix r_right(4, 1, m_right);
            double R[12] = {cos(Omegak), -sin(Omegak) * cos(ik), -(_xk * sin(Omegak) + _yk * cos(Omegak) * cos(ik)),
                            _yk * sin(Omegak) * sin(ik), sin(Omegak), cos(Omegak) * cos(ik),
                            _xk * cos(Omegak) - _yk * sin(Omegak) * cos(ik), _yk * cos(Omegak) * sin(ik),
                            0, sin(ik), 0, _yk * cos(ik)};
            Matrix rate_R(3, 4, R);
            double rate_Rz[9] = {-sin(Omega_BDS * tk), cos(Omega_BDS * tk), 0,
                                 -cos(Omega_BDS * tk), -sin(Omega_BDS * tk), 0,
                                 0, 0, 0};
            Matrix rate_of_Rz(3, 3, rate_Rz);
            rate_of_Rz = rate_of_Rz ^ Omega_BDS;
            rate_xyz = rate_of_Rz * Rx * R_right + Rz * Rx * new_right;
        } else {// MEO/IGSO卫星坐标求法
            Omegak = eph[i].Omega0 + (eph[i].det_Omega - Omega_BDS) * tk - Omega_BDS * eph[i].toe;
            xk = _xk * cos(Omegak) - _yk * cos(ik) * sin(Omegak);
            yk = _xk * sin(Omegak) + _yk * cos(ik) * cos(Omegak);
            zk = _yk * sin(ik);
            // BDS卫星运动速度计算
            rate_Ek = n / (1 - e * cos(Ek));
            double rate_Phik = sqrt((1 + e) / (1 - e)) * pow(cos(Vk / 2) / cos(Ek / 2), 2) * rate_Ek;
            double rate_uk = 2 * (eph[i].cus * cos(2 * Phik) - eph[i].cuc * sin(2 * Phik)) * rate_Phik + rate_Phik;
            double rate_rk = A * e * sin(Ek) * rate_Ek +
                             2 * (eph[i].crs * cos(2 * Phik) - eph[i].crc * sin(2 * Phik)) * rate_Phik;
            double rate_ik = eph[i].IDOT + 2 * (eph[i].cis * cos(2 * Phik) - eph[i].cic * sin(2 * Phik)) * rate_Phik;
            double rate_Omegak = eph[i].det_Omega - Omega_BDS;
            double R[12] = {cos(Omegak), -sin(Omegak) * cos(ik), -(_xk * sin(Omegak) + _yk * cos(Omegak) * cos(ik)),
                            _yk * sin(Omegak) * sin(ik), sin(Omegak), cos(Omegak) * cos(ik),
                            _xk * cos(Omegak) - _yk * sin(Omegak) * cos(ik), _yk * cos(Omegak) * sin(ik),
                            0, sin(ik), 0, _yk * cos(ik)};
            Matrix rate_R(3, 4, R);
            double rateof_xk = rate_rk * cos(uk) - rk * rate_uk * sin(uk);
            double rateof_yk = rate_rk * sin(uk) + rk * rate_uk * cos(uk);
            double m_right[4] = {rateof_xk, rateof_yk, rate_Omegak, rate_ik};
            Matrix right(4, 1, m_right);

            double rate_Xk =
                    -yk * rate_Omegak - (rateof_yk * cos(ik) - zk * rate_ik) * sin(Omegak) + rateof_xk * cos(Omegak);
            double rate_Yk =
                    xk * rate_Omegak + (rateof_yk * cos(ik) - zk * rate_ik) * cos(Omegak) + rateof_xk * sin(Omegak);
            double rate_Zk = rateof_yk * sin(ik) + yk * rate_ik * cos(ik);
            double XYZ_right[3] = {rate_Xk, rate_Yk, rate_Zk};

            rate_xyz = rate_R * right;
        }
        // 计算卫星钟差、钟速
        double dt = bt.second - eph[i].toc;
        if (dt > 302400) dt = dt - 604800;
        else if (dt < -302400) dt = dt + 604800;
        double F = -2 * sqrt(GM_BDS) / pow(C_Light, 2.0);
        double dtr = F * e * eph[i].RootA * sin(Ek);
        double Clock_error = eph[i].a0 + eph[i].a1 * dt + eph[i].a2 * dt * dt + dtr;  //相对论效应改正、对于双频组合观测值而言，不需要群延差改正
        double rate_dtr = F * e * eph[i].RootA * cos(Ek) * rate_Ek;
        double v_clock_error = eph[i].a1 + 2 * eph[i].a2 * dt + rate_dtr;
        // 赋值
        info->BDS[i].Coordinate.X = xk;
        info->BDS[i].Coordinate.Y = yk;
        info->BDS[i].Coordinate.Z = zk;
        info->BDS[i].Vx = rate_xyz(1, 1);
        info->BDS[i].Vy = rate_xyz(2, 1);
        info->BDS[i].Vz = rate_xyz(3, 1);
        info->BDS[i].clock_error = Clock_error;
        info->BDS[i].v_clock_error = v_clock_error;
        info->BDS[i].flag = true;
    }
    return 1;
}

GPST calculate_ts(GPST tr, double P, double dts) {
    tr.minus_sec(P / C_Light + dts);
    return tr;
}

// 打印星历文件解算结果
void print_info(GNSSINFO *info) {
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(6);
    // 打印GPS卫星星历解算结果
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if (info->GPS[i].flag) {
            std::cout << "BG" << i + 1 << " ";
            std::cout << info->GPS[i].Coordinate.X << " " << info->GPS[i].Coordinate.Y << " "
                      << info->GPS[i].Coordinate.Z << " ";
            std::cout << info->GPS[i].Vx << " " << info->GPS[i].Vy << " " << info->GPS[i].Vz << " ";
            std::cout << info->GPS[i].clock_error * 1e6 << " " << info->GPS[i].v_clock_error * 1e6<<"  " ;
            std::cout<<deg(info->GPS[i].E)<<'\n';
        }
    }
    // 打印BDS卫星星历解算结果
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if (info->BDS[i].flag) {
            std::cout << "BC" << i + 1 << " ";
            std::cout << info->BDS[i].Coordinate.X << " " << info->BDS[i].Coordinate.Y << " "
                      << info->BDS[i].Coordinate.Z << " ";
            std::cout << info->BDS[i].Vx << " " << info->BDS[i].Vy << " " << info->BDS[i].Vz << " ";
            std::cout << info->BDS[i].clock_error * 1e6 << " " << info->BDS[i].v_clock_error * 1e6<<"  " ;
            std::cout<<deg(info->BDS[i].E)<<'\n';
        }
    }
}

void GNSSINFO::calculate_tr(OBS *obs, GPSEPH *gpseph, BDSEPH *bdseph) {
    GPST tr = obs->t, G_ts[GPSMAXPRN], C_ts[BDSMAXPEN]; //记录接收机接收时刻,信号发射时刻
    double G_pif[GPSMAXPRN], C_pif[BDSMAXPEN]; //记录双频伪距观测值
    obs->get_GPS_PIF(obs->G_prn, obs->G_num, G_pif);
    obs->get_BDS_PIF(obs->C_prn, obs->C_num, C_pif, bdseph, this);

    // 计算卫星发射时刻(此处手动迭代一次)
    for (int i = 0; i < obs->G_num; ++i) {
        G_ts[obs->G_prn[i]] = calculate_ts(tr, G_pif[obs->G_prn[i]], 0);
    }
    for (int i = 0; i < obs->C_num; ++i) {
        C_ts[obs->C_prn[i]] = calculate_ts(tr, C_pif[obs->C_prn[i]], 0);
    }
    GPS_POS(this, gpseph, G_ts);
    BDS_POS(this, bdseph, C_ts);
    for (int i = 0; i < obs->G_num; ++i) {
        G_ts[obs->G_prn[i]] = calculate_ts(tr, G_pif[obs->G_prn[i]], GPS[obs->G_prn[i]].clock_error);
    }
    for (int i = 0; i < obs->C_num; ++i) {
        C_ts[obs->C_prn[i]] = calculate_ts(tr, C_pif[obs->C_prn[i]], BDS[obs->C_prn[i]].clock_error);
    }
    GPS_POS(this, gpseph, G_ts);
    BDS_POS(this, bdseph, C_ts);

}

void GNSSINFO::calculate_rotation(double G_dtr, double C_dtr, const double *G_pif, const double *C_pif) {
    //注意 接收机钟差的单位为m
    double g_dtr = G_dtr / C_Light, c_dtr = C_dtr / C_Light;
    for (int i = 0; i < G_num; ++i) {
        double sum = G_pif[G_prn[i]] / C_Light + GPS[G_prn[i]].clock_error;
        double dt = sum - g_dtr;
        double d_theta = Omega_WGS * dt;
        double dx = d_theta * GPS[G_prn[i]].Coordinate.Y;
        double dy = -d_theta * GPS[G_prn[i]].Coordinate.X;
        // 更新卫星位置
        GPS[G_prn[i]].Coordinate.X += dx;
        GPS[G_prn[i]].Coordinate.Y += dy;
    }
    for (int i = 0; i < C_num; ++i) {
        double sum = C_pif[C_prn[i]] / C_Light + BDS[C_prn[i]].clock_error;
        double dt = sum - c_dtr;
        double d_theta = Omega_BDS * dt;
        double dx = d_theta * BDS[C_prn[i]].Coordinate.Y;
        double dy = -d_theta * BDS[C_prn[i]].Coordinate.X;
        // 更新卫星位置
        BDS[C_prn[i]].Coordinate.X += dx;
        BDS[C_prn[i]].Coordinate.Y += dy;
    }
}




