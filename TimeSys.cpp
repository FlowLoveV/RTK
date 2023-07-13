//
// Created by 0-0 mashuo on 2023/3/6.
//

#include <iomanip>
#include "iostream"
#include "TimeSys.h"

CommonTime::CommonTime() : year(1949), month(10), day(1), hour(8), minute(0), second(0.0) {}

CommonTime::CommonTime(unsigned short year, unsigned short month, unsigned short day, unsigned short hour,
                       unsigned short minute, double second) {
    this->year = year;
    this->month = month;
    this->day = day;
    this->hour = hour;
    this->minute = minute;
    this->second = second;
}

CommonTime &CommonTime::operator=(const CommonTime &b) {
    if (this == &b)
        return *this;
    this->year = b.year;
    this->month = b.month;
    this->day = b.day;
    this->hour = b.hour;
    this->minute = b.minute;
    this->second = b.second;
    return *this;
}

void CommonTime::print() {
    std::cout << "CT:" << std::setprecision(15) << year << " " << month << " " << day << " " << hour << " "
              << minute << " " << std::setprecision(15) << second << " " << '\n';
}


JD::JD() : days(0), fracdays(0) {}

JD::JD(int days, double fracdays) {
    this->days = days;
    this->fracdays = fracdays;
}

JD &JD::operator=(const JD &b) {
    if (this == &b)
        return *this;
    this->days = b.days;
    this->fracdays = b.fracdays;
    return *this;
}

void JD::print() {
    std::cout << "JD:" << std::setprecision(15) << days << " " << fracdays << '\n';
}


MJD::MJD() : days(0), fracdays(0) {}

MJD::MJD(int days, double fracdays) {
    this->days = days;
    this->fracdays = fracdays;
}

MJD &MJD::operator=(const MJD &b) {
    if (this == &b)
        return *this;
    this->days = b.days;
    this->fracdays = b.fracdays;
    return *this;
}

void MJD::print() {
    std::cout << "MJD:" << std::setprecision(15) << days << " " << fracdays << '\n';
}

GPST::GPST(): weeks(0), second(0) {}

GPST::GPST(unsigned short weeks, double second) {
    this->weeks = weeks;
    this->second = second;
}

GPST &GPST::operator=(const GPST &b) {
    if (this == &b)
        return *this;
    this->weeks = b.weeks;
    this->second = b.second;
    return *this;
}

bool GPST::operator==(const GPST &b) const {
    if (this->weeks==b.weeks && this->second==b.second)
        return true;
    else
        return false;
}

bool GPST::operator!=(const GPST &b) const {
    if (this->weeks==b.weeks && this->second==b.second)
        return false;
    else
        return true;
}

double GPST::operator-(const GPST &b) const {
    unsigned short wk=weeks-b.weeks;
    double sc=second-b.second;
    return wk*604800+sc;
}

void GPST::minus_sec(double sec) {
    double s=second-sec;
    if(s<0) {weeks-=1;second=s+604800;}
    if(s>604800) {weeks+=1;second=s-604800;}
    else {second=s;}
}

void GPST::print() {
    std::cout << "GPST:" << std::setprecision(15) << weeks << " " << second << '\n';
}


DOY::DOY() : year(2022), day(1), fracday(0) {}

DOY::DOY(short year, short day, double fracday){
    this->year = year;
    this->day = day;
    this->fracday = fracday;
}

DOY &DOY::operator=(const DOY &b) {
    if (this == &b)
        return *this;
    this->year = b.year;
    this->day = b.day;
    this->fracday = b.fracday;
    return *this;
}

void DOY::print() {
    std::cout << std::setprecision(15) << "DOY:" << year << " " << day << " " << fracday << '\n';
}

BDST::BDST() : weeks(0), second(0) {}

BDST::BDST(unsigned short weeks, double second) {
    this->weeks = weeks;
    this->second = second;
}

BDST &BDST::operator=(const BDST &b) {
    if (this == &b)
        return *this;
    this->weeks = b.weeks;
    this->second = b.second;
    return *this;
}

double BDST::operator-(const BDST &b) const {
    unsigned short wk=weeks-b.weeks;
    double sc=second-b.second;
    return wk*604800+sc;
}

void BDST::print() {
    std::cout << std::setprecision(15) << "BDST" << weeks << " " << second << '\n';
}

MJD JD2MJD(const JD t) {
    int days;
    double fracdays;
    if (t.fracdays >= 0.5) {
        days = t.days - 2400000;
        fracdays = t.fracdays - 0.5;
    } else {
        days = t.days - 2400001;
        fracdays = t.fracdays + 0.5;
    }
    MJD tout(days, fracdays);
    return tout;
}

// 简化儒略日转儒略日
JD MJD2JD(MJD t) {
    int days;
    double fracdays;
    if (t.fracdays <= 0.5) {
        days = t.days + 2400000;
        fracdays = t.fracdays + 0.5;
    } else {
        days = t.days + 2400001;
        fracdays = t.fracdays - 0.5;
    }
    JD tout(days, fracdays);
    return tout;
}


// 通用时转简化儒略日
MJD Common2MJD(const CommonTime t) {
    int y, m;
    if (t.month <= 2) {
        y = t.year - 1;
        m = t.month + 12;
    } else {
        y = t.year;
        m = t.month;
    }
    double hour = t.hour, minute = t.minute, second = t.second;
    double UT = hour + minute / 60 + second / 3600;
    int days = int(365.25 * y) + int(30.6001 * (m + 1)) + t.day - 679019;
    double fracdays = UT / 24;
    MJD tout(days, fracdays);
    return tout;
}

// 简化儒略日转通用时
CommonTime MJD2Common(const MJD t) {
    int a = int(t.days + t.fracdays + 2400001.0);
    int b = a + 1537;
    int c = int((b - 122.1) / 365.25);
    int d = int(365.25 * c);
    int e = int((b - d) / 30.6001);
    unsigned short D = b - d - int(30.6001 * e);
    double fracD = t.fracdays;
    unsigned short hour = int(fracD * 24);
    unsigned short minute = int(fracD * 24 * 60) - hour * 60;
    double second = fracD * 24 * 3600 - hour * 3600 - minute * 60;
    unsigned short M = e - 1 - 12 * int(e / 14);
    unsigned short Y = c - 4715 - int((7 + M) / 10);
    CommonTime tout(Y, M, D, hour, minute, second);
    return tout;
}


// 简化儒略日转为GPST
GPST MJD2GPST(const MJD t) {
    int weeks = int((t.days + t.fracdays - 44244) / 7);
    double second = (t.days - 44244 - weeks * 7) * 86400.0 + t.fracdays * 86400.0;
    GPST tout(weeks, second);
    return tout;
}

// GPST转简化儒略日
MJD GPST2MJD(const GPST t) {
    int day = 44244 + t.weeks * 7 + int(t.second / 86400.0);
    double fracdays = t.second / 86400.0 - int(t.second / 86400.0);
    MJD tout(day, fracdays);
    return tout;
}


// 通用时转为GPST
GPST Common2GPST(const CommonTime t) {
    MJD t1 = Common2MJD(t);
    GPST tout = MJD2GPST(t1);
    return tout;
}

// GPST转为通用时
CommonTime GPST2Common(const GPST t) {
    MJD t1 = GPST2MJD(t);
    CommonTime tout = MJD2Common(t1);
    return tout;
}


// 通用时转年积日
DOY Common2DOY(const CommonTime t) {
    int month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int day = 0;
    if (t.year % 4 == 0 && (t.year % 100 != 0 || t.year % 400 == 0)) {
        month[1] = 29; //闰年
    }
    for (int i = 0; i < t.month - 1; ++i) {
        day = day + month[i];
    }
    double hour = t.hour, min = t.minute;
    double fracday = hour / 24 + min / (60 * 24) + t.second / 86400;
    DOY tout(t.year, day + t.day, fracday);
    return tout;
}

// 年积日转通用时
CommonTime DOY2Common(const DOY t) {
    int month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (t.year % 4 == 0 && (t.year % 100 != 0 || t.year % 400 == 0)) {
        month[1] = 29; //闰年
    }
    int m = 0, d = 0, count = t.day;
    for (int i = 0; i < 12; ++i) {
        count -= month[i];
        m += 1;
        if (count > 0)
            continue;
        d = count + month[i];
        break;
    }
    int hour = int(t.fracday * 24);
    int minute = int(t.fracday * 24 * 60) - hour * 60;
    double second = t.fracday * 86400 - hour * 3600 - minute * 60;
    CommonTime tout(t.year, m, d, hour, minute, second);
    return tout;
}


// DOY转GPST
GPST DOY2GPST(const DOY t) {
    CommonTime t1 = DOY2Common(t);
    GPST tout = Common2GPST(t1);
    return tout;
}

// GSPT转DOY
DOY GPST2DOY(const GPST t) {
    CommonTime t1 = GPST2Common(t);
    DOY tout = Common2DOY(t1);
    return tout;
}


// GPST转BDST
BDST GPST2BDST(const GPST t) {
    BDST tout(t.weeks - 1356, t.second - 14);
    if(tout.second<0) {
        tout.weeks -=1;
        tout.second += 604800;
    }
    return tout;
}

// BDST转GPST
GPST BDST2GPST(const BDST t) {
    GPST tout(t.weeks + 1356, t.second + 14);
    return tout;
}
