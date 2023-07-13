//
// Created by 0-0 mashuo on 2023/3/6.
//

#ifndef RTK_TIMESYS_H
#define RTK_TIMESYS_H


// 通用时
struct CommonTime {
    unsigned short year;
    unsigned short month;
    unsigned short day;
    unsigned short hour;
    unsigned short minute;
    double second;

    // init
    CommonTime();

    CommonTime(unsigned short year, unsigned short month, unsigned short day, unsigned short hour,
               unsigned short minute, double second);

    // 赋值运算符重载
    CommonTime &operator=(const CommonTime &b);

    // 打印
    void print();
};

// 儒略日
struct JD {
    int days;
    double fracdays;

    //init
    JD();

    JD(int days, double fracdays);

    // 赋值重载
    JD &operator=(const JD &b);

    void print();
};

// 简化儒略日
struct MJD {
    int days;
    double fracdays;

    //init
    MJD();

    MJD(int days, double fracdays);

    MJD &operator=(const MJD &b);

    void print();
};

// GPST周秒
struct GPST {
    unsigned short weeks;
    double second;

    //init
    GPST();

    GPST(unsigned short weeks, double second);

    //赋值运算符
    GPST &operator=(const GPST &b);

    // 判断是否相等运算符
    bool operator==(const GPST &b) const;

    bool operator!=(const GPST &b) const;

    // 定义减号运算符(这种运算只能用于简单判断两个GPST的时间差大小，不可用于精密计算)
    double operator-(const GPST &b) const;

    // 定义GPST的运算,减去某个周内秒(可为负值)(也即减去后周内秒范围在-604800-2*604800内)后的时间
    void minus_sec(double sec);

    void print();
};

// 年积日
struct DOY {
    unsigned short year;
    unsigned short day;
    double fracday;

    //init
    DOY();

    DOY(short year, short day, double fracday);

    DOY &operator=(const DOY &b);

    void print();
};

// BDST周秒
struct BDST {
    unsigned short weeks;
    double second;

    //init
    BDST();

    BDST(unsigned short weeks, double second);

    // 赋值运算符
    BDST &operator=(const BDST &b);

    // 减号运算符
    double operator-(const BDST &b) const;

    void print();
};

// 儒略日转简化儒略日
MJD JD2MJD(const JD t);

// 简化儒略日转儒略日
JD MJD2JD(MJD t);


// 通用时转简化儒略日
MJD Common2MJD(const CommonTime t);

// 简化儒略日转通用时
CommonTime MJD2Common(const MJD t);


// 简化儒略日转为GPST
GPST MJD2GPST(const MJD t);

// GPST转简化儒略日
MJD GPST2MJD(const GPST t);


// 通用时转为GPST
GPST Common2GPST(const CommonTime t);

// GPST转为通用时
CommonTime GPST2Common(const GPST t);


// 通用时转年积日
DOY Common2DOY(const CommonTime t);

// 年积日转通用时
CommonTime DOY2Common(const DOY t);


// DOY转GPST
GPST DOY2GPST(const DOY t);

// GSPT转DOY
DOY GPST2DOY(const GPST t);


// GPST转BDST
BDST GPST2BDST(const GPST t);

// BDST转GPST
GPST BDST2GPST(const BDST t);


#endif //RTK_TIMESYS_H
