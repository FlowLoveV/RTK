//
// Created by 0-0 mashuo on 2023/3/20.
//

#ifndef RTK_SETTING_MAP_H
#define RTK_SETTING_MAP_H

/* 此文件记录配置文件的配置数据
 * 具体配置可见配置文件说明
 *
 *
 */
#include "iostream"

extern int Solution_mode;                       //  0=PPP      1=RTK
extern int Data_source;                         //  0=Bfile    1=port
extern int Variance_model;                      //  1为高度角模型   2为信噪比模型  3为等权模型
extern int Variance_ratio;                      //  伪距方差 / 载波方差  的比值
extern char Satellite_system_frequency[100];    //  参与RTK解算的配置                      ** 每段载波长度为3，最多可容纳33段载波，足用
extern char file_directory1[260];               //  文件路径1---移动站或者单点定位数据文件路径 **win10系统对路径字符串长度限制为260
extern char file_directory2[260];               //  文件路径2---基站数据文件路径
extern char OUTPUT_FILE_DIRECTORY[260];         //  输入结果文件路径
extern char Setting_IP[63];                     //  网络端口1ip地址                         ** IPV6最长字符串长度为63，IPV4为15
extern char Setting_Port[5];                    //  ip端口1                                ** 每个ip地址最多有2^16=65536个端口
extern char Setting_IP2[63];                    //  网络端口2ip地址                         ** IPV6最长字符串长度为63，IPV4为15
extern char Setting_Port2[5];                   //  ip端口2                                ** 每个ip地址最多有2^16=65536个端口
extern int ELEVATION_LIMIT;                     //  高度角限制（deg)
extern int SNR_LIMIT ;                          //  信噪比限制
extern char PROCESS_FILE_PATH[260];             //  数据处理中间文件路径
extern int RATIO_LIMIT;                         //   ratio阈值





#endif //RTK_SETTING_MAP_H
