//
// Created by 0-0 mashuo on 2023/3/27.
//

#ifndef RTK_RECEIVE_PORT_H
#define RTK_RECEIVE_PORT_H

// 此文件用于实现网络端口通讯
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include "setting_map.h"
#include "readB.h"
#include "setting_map.h"

#define MAXMESSAGELEN 16384


int spp_port();

int rtk_port();

void AppendBinaryFile(const std::string& filename, const uint8_t* data, size_t size);




#endif //RTK_RECEIVE_PORT_H
