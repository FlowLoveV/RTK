//
// Created by 0-0 mashuo on 2023/3/27.
//

#include "receive_port.h"
#include "readB.h"
#include "Positioning.h"
#include "RelativePositioning.h"
#include <unistd.h>


int spp_port()
{
    int client_sockfd;
    int msg_len;
    struct sockaddr_in remote_addr;                  //服务器端网络地址结构体
    uint8_t msg_buf[3*MAXMESSAGELEN];                //数据传送的缓冲区
    uint8_t data_buf[10*MAXROWLEN];                  //数据暂存区,长度定为2*MAXROWLEN,避免memcpy碰到相连的内存
    memset(&remote_addr,0,sizeof(remote_addr));   //数据初始化--清零
    remote_addr.sin_family=AF_INET;                  //设置为IP通信
    remote_addr.sin_addr.s_addr=inet_addr(Setting_IP);//服务器IP地址
    int port = std::stoi(std::string(Setting_Port));
    remote_addr.sin_port=htons(port);                //服务器端口号

    /*创建客户端套接字--IPv4协议，面向连接通信，TCP协议*/
    if((client_sockfd=socket(PF_INET,SOCK_STREAM,0))<0)
    {
        perror("本机未连接网络!\n");
        return 1;
    }
    /*将套接字绑定到服务器的网络地址上*/
    if(connect(client_sockfd,(struct sockaddr *)&remote_addr,sizeof(struct sockaddr))<0)
    {
        perror("无法连接指定服务器网络!\n");
        return 1;
    }
    printf("已连接IP:%s\t 端口:%s\n",Setting_IP,Setting_Port);
    int lenData = 0;
    int length = 0;  // 本次读取消息长度
    auto *header = new HeaderB;
    auto *gps_eph = new GPSEPH[GPSMAXPRN];
    auto *bds_eph = new BDSEPH[BDSMAXPEN];
    auto *pos = new ReciverPos;
    auto *obs = new OBS;
    /*循环的发送接收信息并打印接收信息（可以按需发送）--recv返回接收到的字节数，send返回发送的字节数*/
    // 首先捕获文件头
    std::string filename(OUTPUT_FILE_DIRECTORY);
    filename += "binary.log";
    while(1){
        if((msg_len = recv(client_sockfd,msg_buf,3*MAXMESSAGELEN,0)) > 0){
            if(is_header(msg_buf,length)) {
                memcpy(data_buf+lenData,msg_buf,msg_len);
                lenData += msg_len;
                AppendBinaryFile(filename,msg_buf,msg_len);
                std::cout<<"消息头捕获成功!\n";
                break;
            }
        }
    }
    while(1){
        usleep(980000);
        if( (msg_len = recv(client_sockfd,msg_buf,3*MAXMESSAGELEN,0)) > 0){
            AppendBinaryFile(filename,msg_buf,msg_len);
            // 上个历元未捕获文件头
            if(lenData+msg_len>10*MAXROWLEN) {
                lenData=0;
                memset(data_buf,0x0,10*MAXROWLEN);
            }
            // 将新消息补充到尾部
            memcpy(data_buf+lenData,msg_buf,msg_len);
            lenData += msg_len;
            // 记录读取偏移量
            int offset = 0;
            while(1){
                if(lenData - offset <= 7) break;
                int flag = port_read_station(data_buf+offset,obs,gps_eph,bds_eph,header,length);
                if(flag==0 && (lenData - offset) < length){
                    break;
                }
                offset += length;  // 偏移量增加
                if(flag == 1){
                    // 读取到观测值，进行单点定位
                    auto *info = new GNSSINFO;
                    if(PIF_location(obs,gps_eph,bds_eph,pos,info)){
                        pos->print();// 结果打印
                    }
                    delete info;
                }
            }
            // offset=0，则本历元消息长度不够，继续读取
            if(offset==0)
                continue;
            // 拼接剩余消息
            uint8_t temp_buf[lenData-offset];
            memcpy(temp_buf,data_buf+offset,lenData-offset);
            memset(data_buf,0x0,lenData); // 先把所有数据擦除s
            memcpy(data_buf,temp_buf,lenData-offset);
            lenData = lenData - offset;
        }
        else std::cout<<"网络端口无数据!\n";
    }
    /*关闭套接字*/
    close(client_sockfd);
    return 0;
}

int rtk_port() {
    int client_sockfd_base, client_sockfd_mobile;
    int msg_len_base, msg_len_mobile;
    struct sockaddr_in remote_addr_base;             //基准战服务器端网络地址结构体
    struct sockaddr_in remote_addr_mobile;           //流动站服务器端网络地址结构体
    //数据传送的缓冲区
    uint8_t msg_buf_base[2 * MAXMESSAGELEN], msg_buf_mobile[2 * MAXROWLEN];
    //数据暂存区,长度定为10*MAXROWLEN,避免memcpy碰到相连的内存
    uint8_t data_buf_base[10 * MAXROWLEN], data_buf_mobile[10 * MAXROWLEN];
    //数据初始化--清零
    memset(&remote_addr_base, 0, sizeof(remote_addr_base));
    memset(&remote_addr_mobile, 0, sizeof(remote_addr_mobile));
    //基准战网口设置
    remote_addr_base.sin_family = AF_INET;                   //设置为IP通信
    remote_addr_base.sin_addr.s_addr = inet_addr(Setting_IP);//服务器IP地址
    int port = std::stoi(std::string(Setting_Port));
    remote_addr_base.sin_port = htons(port);                 //服务器端口号
    //流动站网口设置
    remote_addr_mobile.sin_family = AF_INET;                  //设置为IP通信
    remote_addr_mobile.sin_addr.s_addr = inet_addr(Setting_IP2);//服务器IP地址
    int port2 = std::stoi(std::string(Setting_Port2));
    remote_addr_mobile.sin_port = htons(port2);                //服务器端口号

    /*创建客户端套接字--IPv4协议，面向连接通信，TCP协议*/
    if ((client_sockfd_base = socket(PF_INET, SOCK_STREAM, 0)) < 0 ||
        (client_sockfd_mobile = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        perror("本机未连接网络!\n");
        return 1;
    }
    /*将基准战套接字绑定到基准战服务器的网络地址上*/
    if (connect(client_sockfd_base, (struct sockaddr *) &remote_addr_base, sizeof(struct sockaddr)) < 0) {
        perror("无法连接基准站网络端口!\n");
        return 1;
    }
    printf("已连接基准站网络端口 ip:%s port:%s\n", Setting_IP, Setting_Port);
    /*将基准战套接字绑定到基准战服务器的网络地址上*/
    if (connect(client_sockfd_mobile, (struct sockaddr *) &remote_addr_mobile, sizeof(struct sockaddr)) < 0) {
        perror("无法连接流动站网络端口!\n");
        return 1;
    }
    printf("已连接流动站网络端口 ip:%s port:%s\n", Setting_IP2, Setting_Port2);

    int length_base = 0, length_mobile = 0;
    int lenData_base = 0, lenData_mobile = 0;
    // 创建两个 观测、星历、卫星信息、定位结果结构体
    auto *base_header = new HeaderB;
    auto *mobile_header = new HeaderB;
    auto *base_gpseph = new GPSEPH[GPSMAXPRN];
    auto *mobile_gpseph = new GPSEPH[GPSMAXPRN];
    auto *base_bdseph = new BDSEPH[BDSMAXPEN];
    auto *mobile_bdseph = new BDSEPH[BDSMAXPEN];
    auto *base_pos = new ReciverPos;
    auto *mobile_pos = new ReciverPos;
    OBS *base_obs = new OBS;
    OBS *mobile_obs = new OBS;
    double float_res = 0;
    double fixed_res = 0;
    // 观测判断
    std::vector<OBS> obs_base_vector;
    std::vector<OBS> obs_mobile_vector;
    bool if_receive_obs[2] = {false,false};

    // 创建文件
    std::string filename(OUTPUT_FILE_DIRECTORY);
    std::string filename_base = filename + "Base.log";
    std::string filename_mobile = filename + "Move.log";
    // 首先捕获文件头
    bool header_base = false, header_mobile = false;
    while (1) {
        if ((msg_len_base = recv(client_sockfd_base, msg_buf_base, MAXMESSAGELEN, 0)) > 0) {
            if (is_header(msg_buf_base, length_base)) {
                memcpy(data_buf_base, msg_buf_base, msg_len_base);
                lenData_base += msg_len_base;
                AppendBinaryFile(filename_base,msg_buf_base,msg_len_base);
                std::cout<<"基准站消息头捕获成功!\n";
                header_base = true;
            }
        }
        if ((msg_len_mobile = recv(client_sockfd_mobile, msg_buf_mobile, MAXMESSAGELEN, 0)) > 0) {
            if (is_header(msg_buf_mobile, length_mobile)) {
                memcpy(data_buf_mobile, msg_buf_mobile, msg_len_mobile);
                lenData_mobile += msg_len_mobile;
                AppendBinaryFile(filename_mobile,msg_buf_mobile,msg_len_mobile);
                std::cout<<"流动站消息头捕获成功!\n";
                header_mobile = true;
            }
        }
        if (header_base && header_mobile) break;
    }

    // 开始读取数据
    while (1) {
        usleep(980000);
        // 读取基准站数据
        if ((msg_len_base = recv(client_sockfd_base, msg_buf_base, 2 * MAXMESSAGELEN, 0)) > 0) {
            // 添加文件
            AppendBinaryFile(filename_base,msg_buf_base,msg_len_base);
            // 上个历元未捕获文件头
            if (lenData_base + msg_len_base > 10 * MAXROWLEN) {
                lenData_base = 0;
                memset(data_buf_base, 0x0, 10 * MAXROWLEN);
            }
            // 将新消息补充到尾部
            memcpy(data_buf_base + lenData_base, msg_buf_base, msg_len_base);
            lenData_base += msg_len_base;
            int offset = 0;
            while (1) {
                if(length_base - offset <= 7) break;
                int flag = port_read_station(data_buf_base + offset, base_obs, base_gpseph, base_bdseph, base_header,
                                             length_base);
                if (flag == 0 && (lenData_base - offset) < length_base) {
                    break;
                }
                offset += length_base;  // 偏移量增加
                if(flag == 1){
                    obs_base_vector.push_back(*base_obs);
                    if_receive_obs[0] = true;
                }
            }
            // offset=0，则本历元消息长度不够，继续读取
            if (offset == 0)
                continue;
            // 拼接剩余消息
            uint8_t temp_buf[lenData_base - offset];
            memcpy(temp_buf, data_buf_base + offset, lenData_base - offset);
            memset(data_buf_base, 0x0, lenData_base); // 先把所有数据擦除
            memcpy(data_buf_base, temp_buf, lenData_base - offset);
            lenData_base = lenData_base - offset;
        } else std::cout << "基准站网络端口无数据!\n";

        // 读取流动站数据
        if ((msg_len_mobile = recv(client_sockfd_mobile, msg_buf_mobile, 2 * MAXMESSAGELEN, 0)) > 0) {
            AppendBinaryFile(filename_mobile,msg_buf_mobile,msg_len_mobile);
            // 上个历元未捕获文件头
            if (lenData_mobile + msg_len_mobile > 10 * MAXROWLEN) {
                lenData_mobile = 0;
                memset(data_buf_mobile, 0x0, 10 * MAXROWLEN);
            }
            // 将新消息补充到尾部
            memcpy(data_buf_mobile + lenData_mobile, msg_buf_mobile, msg_len_mobile);
            lenData_mobile += msg_len_mobile;
            int offset = 0;
            while (1) {
                if(lenData_mobile - offset <= 7) break;
                int flag = port_read_station(data_buf_mobile + offset, mobile_obs, mobile_gpseph, mobile_bdseph,
                                             mobile_header, length_mobile);
                if (flag == 0 && (lenData_mobile - offset) < length_mobile) {
                    break;
                }
                offset += length_mobile;  // 偏移量增加
                if(flag == 1){
                    obs_mobile_vector.push_back(*mobile_obs);
                    if_receive_obs[1] = true;
                }
            }
            // offset=0，则本历元消息长度不够，继续读取
            if (offset == 0)
                continue;
            // 拼接剩余消息
            uint8_t temp_buf[lenData_mobile - offset];
            memcpy(temp_buf, data_buf_mobile + offset, lenData_mobile - offset);
            memset(data_buf_mobile, 0x0, lenData_mobile); // 先把所有数据擦除
            memcpy(data_buf_mobile, temp_buf, lenData_mobile - offset);
            lenData_mobile = lenData_mobile - offset;
        } else std::cout << "网络端口无数据!\n";

        // 时间对齐判断
        if (if_receive_obs[0] && if_receive_obs[1]){
            while(!TimeAlignment(obs_base_vector[0].t,obs_mobile_vector[0].t) && !obs_base_vector.empty() && !obs_mobile_vector.empty()){
                // 对齐观测值向量
                double dt = obs_base_vector[0].t - obs_mobile_vector[0].t;
                if (dt < -0.01) obs_base_vector.erase(obs_base_vector.begin());
                else if (dt > 0.01) obs_mobile_vector.erase(obs_mobile_vector.begin());
            }
            // 当观测值向量均不为空时，开始相对定位
            while (!obs_base_vector.empty() && !obs_mobile_vector.empty() && TimeAlignment(obs_base_vector[0].t,obs_mobile_vector[0].t)) {
                // 单点定位
                auto *base_info = new GNSSINFO;
                auto *mobile_info = new GNSSINFO;
                PIF_location(&obs_base_vector[0], base_gpseph, base_bdseph, base_pos, base_info);
                PIF_location(&obs_mobile_vector[0], mobile_gpseph, mobile_bdseph, mobile_pos, mobile_info);
                // 单点定位成功才可以进行RTK
                if (base_pos->known && mobile_pos->known) {
                    Station base(obs_base_vector[0], *base_gpseph, *base_bdseph, *base_info, *base_pos);
                    Station mobile(obs_mobile_vector[0], *mobile_gpseph, *mobile_bdseph, *mobile_info, *mobile_pos);
                    RelativeStation bm(base, mobile);
                    bm.LS();
                    if (bm.fixed) fixed_res += 1;
                    else float_res += 1;
                }
                // 定位一次后，删除此历元观测数据
                obs_base_vector.erase(obs_base_vector.begin());
                obs_mobile_vector.erase(obs_mobile_vector.begin());
            }
            if_receive_obs[0] = if_receive_obs[1] = false;
        }
    }
}


void AppendBinaryFile(const std::string& filename, const uint8_t* data, size_t size) {
    std::ofstream file(filename, std::ios::binary | std::ios::app);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    std::vector<uint8_t> binaryData(data, data + size);
    file.write(reinterpret_cast<const char*>(binaryData.data()), binaryData.size());
    file.close();
    if (!file) {
        std::cerr << "Error occurred while writing to file: " << filename << std::endl;
    }
}

