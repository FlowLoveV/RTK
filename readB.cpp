//
// Created by 0-0 mashuo on 2023/3/6.
//

#include <iomanip>
#include "readB.h"
#include "Positioning.h"
#include "RelativePositioning.h"
#include "fstream"
#include "setting_map.h"
#include "BitOperation.h"
#include "setting_map.h"

static uint16_t U2(uint8_t *p) {
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

static uint32_t U4(uint8_t *p) {
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}

static int32_t I4(uint8_t *p) {
    int32_t i;
    memcpy(&i, p, 4);
    return i;
}

static float R4(uint8_t *p) {
    float r;
    memcpy(&r, p, 4);
    return r;
}

static double R8(uint8_t *p) {
    double r;
    memcpy(&r, p, 8);
    return r;
}

// 单个卫星数据读取
static int readsys(uint8_t *buff, OBS *obs) {
    uint32_t status = U4(buff + 40);
    uint32_t signaltype = (status >> 21) & 0x1F;
    uint32_t satsys = (status >> 16) & 7;
    uint16_t PRN = U2(buff) - 1;
    switch (satsys) {
        // sys=GPS
        case 0: {
            switch (signaltype) {
                // signaltype L1C/A
                case 0: {
                    obs->GPS[PRN].C[0] = R8(buff + 4);
                    obs->GPS[PRN].L[0] = -R8(buff + 16);
                    obs->GPS[PRN].D[0] = R4(buff + 28);
                    obs->GPS[PRN].S[0] = R4(buff + 32);
                    return 1;
                }
                    // signaltype L2P/Y
                case 9: {
                    obs->GPS[PRN].C[1] = R8(buff + 4);
                    obs->GPS[PRN].L[1] = -R8(buff + 16);
                    obs->GPS[PRN].D[1] = R4(buff + 28);
                    obs->GPS[PRN].S[1] = R4(buff + 32);
                    return 1;
                }
                default:
                    return 0;
            }
        }
            // sys=BDS
        case 4: {
            if (signaltype == 0 || signaltype == 4) {
                obs->BDS[PRN].C[0] = R8(buff + 4);
                obs->BDS[PRN].L[0] = -R8(buff + 16);
                obs->BDS[PRN].D[0] = R4(buff + 28);
                obs->BDS[PRN].S[0] = R4(buff + 32);
                return 1;
            } else if (signaltype == 2 || signaltype == 6) {
                obs->BDS[PRN].C[1] = R8(buff + 4);
                obs->BDS[PRN].L[1] = -R8(buff + 16);
                obs->BDS[PRN].D[1] = R4(buff + 28);
                obs->BDS[PRN].S[1] = R4(buff + 32);
                return 1;
            } else {
                return 0;
            }
        }
        default:
            return 0;
    }
}

// crc检验函数
unsigned int crc32(const unsigned char *buff, int len) {
    int i, j;
    unsigned int crc = 0;

    for (i = 0; i < len; i++) {
        crc ^= buff[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

// 头文件读取
static bool readHeader(uint8_t *buff, FILE *file, HeaderB *headerB) {
    using namespace std;
    // 文件头判断
    fread(buff, 1, 7, file);
    uint8_t type = *(buff + 6);
    if (!(buff[0] == OEM7SYNC1 && buff[1] == OEM7SYNC2 && buff[2] == OEM7SYNC3) ||
        (type >> 5 & 3) != 0) {
        return false;
    }
    uint8_t HeaderLen = *(buff + 3);
    headerB->len = HeaderLen;
    fread(buff + 7, 1, HeaderLen - 7, file); //将头文件剩余部分读取
    uint16_t MsgLen = U2(buff + 8);
    headerB->MsgLen = MsgLen;
    fread(buff + HeaderLen, 1, MsgLen + 4, file);  //将data和crc校验码读入buff
    // crc检验
    if (crc32(buff, HeaderLen + MsgLen) != U4(buff + HeaderLen + MsgLen)) {
        std::cout << "crc校验失败!\n" ;
        return false;
    }
    // GPS时
    headerB->t.weeks = U2(buff + 14);
    headerB->t.second = 1e-3 * U4(buff + 16);
    // ID
    uint16_t ID = U2(buff + 4);
    headerB->MsgId = ID;
    return true;
}

// 读取观测数据
static bool readObs(uint8_t *buff, HeaderB *headerB, OBS *obs) {
    // 按卫星数读取数据
    uint32_t nums = U4(buff + headerB->len);
    int flag = 0, k;
    obs->t = headerB->t; //记录观测时间
    for (int i = 0; i < nums; ++i) {
        k = readsys(buff + headerB->len + i * 44 + 4, obs);
        flag = flag + k;
    }
    obs->nums = flag; //观测卫星数目
    return true;
}

// 读取星历
static bool readGPSEPH(uint8_t *buff, HeaderB *header, GPSEPH *eph) {
    uint32_t PRN = U4(buff) - 1;
    // health判断
    if (U4(buff + 12) != 0)
        return true;
    else {
        // flag转为true，表示该卫星星历读入
        eph[PRN].flag = true;
        // 读入其他数据
        eph[PRN].tow = R8(buff + 4);
        eph[PRN].IODE1 = U4(buff + 16);
        eph[PRN].IODE2 = U4(buff + 20);
        eph[PRN].week = U4(buff + 24);
        eph[PRN].z_week = U4(buff + 28);
        eph[PRN].toe = R8(buff + 32);
        eph[PRN].A = R8(buff + 40);
        eph[PRN].detN = R8(buff + 48);
        eph[PRN].M0 = R8(buff + 56);
        eph[PRN].ecc = R8(buff + 64);
        eph[PRN].omega = R8(buff + 72);
        eph[PRN].cuc = R8(buff + 80);
        eph[PRN].cus = R8(buff + 88);
        eph[PRN].crc = R8(buff + 96);
        eph[PRN].crs = R8(buff + 104);
        eph[PRN].cic = R8(buff + 112);
        eph[PRN].cis = R8(buff + 120);
        eph[PRN].I0 = R8(buff + 128);
        eph[PRN].det_IO = R8(buff + 136);
        eph[PRN].omega_0 = R8(buff + 144);
        eph[PRN].det_omega = R8(buff + 152);
        eph[PRN].iodc = U4(buff + 160);
        eph[PRN].toc = R8(buff + 164);
        eph[PRN].tgd = R8(buff + 172);
        eph[PRN].a0 = R8(buff + 180);
        eph[PRN].a1 = R8(buff + 188);
        eph[PRN].a2 = R8(buff + 196);
        eph[PRN].N = R8(buff + 208);
    }
    return true;
}

static bool readBDSEPH(uint8_t *buff, HeaderB *header, BDSEPH *eph) {
    uint32_t PRN = U4(buff) - 1;
    // health判断
    if (U4(buff + 16) != 0)
        return true;
    else {
        // flag转为true，表示该卫星位置已知
        eph[PRN].flag = true;
        // 读入其他数据
        eph[PRN].week = U4(buff + 4);
        eph[PRN].tgd1 = R8(buff + 20);
        eph[PRN].tgd2 = R8(buff + 28);
        eph[PRN].AODC = U4(buff + 36);
        eph[PRN].toc = U4(buff + 40);
        eph[PRN].a0 = R8(buff + 44);
        eph[PRN].a1 = R8(buff + 52);
        eph[PRN].a2 = R8(buff + 60);
        eph[PRN].AODE = U4(buff + 68);
        eph[PRN].toe = U4(buff + 72);
        eph[PRN].RootA = R8(buff + 76);
        eph[PRN].ecc = R8(buff + 84);
        eph[PRN].omega = R8(buff + 92);
        eph[PRN].detN = R8(buff + 100);
        eph[PRN].M0 = R8(buff + 108);
        eph[PRN].Omega0 = R8(buff + 116);
        eph[PRN].det_Omega = R8(buff + 124);
        eph[PRN].i0 = R8(buff + 132);
        eph[PRN].IDOT = R8(buff + 140);
        eph[PRN].cuc = R8(buff + 148);
        eph[PRN].cus = R8(buff + 156);
        eph[PRN].crc = R8(buff + 164);
        eph[PRN].crs = R8(buff + 172);
        eph[PRN].cic = R8(buff + 180);
        eph[PRN].cis = R8(buff + 188);
    }
    return true;
}


// 在参数中添加数据结构体，读取过程中赋值，如果读取成功，则返回1
bool read_B_SPP(const char *filename) {
    using namespace std;
    FILE *file;
    if ((file = fopen(filename, "rb")) == NULL) {
        cout << "Cannot open file!";
    }
    rewind(file);
    auto *header = new HeaderB;
    auto *gps_eph = new GPSEPH[GPSMAXPRN];
    auto *bds_eph = new BDSEPH[BDSMAXPEN];
    auto *pos = new ReciverPos;
    auto *obs = new OBS;
    // 循环读取
    while(!feof(file)){
        uint8_t buff[MAXROWLEN];
        int i = read_station(file,buff,obs,gps_eph,bds_eph,header);
        if(i==1){
            auto *info = new GNSSINFO;
            if(PIF_location(obs,gps_eph,bds_eph,pos,info)){
                std::cout<<std::endl;
                pos->print();std::cout << '\n';  // 结果打印
            }
            delete info;
        }
    }
    fclose(file);
    // 释放内存
    delete header;
    delete []gps_eph;
    delete []bds_eph;
    delete pos;
    delete obs;

    return true;
}


// 检验解码观测数据是否正确
void print_obs(OBS obs) {
    CommonTime ct = GPST2Common(obs.t);
    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(3);
    std::cout << ct.year << " " << ct.month << " " << ct.day << " " << ct.hour << " " << ct.minute << " " << ct.second
              << '\n';
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if (obs.GPS[i].C[0] != 0 && obs.GPS[i].C[1] != 0) {
            std::cout << "G" << i + 1 << " ";
            std::cout << obs.GPS[i].C[0] << "    " << obs.GPS[i].L[0] << "    ";
            std::cout << obs.GPS[i].D[0] << "    " << obs.GPS[i].S[0] << "    ";
            std::cout << obs.GPS[i].C[1] << "    " << obs.GPS[i].L[1] << "    ";
            std::cout << obs.GPS[i].D[1] << "    " << obs.GPS[i].S[1] << '\n';
        }
    }
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if (obs.BDS[i].C[0] != 0 && obs.BDS[i].C[1] != 0) {
            std::cout << "C" << i + 1 << " ";
            std::cout << obs.BDS[i].C[0] << "    " << obs.BDS[i].L[0] << "    ";
            std::cout << obs.BDS[i].D[0] << "    " << obs.BDS[i].S[0] << "    ";
            std::cout << obs.BDS[i].C[1] << "    " << obs.BDS[i].L[1] << "    ";
            std::cout << obs.BDS[i].D[1] << "    " << obs.BDS[i].S[1] << '\n';
        }
    }
}

void print_eph(GPSEPH *eph) {
    for (int i = 0; i < GPSMAXPRN; ++i) {
        if (eph[i].flag) {
            std::cout << "G" << i + 1 << "\t\t"<<eph[i].week <<"\t\t"<<eph[i].toe<<'\n';
        }
    }
}

void print_eph(BDSEPH *eph) {
    for (int i = 0; i < BDSMAXPEN; ++i) {
        if (eph[i].flag) {
            std::cout << "C" << i + 1 << "\t\t"<<eph[i].week <<"\t\t"<<eph[i].toe <<'\n';
        }
    }
}

void read_B_RTK(const char *base_file, const char *mobile_file) {
    using namespace std;
    // 打开基准站、流动站数据文件
    FILE *BaseFile, *MobileFile;
    if ((BaseFile = fopen(base_file, "rb")) == NULL) cerr << "can't open base file!";
    if ((MobileFile = fopen(mobile_file, "rb")) == NULL) cerr << "can't open mobile file";
    rewind(BaseFile);
    rewind(MobileFile);
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
    // 创建结果文件
    string filename(OUTPUT_FILE_DIRECTORY);
    // 获取当前时间
    time_t now = time(0);
    // 将 now 转换为 tm 结构
    tm* gmtm = gmtime(&now);
    char buffer[80];
    // 格式化时间
    strftime(buffer, 80, "%m%d-%H-%M-%S", gmtm);
    filename = filename + "/RTKRes" + string(buffer) + ".txt";
    ofstream ResFile(filename);
    // 输入部分解算信息
    ResFile << "########################################\n";
    ResFile << "########################################\n";
    ResFile << "# 解算配置说明： \n";
    ResFile << "# 参与解算的载波： ";
    if(GET_BIT(V_FUNC,0)) ResFile << "GL1";
    if(GET_BIT(V_FUNC,1)) ResFile << "GL2";
    if(GET_BIT(V_FUNC,2)) ResFile << "BI1";
    if(GET_BIT(V_FUNC,3)) ResFile << "BI3";
    ResFile << std::endl;
    ResFile << "# 高度截止角： " << ELEVATION_LIMIT << "°\n";
    ResFile << "# 信噪比截止值： " << SNR_LIMIT << "\n";
    ResFile << "# Ratio阈值： " << RATIO_LIMIT << "\n";
    ResFile << "# 随机模型： ";
    if(Variance_model==1) ResFile << "高度角模型";
    if(Variance_model==2) ResFile << "信噪比模型";
    if(Variance_model==3) ResFile << "等权模型";
    ResFile << std::endl;
    ResFile << "# 伪距载波方差比： " << Variance_ratio << "\n";
    ResFile << "# GPST-Week GPST-Second fixed Ratio dx dy dz mx my mz dn de du mn me mu S mS Sigma2 RMS\n";
    // 开始读取文件
    while (!feof(BaseFile) && !feof(MobileFile)) {
        uint8_t base_buff[MAXROWLEN]; //开辟缓存区
        uint8_t mobile_buff[MAXROWLEN];
        // 一直读取到观测值文件为止
        while (read_station(BaseFile, base_buff, base_obs, base_gpseph, base_bdseph, base_header) != 1) {
            if (feof(BaseFile)) break;
        };
        while (read_station(MobileFile, mobile_buff, mobile_obs, mobile_gpseph, mobile_bdseph, mobile_header) !=
               1) { if (feof(MobileFile)) break; };
        //观测值时间对齐
        if (TimeAlignment(base_obs->t, mobile_obs->t)) {
            goto RTK;
        }
            //观测值时间未对齐
        else {
            double DifTime = base_obs->t - mobile_obs->t;
            while (!TimeAlignment(base_obs->t, mobile_obs->t)) {
                if (DifTime < 0) {
                    while (read_station(BaseFile, base_buff, base_obs, base_gpseph, base_bdseph, base_header) !=
                           1) { if (feof(BaseFile)) break; };
                } else {
                    while (read_station(MobileFile, mobile_buff, mobile_obs, mobile_gpseph, mobile_bdseph,
                                        mobile_header) != 1) { if (feof(MobileFile)) break; };
                }
                if (feof(BaseFile) || feof(MobileFile)) {
                    break;
                }
            }
            if (TimeAlignment(base_obs->t, mobile_obs->t)) {
                goto RTK;
            }
        }
        RTK:
        {
            if(base_obs->t == GPST(2197,95433))
                int a=1;
            // 单点定位
            auto *base_info = new GNSSINFO;
            auto *mobile_info = new GNSSINFO;
            PIF_location(base_obs, base_gpseph, base_bdseph, base_pos, base_info);
            PIF_location(mobile_obs, mobile_gpseph, mobile_bdseph, mobile_pos, mobile_info);
            // 单点定位成功才可以进行RTK
            if (base_pos->known && mobile_pos->known) {
                Station base(*base_obs, *base_gpseph, *base_bdseph, *base_info, *base_pos);
                Station mobile(*mobile_obs, *mobile_gpseph, *mobile_bdseph, *mobile_info, *mobile_pos);
                RelativeStation bm(base, mobile);
                bm.LS();
                if(bm.fixed) fixed_res += 1;
                else float_res += 1;
                // 在这里添加结果输出的代码
                ResFile << bm.base.obs.t.weeks << " " << bm.base.obs.t.second << " ";
                ResFile << bm.fixed << " " << bm.ratio << " ";
                ResFile << bm.baseline_xyz[0] << " " << bm.baseline_xyz[1] << " " << bm.baseline_xyz[2] << " ";
                ResFile << bm.m_xyz[0] << " " << bm.m_xyz[1] << " " << bm.m_xyz[2] << " ";
                ResFile << bm.baseline_neu[0] << " " << bm.baseline_neu[1] << " " << bm.baseline_neu[2] << " ";
                ResFile << bm.m_neu[0] << " " << bm.m_neu[1] << " " << bm.m_neu[2] << " ";
                ResFile << bm.S_length << " " << bm.m_S << " ";
                ResFile << bm.sigma2 << " " << bm.RMS << "\n";
            }
        };
    }
    // 释放内存
    delete base_header;
    delete mobile_header;
    delete []base_gpseph;
    delete []base_bdseph;
    delete []mobile_gpseph;
    delete []mobile_bdseph;
    delete base_pos;
    delete mobile_pos;
    delete base_obs;
    delete mobile_obs;
    std::cout<<"固定率为:"<<fixed_res/(fixed_res+float_res)*100<<"%\n";
    fclose(BaseFile);
    fclose(MobileFile);
    ResFile.seekp(ResFile.beg); // 回到文件开头，输入固定率信息
    ResFile << fixed_res+float_res << " ";
    ResFile << fixed_res/(fixed_res+float_res)*100 << "%\n";
    ResFile.close();
}

int read_station(FILE *file, uint8_t *buff, OBS *obs, GPSEPH *gpseph, BDSEPH *bdseph, HeaderB *headerB) {
    if (feof(file)) return -1;
    if (readHeader(buff, file, headerB)) {
        switch (headerB->MsgId) {
            case ID_RANGE:
                obs->Formatting();
                readObs(buff, headerB, obs);
                return 1;
            case ID_GPSEPH:
                readGPSEPH(buff + headerB->len, headerB, gpseph);
                return 2;
            case ID_BDSEPH:
                readBDSEPH(buff + headerB->len, headerB, bdseph);
                return 3;
            default:
                return 4;
        }
    } else return 0;
}


void read_setting() {
    std::fstream infile("/Users/0-0mashuo/Desktop/Clion/RTK/Settings.txt", std::ios::in); // ..返回上级目录
    if (!infile.is_open()) {
        std::cerr << "can't reading file setting.txt\n";
    }

    std::string line;
    while (getline(infile, line)) {
        char ch[120];
        strcpy(ch, line.c_str());
        if (strlen(ch) < 80) continue;   // 无配置则略去
        if (strstr(ch, "END OF SETTING") != nullptr) break;  // 读取到配置结尾，则停止读取
        // 解算模式/数据源
        if (strstr(ch,"# Solution mode/Data source") != nullptr){
            std::string str = strtok(ch, " "); uint8_t mode= std::stoi(str);
            Solution_mode = mode&1;
            Data_source = (mode>>1)&1;
            continue;
        }
        // 高度角限制
        if (strstr(ch,"# Elevation limit") != nullptr) {
            std::string str = strtok(ch, " ");
            ELEVATION_LIMIT = std::stoi(str);
            continue;
        }
        // 信噪比限制
        if (strstr(ch,"# SNR limit") != nullptr) {
            std::string str = strtok(ch, " ");
            SNR_LIMIT = std::stoi(str);
            continue;
        }
        // 方差模型
        if (strstr(ch,"# Variance model") != nullptr) {
            std::string str = strtok(ch, " ");
            Variance_model = std::stoi(str);
            continue;
        }
        if (strstr(ch,"# Ratio limit") != nullptr){
            std::string str = strtok(ch, " ");
            RATIO_LIMIT = std::stoi(str);
            continue;
        }
        // 伪距方差
        if (strstr(ch,"# Variance Ratio") != nullptr) {
            std::string str = strtok(ch, " ");
            Variance_ratio = std::stoi(str);
            continue;
        }
        // 参与RTK解算的卫星和频率
        if (strstr(ch,"# Satellite system and frequency") != nullptr) {
            std::string str = strtok(ch, " ");
            strcpy(Satellite_system_frequency,ch);
            continue;
        }
        if (strstr(ch, "# Process file path") != nullptr){
            std::string str = strtok(ch, " ");
            strcpy(OUTPUT_FILE_DIRECTORY,ch);
            continue;
        }
        // 设定路径
        if (Data_source == 0 && strstr(ch,"# Mobile logFile directory(SPP file)") != nullptr){
            std::string str = strtok(ch, " ");
            strcpy(file_directory1,ch);
            continue;
        }
        if (Data_source == 0 && strstr(ch,"# Base logFile directory") != nullptr){
            std::string str = strtok(ch, " ");
            strcpy(file_directory2,ch);
            continue;
        }
        // 设定网络端口
        if (Data_source == 1 && strstr(ch,"# BASE IP(SPP IP)")){
            std::string str = strtok(ch, " ");
            strcpy(Setting_IP,ch);
            continue;
        }
        if (Data_source == 1 && strstr(ch,"# BASE PORT(SPP PORT)")){
            std::string str = strtok(ch, " ");
            strcpy(Setting_Port,ch);
            continue;
        }
        if (Data_source == 1 && strstr(ch,"# MOBILE IP")){
            std::string str = strtok(ch, " ");
            strcpy(Setting_IP2,ch);
            continue;
        }
        if (Data_source == 1 && strstr(ch,"# MOBILE PORT")){
            std::string str = strtok(ch, " ");
            strcpy(Setting_Port2,ch);
            continue;
        }
        // 数据处理文件路径配置
        if (strstr(ch,"# Process file path") != nullptr){
            std::string str = strtok(ch, " ");
            strcpy(PROCESS_FILE_PATH,ch);
            continue;
        }
    }
}

static bool port_readHeader(uint8_t *buff, HeaderB *headerB, int & len){
    // 文件头判断
    uint8_t type = *(buff + 6);
    if (!(buff[0] == OEM7SYNC1 && buff[1] == OEM7SYNC2 && buff[2] == OEM7SYNC3) ||
        (type >> 5 & 3) != 0){
        len = 7; // 本次消息长度
        return false;
    }
    uint8_t HeaderLen = *(buff + 3);
    headerB->len = HeaderLen;
    uint16_t MsgLen = U2(buff + 8);
    headerB->MsgLen = MsgLen;
    // crc检验
    if (crc32(buff, HeaderLen + MsgLen) != U4(buff + HeaderLen + MsgLen)){
        len = HeaderLen + MsgLen + 4; // 本次消息长度
        return false;
    }
    len = HeaderLen + MsgLen + 4; // 本次消息长度
    // GPS时
    headerB->t.weeks = U2(buff + 14);
    headerB->t.second = 1e-3 * U4(buff + 16);
    // ID
    uint16_t ID = U2(buff + 4);
    headerB->MsgId = ID;
    return true;
}

int port_read_station(uint8_t *buff, OBS *obs, GPSEPH *gpseph, BDSEPH *bdseph, HeaderB *headerB, int & len) {
    if (port_readHeader(buff,  headerB, len)) {
        switch (headerB->MsgId) {
            case ID_RANGE:
                obs->Formatting();
                readObs(buff, headerB, obs);
                return 1;
            case ID_GPSEPH:
                readGPSEPH(buff + headerB->len, headerB, gpseph);
                return 2;
            case ID_BDSEPH:
                readBDSEPH(buff + headerB->len, headerB, bdseph);
                return 3;
            default:
                return 4;
        }
    } else return 0;
}

bool is_header(uint8_t buff[],int &len){
    uint8_t type = *(buff + 6);
    if (!(buff[0] == OEM7SYNC1 && buff[1] == OEM7SYNC2 && buff[2] == OEM7SYNC3) ||
        (type >> 5 & 3) != 0) { return false; }
    uint8_t HeaderLen = *(buff + 3);
    uint16_t MsgLen = U2(buff + 8);
    len = HeaderLen + MsgLen + 4; // 完整消息包括头文件、消息体、和crc校验码
    return true;
}

