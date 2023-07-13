#include <iostream>
#include "readB.h"
#include "setting_map.h"
#include "RelativePositioning.h"
#include "receive_port.h"


int main() {



    std::cout<<"正在读取配置......\n\n";
    read_setting();

    if (Solution_mode==0){
        if(Data_source==0){
            // 二进制文件单点定位
            std::cout<<"正在读取二进制文件:"<<file_directory1<<"并进行单点定位......\n\n";
            read_B_SPP(file_directory1);
            std::cout<<"\n文件读取结束!";
            return 0;
        }
        if(Data_source==1){
            // 网络流数据单点定位
            std::cout<<"正在读取网络数据流，进行单点定位......\n\n";
            spp_port();
            return 0;
        }
        std::cerr<<"Data Source chosen wrong!";
    }
    if (Solution_mode==1){
        std::cout<<"正在进行RTK解算配置......\n\n";
        dispose_RTK(); // RTK解算配置
        if(Data_source==0){
            // 二进制文件RTK路径输出
            std::cout<<"基准站数据:"<<file_directory2<<'\n'<<"流动站数据:"<<file_directory1<<'\n'<<"进行RTK中......\n\n";
            read_B_RTK(file_directory2,file_directory1);
            std::cout<<"\n文件读取结束!";
            return 0;
        }
        if(Data_source==1){
            // 网络流数据RTK
            std::cout<<"正在读取网络数据流，进行RTK......\n\n";
            rtk_port();
            return 0;
        }
        std::cerr<<"Data Source chosen wrong!";
    }
    // 解算模式配置错误
    std::cerr<<"Solution mode set wrong!";
}
