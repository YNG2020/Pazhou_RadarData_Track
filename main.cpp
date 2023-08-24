// #include<iostream>
// #include<string>
// #include<vector>
// #include<thread>
// #include "readData.h"
// std::vector<rtk> rtkline1;
// std::vector<rtk> rtkline2;
// std::vector<rtk> rtkline3;
// std::vector<Radar> RadarData;
// int main(){
//     readRtk("./data/Lane1_rtk.dat",rtkline1);
//     readRtk("./data/Lane2_rtk.dat",rtkline2);
//     readRtk("./data/Lane3_rtk.dat",rtkline3);
//     readRadarData("./data/RadarData.csv",RadarData);

//     return 0;
// }

#include <iostream>
#include <ctime>

// 定义GPS起始时间的Unix时间戳，即1980年1月6日午夜
const time_t GPS_START_TIMESTAMP = 315964800;
// 将GNSS周数和周内秒数转换为Unix时间戳
time_t convertGNSSWeekToTimestamp(int gpsWeek, double gpsWeekSeconds) {
    time_t timestamp = GPS_START_TIMESTAMP + (gpsWeek * 7 * 24 * 60 * 60) + static_cast<time_t>(gpsWeekSeconds);
    return timestamp;
}

int main() {
    int gnssWeek = 2269; // 假设的GNSS周数
    double gnssWeekSeconds = 271158.968000000; // 假设的周内秒数

    time_t timestamp = convertGNSSWeekToTimestamp(gnssWeek, gnssWeekSeconds);

    std::cout << "Unix时间戳：" << timestamp << std::endl;

    return 0;
}