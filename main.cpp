#include<iostream>
#include<string>
#include<vector>
#include<thread>
#include "readData.h"
std::vector<rtk> rtkline1;
std::vector<rtk> rtkline2;
std::vector<rtk> rtkline3;
std::vector<Radar> RadarData;
int main(){
    readRtk("./data/Lane1_rtk.dat",rtkline1);
    readRtk("./data/Lane2_rtk.dat",rtkline2);
    readRtk("./data/Lane3_rtk.dat",rtkline3);
    readRadarData("./data/RadarData.csv",RadarData);

    return 0;
}