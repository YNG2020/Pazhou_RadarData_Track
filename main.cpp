#include<iostream>
#include<string>
#include<vector>
#include<thread>
#include<cmath>
#include "readData.h"
std::vector<rtk> rtkline1;
std::vector<rtk> rtkline2;
std::vector<rtk> rtkline3;
std::vector<Radar> RadarData;
std::vector<res> Results;
int main(){
    readRtk("./data/Lane1_rtk.dat",rtkline1);
    readRtk("./data/Lane2_rtk.dat",rtkline2);
    readRtk("./data/Lane3_rtk.dat",rtkline3);
    readRadarData("./data/RadarData.csv",RadarData);
    std::cout<<RadarData.size()<<std::endl;
    int index=0;
    for(auto& radar:RadarData){
        res data;
        data.Timestamp=radar.timestamp;
        data.Object_ID=radar.ID; //需要追踪
        if(abs(radar.RCS)<2){ //随机,无监督
            data.Object_Class=0;
        }else if(abs(radar.RCS)<5){
            data.Object_Class=1;
        }else{
            data.Object_Class=2;
        }
        data.Object_DistLong=radar.DistLong;
        data.Object_DistLat=radar.DistLat;
        //x,y对应的速度
        data.Object_VeloLong=radar.VeloRadial/sqrt(radar.DistLong*radar.DistLong+radar.DistLat*radar.DistLat)*radar.DistLong;
        data.Object_VeloLat=radar.VeloRadial/sqrt(radar.DistLong*radar.DistLong+radar.DistLat*radar.DistLat)*radar.DistLat;
        //RCS
        data.Object_RCS=radar.RCS;
        //经纬度坐标,需要进行经纬度变换
        data.Object_Longitude=23.26320186825;
        data.Object_Latitude=113.52464821735;
        data.Object_Altitude=3;
        //停车，超速，逆行
        if(radar.VeloRadial==0){
            data.Object_parking=1;
        }else{
            data.Object_parking=0;
        }
        if(radar.VeloRadial>0){
            data.Object_retrograde=1;
        }else{
            data.Object_retrograde=0;
        }
        if(radar.VeloRadial>16.667){
            data.Object_overspeed=1;
        }else{
            data.Object_overspeed=0;
        }
        Results.push_back(data);
        index++;
        // if(index==100){
        //     break;
        // }
    }
    wirteResult("./data/result.csv",Results);
    return 0;
}