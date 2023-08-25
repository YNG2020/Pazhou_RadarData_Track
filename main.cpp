#include<iostream>
#include<string>
#include<vector>
#include<thread>
#include<cmath>
#include<unordered_map>
#include<algorithm>
#include "readData.h"
std::vector<rtk> rtkline1;
std::vector<rtk> rtkline2;
std::vector<rtk> rtkline3;
std::vector<targetinfo> targetinfos;
std::unordered_map<double,std::vector<double>> xToLongitude; //通过X得到经度
std::unordered_map<double,std::vector<double>> yToLatitude;  //通过Y得到纬度
std::vector<std::pair<double,double>> X; //保存所有的X
std::vector<std::pair<double,double>> Y; //保存所有的Y 
std::vector<Radar> RadarData;
std::vector<res> Results;
int main(){
    readRtk("./data/Lane1_rtk.dat",rtkline1);
    readRtk("./data/Lane2_rtk.dat",rtkline2);
    readRtk("./data/Lane3_rtk.dat",rtkline3);
    readLaneTarget("./data/lane1.csv",targetinfos);
    readLaneTarget("./data/lane2.csv",targetinfos);
    readLaneTarget("./data/lane3.csv",targetinfos);
    std::cout<<targetinfos.size()<<std::endl;
    readRadarData("./data/RadarData.csv",RadarData);
    for(auto& targetinfo:targetinfos){
        xToLongitude[targetinfo.Object_DistLong].push_back(targetinfo.Object_Longitude);
        yToLatitude[targetinfo.Object_DistLat].push_back(targetinfo.Object_Latitude);
    }
    for(auto& xT:xToLongitude){
        double sum=0.0;
        for(auto& t:xT.second){
            sum+=t;
        }
        X.push_back({xT.first,sum/xT.second.size()});
    }
    for(auto& yT:yToLatitude){
        double sum=0.0;
        for(auto& t:yT.second){
            sum+=t;
        }
        Y.push_back({yT.first,sum/yT.second.size()});
    }
    std::sort(X.begin(),X.end(),[](std::pair<double,double> a,std::pair<double,double> b){
        return a.first<b.first;
    });//排序进行二分
    std::sort(Y.begin(),Y.end(),[](std::pair<double,double> a,std::pair<double,double> b){
        return a.first<b.first;
    });//排序进行二分
    std::cout<<RadarData.size()<<std::endl;
    int cur_ID=0;
    double cur_timestamp=RadarData[0].timestamp;
    int index=0;
    for(auto& radar:RadarData){
        if(radar.VeloRadial==0){
            continue;
        }
        if (radar.DistLat > 5 || radar.DistLat < -9.5) {    // 加入点云范围限制，以剔除道路外的影响
            continue;
        }
        if(radar.timestamp!=cur_timestamp){
            cur_timestamp=radar.timestamp;
            cur_ID=0;
        }
        res data;
        data.Timestamp=radar.timestamp;
        data.Object_ID=cur_ID++;
        // data.Object_ID=radar.ID; //需要追踪
        if(abs(radar.RCS)<1){ //随机,无监督
            data.Object_Class=0;
        }else if(abs(radar.RCS)<3){
            data.Object_Class=0;
        }else{
            data.Object_Class=0;
        }
        data.Object_DistLong=radar.DistLong;
        data.Object_DistLat=radar.DistLat;
        //x,y对应的速度
        data.Object_VeloLong=radar.VeloRadial/sqrt(radar.DistLong*radar.DistLong+radar.DistLat*radar.DistLat)*radar.DistLong;
        data.Object_VeloLat=radar.VeloRadial/sqrt(radar.DistLong*radar.DistLong+radar.DistLat*radar.DistLat)*radar.DistLat;
        //RCS
        data.Object_RCS=radar.RCS;
        //经纬度坐标,需要进行经纬度变换
        auto itx=std::lower_bound(X.begin(),X.end(),radar.DistLong,[](const std::pair<double, double>& pair, double value){
            return pair.first < value;
        });
        if(itx!=X.end()){
            int index=itx-X.begin();
            if(X[index].first!=radar.DistLong){
                if(index>0){
                    data.Object_Longitude=(X[index].second+X[index-1].second)/2;
                }else{
                    data.Object_Longitude=X[index].second;
                }
            }else{
                data.Object_Longitude=X[index].second;
            }
            
        }else{
            data.Object_Longitude=X.back().second;
        }

        auto ity=std::lower_bound(Y.begin(),Y.end(),radar.DistLat,[](const std::pair<double, double>& pair, double value){
            return pair.first < value;
        });
        if(ity!=Y.end()){
            int index=ity-Y.begin();
            if(Y[index].first!=radar.DistLat){
                if(index>0){
                    data.Object_Latitude=(Y[index].second+Y[index-1].second)/2;
                }else{
                    data.Object_Latitude=Y[index].second;
                }
            }else{
                data.Object_Latitude=Y[index].second;
            }
            
        }else{
            data.Object_Latitude=Y.back().second;
        }

        data.Object_Altitude=3;
        //停车，超速，逆行
        if(radar.VeloRadial==0){
            data.Object_parking=1;
        }else{
            data.Object_parking=0;
        }
        if(radar.VeloRadial<0){
            data.Object_retrograde=1;
        }else{
            data.Object_retrograde=0;
        }
        if(abs(radar.VeloRadial)>16.667){
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