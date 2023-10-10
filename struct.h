#pragma once
#include<iostream>
#include<string>
struct rtk{        
    unsigned long Week{0};
    double Seconds{0};
    double timestamp{0};
    double Lat{0};
    double Lon{0};
    double Hgt{0};
    double NorthVelocity{0};
    double EastVelocity{0};
    double UpVelocity{0};
    double Roll{0};
    double Pitch{0};
    double Azimuth{0};
    char Status[4];
    char xxx[4]; 
    void print(){
        printf("%ld,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f\n",
            Week,Seconds,timestamp,Lat,Lon,Hgt,NorthVelocity,EastVelocity,UpVelocity,Roll,Pitch,Azimuth
        );
    }
    static void readRtk(const char* rtkPath, std::vector<rtk>& rtkline);
};

struct Radar{        
    double timestamp{0};
    unsigned long  ID{0};
    double DistLong{0};
    double DistLat{0};
    double VeloRadial{0};
    double RCS{0};
    void print(){
        printf("%.15f,%ld,%.15f,%.15f,%.15f,%.15f\n",
            timestamp,ID,DistLong,DistLat,VeloRadial,RCS
        );
    }
    static int readRadarData(const std::string& RadarDataPath, std::vector<Radar>& RadarData);
};

struct targetInfo //标定车辆在雷达区域的消息
{
    unsigned long ID{0};
    double Object_DistLong{0};
    double Object_DistLat{0};
    double Object_Longitude{0};
    double Object_Latitude{0};
    double Object_VeloLong{0};
    double Object_VeloLat{0};
    double VeloRadial{0};
    double timestamp{0};
    double RCS{0};
    double Velodis{0}; //目标速度之差
    void print(){
        printf("%ld,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f\n",
            ID,Object_DistLong,Object_DistLat,Object_Longitude,Object_Latitude,Object_VeloLat
            ,VeloRadial,timestamp,RCS,Velodis
        );
    }  
};

struct res{
    double Timestamp{0};
    unsigned long Object_ID{0};
    double Object_DistLong{0};
    double Object_DistLat{0};
    double Object_VeloLong{0};
    double Object_VeloLat{0};
    double Object_RCS{0};
    int Object_Class{0};
    double Object_Longitude{0};
    double Object_Latitude{0};
    double Object_Altitude{0};
    int Object_parking{0};
    int Object_retrograde{0};
    int Object_overspeed{0};

    static void writeResult(const std::string& Path, std::vector<res>& Result, const std::vector<int>& carID_buffer);
    res() { Timestamp = 0.0; };
};
