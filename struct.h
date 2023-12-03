#pragma once
#include<iostream>
#include<string>
struct rtk{        
    unsigned long Week;
    double Seconds;
    double timestamp;
    double Lat;
    double Lon;
    double Hgt;
    double NorthVelocity;
    double EastVelocity;
    double UpVelocity;
    double Roll;
    double Pitch;
    double Azimuth;
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
    double timestamp;
    unsigned long  ID;
    double DistLong;
    double DistLat;
    double VeloRadial;
    double RCS;
    void print(){
        printf("%.15f,%ld,%.15f,%.15f,%.15f,%.15f\n",
            timestamp,ID,DistLong,DistLat,VeloRadial,RCS
        );
    }
    static int readRadarData(const std::string& RadarDataPath, std::vector<Radar>& RadarData);
};

struct targetInfo //标定车辆在雷达区域的消息
{
    unsigned long ID;
    double Object_DistLong;
    double Object_DistLat;
    double Object_Longitude;
    double Object_Latitude;
    double Object_VeloLong;
    double Object_VeloLat;
    double VeloRadial;
    double timestamp;
    double RCS;
    double Velodis; //目标速度之差
    void print(){
        printf("%ld,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f,%.15f\n",
            ID,Object_DistLong,Object_DistLat,Object_Longitude,Object_Latitude,Object_VeloLat
            ,VeloRadial,timestamp,RCS,Velodis
        );
    }  
};

struct res{
    double Timestamp;
    unsigned long Object_ID;
    double Object_DistLong;
    double Object_DistLat;
    double Object_VeloLong;
    double Object_VeloLat;
    double Object_RCS;
    int Object_Class;
    double Object_Longitude;
    double Object_Latitude;
    double Object_Altitude;
    int Object_parking;
    int Object_retrograde;
    int Object_overspeed;

    static void writeResult(const std::string& Path, std::vector<res>& Result, const std::vector<int>& carID_buffer);
    res() { Timestamp = 0.0; };
};
