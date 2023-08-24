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
};