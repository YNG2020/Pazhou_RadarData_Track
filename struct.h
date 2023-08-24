#include<iostream>
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