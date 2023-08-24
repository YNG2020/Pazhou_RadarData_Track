#pragma once
#include<iostream>
#include<string>
#include<fstream>
#include<vector>
#include "struct.h"
double strTodouble(std::string& str){
    double num=0;
    int index=0;
    int flag=1;
    if(str[index]=='-'){
        flag=-1;
        index++;
    }
    while(index<str.size()&&str[index]!='.'){
        num=num*10+str[index]-'0';
        index++;
    }
    index++;
    double point=0.1;
    while(index<str.size()){
        num=num+point*(str[index]-'0');
        point=point*0.1;
        index++;
    }
    return num*flag;
}
const double GPS_START_TIMESTAMP = 315964800;

// 将GNSS周数和周内秒数转换为Unix时间戳
double convertGNSSWeekToTimestamp(int gpsWeek, double gpsWeekSeconds) {
    time_t timestamp = GPS_START_TIMESTAMP + (gpsWeek * 7 * 24 * 60 * 60) + gpsWeekSeconds;
    return timestamp;
}
void readRtk(const std::string& rtkPath,std::vector<rtk>& rtkline){
    std::ifstream inputFile(rtkPath); // 打开一个文件流并指定文件名

    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return ;
    }

    std::string line;
    while (std::getline(inputFile, line)) { // 逐行读取文件内容
        rtk data;
        int index=0;
        while(line[index++]!=';'){

        };
        std::string buff="";
        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Week=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Seconds=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Lat=strTodouble(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Lon=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Hgt=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.NorthVelocity=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.EastVelocity=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.UpVelocity=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Roll=strTodouble(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Pitch=strTodouble(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Azimuth=strTodouble(buff);
        buff.clear();

        rtkline.push_back(data);
        // std::cout<<line<<std::endl;
        // data.print();
    }
    inputFile.close(); // 关闭文件流
}   
void readRadarData(const std::string& RadarDataPath,std::vector<Radar>& RadarData){
    std::ifstream inputFile(RadarDataPath); // 打开一个文件流并指定文件名

    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return ;
    }

    std::string line;
    while (std::getline(inputFile, line)) { // 逐行读取文件内容
        Radar data;
        int index=0;
        std::string buff="";
        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.timestamp=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.ID=strTodouble(buff);
        buff.clear();

         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.DistLong=strTodouble(buff);
        buff.clear();


         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.DistLat=strTodouble(buff);
        buff.clear();


         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.VeloRadial=strTodouble(buff);
        buff.clear();

         while(index<line.size()&&(line[index]!=','&&line[index]!='\r'&&line[index]!='\n')){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.RCS=strTodouble(buff);
        buff.clear();
        // std::cout<<line<<std::endl;
        // data.print();

    }
    inputFile.close();
}