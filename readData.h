#pragma once
#include<iostream>
#include<string>
#include<fstream>
#include<vector>
#include <iomanip>
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
    std::getline(inputFile, line);
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
        RadarData.push_back(data);
        

    }
    inputFile.close();
}
void readLaneTarget(const std::string& targetinfoPath,std::vector<targetinfo>& targetinfos){
    std::ifstream inputFile(targetinfoPath); // 打开一个文件流并指定文件名

    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return ;
    }

    std::string line;
    std::getline(inputFile, line);
    while (std::getline(inputFile, line)) { // 逐行读取文件内容
        targetinfo data;
        int index=0;
        std::string buff="";
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
        data.Object_DistLong=strTodouble(buff);
        buff.clear();

         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_DistLat=strTodouble(buff);
        buff.clear();


         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_Longitude=strTodouble(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_Latitude=strTodouble(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_VeloLong=strTodouble(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_VeloLat=strTodouble(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.VeloRadial=strTodouble(buff);
        buff.clear();


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
        data.RCS=strTodouble(buff);
        buff.clear();


        while(index<line.size()&&(line[index]!=','&&line[index]!='\r'&&line[index]!='\n')){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Velodis=strTodouble(buff);
        buff.clear();
        // std::cout<<line<<std::endl;
        // data.print();
        targetinfos.push_back(data);
    }
    inputFile.close();
}

void wirteResult(const std::string& Path,std::vector<res>& Result){
     std::ofstream outFile; // 创建一个输出文件流对象

    // 打开文件，如果文件不存在则创建
    outFile.open(Path);

    if (!outFile) {
        std::cerr << "无法打开文件!" << std::endl;
        return;
    }
    for(auto& re:Result){
        outFile<<std::fixed << std::setprecision(3)<<re.Timestamp<<",";
        outFile<<re.Object_ID<<",";
        outFile<<re.Object_DistLong<<",";
        outFile<<re.Object_DistLat<<",";
        outFile<<re.Object_VeloLong<<",";
        outFile<<re.Object_VeloLat<<",";
        outFile<<re.Object_RCS<<",";
        outFile<<re.Object_Class<<",";
        outFile<<std::fixed << std::setprecision(11)<<re.Object_Longitude<<",";
        outFile<<re.Object_Latitude<<",";
        outFile<<re.Object_Altitude<<",";
        outFile<<re.Object_parking<<",";
        outFile<<re.Object_retrograde<<",";
        outFile<<re.Object_overspeed;
        outFile<<std::endl;
    }
    // 关闭文件流
    outFile.close();
}