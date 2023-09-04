#pragma once
#include<iostream>
#include<string>
#include<fstream>
#include<vector>
#include <iomanip>
#include "struct.h"

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
        data.Week=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Seconds=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Lat=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Lon=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Hgt=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.NorthVelocity=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.EastVelocity=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.UpVelocity=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Roll=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Pitch=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Azimuth=str2double(buff);
        buff.clear();

        rtkline.push_back(data);
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
        data.timestamp=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.ID=str2double(buff);
        buff.clear();

         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.DistLong=str2double(buff);
        buff.clear();


         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.DistLat=str2double(buff);
        buff.clear();


         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.VeloRadial=str2double(buff);
        buff.clear();

         while(index<line.size()&&(line[index]!=','&&line[index]!='\r'&&line[index]!='\n')){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.RCS=str2double(buff);
        buff.clear();
        RadarData.push_back(data);
        

    }
    inputFile.close();
}

void readLaneTarget(const std::string& targetInfoPath,std::vector<targetInfo>& targetInfos){
    std::ifstream inputFile(targetInfoPath); // 打开一个文件流并指定文件名

    if (!inputFile.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return ;
    }

    std::string line;
    std::getline(inputFile, line);
    while (std::getline(inputFile, line)) { // 逐行读取文件内容
        targetInfo data;
        int index=0;
        std::string buff="";
        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.ID=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_DistLong=str2double(buff);
        buff.clear();

         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_DistLat=str2double(buff);
        buff.clear();


         while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_Longitude=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_Latitude=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_VeloLong=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Object_VeloLat=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.VeloRadial=str2double(buff);
        buff.clear();


        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.timestamp=str2double(buff);
        buff.clear();

        while(line[index]!=','){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.RCS=str2double(buff);
        buff.clear();


        while(index<line.size()&&(line[index]!=','&&line[index]!='\r'&&line[index]!='\n')){
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.Velodis=str2double(buff);
        buff.clear();
        targetInfos.push_back(data);
    }
    inputFile.close();
}

void writeResult(const std::string& Path,std::vector<res>& Result){
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