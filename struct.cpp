#include <iomanip>
#include<iostream>
#include<string>
#include<fstream>
#include<vector>
#include "struct.h"
#include "Solution.h"

void rtk::readRtk(const char* rtkPath, std::vector<rtk>& rtkline) {
    rtkline.reserve(11000);
    std::ifstream inputFile(rtkPath); // 打开一个文件流并指定文件名
    std::string line;
    while (std::getline(inputFile, line)) { // 逐行读取文件内容
        if (line[1] == 'B')
            continue;
        rtk data;
        int index = 0;
        while (line[index++] != ';') {

        };
        data.Week = str2double(line, index);
        index++;
        data.Seconds = str2double(line, index);
        index++;
        data.Lat = str2double(line, index);
        index++;
        data.Lon = str2double(line, index);
        index++;
        data.Hgt = str2double(line, index);
        index++;
        data.NorthVelocity = str2double(line, index);
        index++;
        data.EastVelocity = str2double(line, index);
        index++;
        data.UpVelocity = str2double(line, index);
        index++;
        data.Roll = str2double(line, index);
        index++;
        data.Pitch = str2double(line, index);
        index++;
        data.Azimuth = str2double(line, index);
        index++;
        rtkline.push_back(data);
    }
    inputFile.close(); // 关闭文件流
}

int Radar::readRadarData(const std::string& RadarDataPath, std::vector<Radar>& RadarData) {
    RadarData.reserve(3000000);
    std::ifstream inputFile(RadarDataPath); // 打开一个文件流并指定文件名

    int frameCnt = 0;
    double lastTimeStamp = -1.0;

    std::string line;
    std::getline(inputFile, line);
    while (std::getline(inputFile, line)) { // 逐行读取文件内容
        Radar data;
        int index = 0;
        data.timestamp = str2double(line, index);
        data.timestamp = round(data.timestamp * 1000.0) / 1000.0;
        if (lastTimeStamp != data.timestamp) {
            lastTimeStamp = data.timestamp;
            frameCnt = frameCnt + 1;
        }
        index++;
        data.ID = str2double(line, index);
        index++;
        data.DistLong = str2double(line, index);
        index++;
        data.DistLat = str2double(line, index);
        index++;
        data.VeloRadial = str2double(line, index);
        index++;
        std::string buff = "";
        while (index < line.size() && (line[index] != ',' && line[index] != '\r' && line[index] != '\n')) {
            buff.push_back(line[index]);
            index++;
        }
        index++;
        data.RCS = str2double(buff);
        RadarData.push_back(data);
    }
    inputFile.close();
    return frameCnt;
}

void res::writeResult(const std::string& Path, std::vector<res>& Result, const std::vector<int>& carID_buffer) {
    std::ofstream outFile; // 创建一个输出文件流对象
    int cnt = 0;
    // 打开文件，如果文件不存在则创建
    outFile.open(Path);
    for (auto& re : Result) {
        if (re.Timestamp == 0)
            break;
        if (carID_buffer[re.Object_ID] <= 1)
            continue;
        outFile << std::fixed << std::setprecision(3) << re.Timestamp << ",";
        outFile << re.Object_ID << ",";
        outFile << re.Object_DistLong << ",";
        outFile << re.Object_DistLat << ",";
        outFile << re.Object_VeloLong << ",";
        outFile << re.Object_VeloLat << ",";
        outFile << re.Object_RCS << ",";
        outFile << re.Object_Class << ",";
        outFile << std::fixed << std::setprecision(11) << re.Object_Longitude << ",";
        outFile << re.Object_Latitude << ",";
        outFile << re.Object_Altitude << ",";
        outFile << re.Object_parking << ",";
        outFile << re.Object_retrograde << ",";
        outFile << re.Object_overspeed;
        outFile << "\n";
    }
    // 关闭文件流
    outFile.close();
}
