#include <iomanip>
#include<iostream>
#include<string>
#include<fstream>
#include<vector>
#include "struct.h"
#include "Solution.h"

void rtk::readRtk(const char* rtkPath, std::vector<rtk>& rtkline) {
    rtkline.reserve(11000);
    std::ifstream inputFile(rtkPath); // ��һ���ļ�����ָ���ļ���
    std::string line;
    while (std::getline(inputFile, line)) { // ���ж�ȡ�ļ�����
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
    inputFile.close(); // �ر��ļ���
}

int Radar::readRadarData(const std::string& RadarDataPath, std::vector<Radar>& RadarData) {
    // RadarData.reserve(3000000);
    std::ifstream inputFile(RadarDataPath); // ��һ���ļ�����ָ���ļ���

    int frameCnt = 0;
    double lastTimeStamp = -1.0;

    std::string line;
    std::getline(inputFile, line);
    while (std::getline(inputFile, line)) { // ���ж�ȡ�ļ�����
        Radar data;
        int index = 0;
        data.timestamp = str2double(line, index);
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
        // std::cout<<line<<std::endl;
        // data.print();
    }
    inputFile.close();
    return frameCnt;
}

void res::writeResult(const std::string& Path, std::vector<res>& Result, const std::vector<int>& carID_buffer) {
    std::ofstream outFile; // ����һ������ļ�������
    int cnt = 0;
    // ���ļ�������ļ��������򴴽�
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
    // �ر��ļ���
    outFile.close();
}
