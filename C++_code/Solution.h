#pragma once
#include <vector>
#include <string>
#include "struct.h"

struct rtk;
struct Radar;
struct targetInfo;
struct res;
using std::vector;


// 字符串转小数1
inline double str2double(std::string& str) {
    double num = 0;
    int index = 0;
    int flag = 1;
    if (str[index] == '-') {
        flag = -1;
        index++;
    }
    while (index < str.size() && str[index] != '.') {
        num = num * 10 + str[index] - '0';
        index++;
    }
    index++;
    double point = 0.1;
    while (index < str.size()) {
        num = num + point * (str[index] - '0');
        point = point * 0.1;
        index++;
    }
    return num * flag;
}

// 字符串转小数2
inline double str2double(std::string& str, int& index) {
    double num = 0;
    int flag = 1;
    if (str[index] == '-') {
        flag = -1;
        index++;
    }
    while (str[index] != ',' && str[index] != '.') {
        num = num * 10 + str[index] - '0';
        index++;
    }
    if (str[index] == ',') {
        return num * flag;
    }
    index++;
    double point = 0.1;
    while (str[index] != ',') {
        num = num + point * (str[index] - '0');
        point = point * 0.1;
        index++;
    }
    return num * flag;
}
// GPST时间转换为UNIX时间
extern double GPST2UNIX(int weeks, double second_in_weeks);
// 预测车辆在下一帧出现的位置，将历史速度与当前速度做一个权衡，用权衡后的速度乘以经过的时间，加上原本的位置，即得到预测位置
extern void predict_lot(int LastOKIDX, int tmpDotIdx, vector<Radar>::iterator radarBegin, const vector<vector<double>>& laneRadarTrackBegin, int lastLastOKIdx, double& predict_x, double& predict_y);
// 通过车辆当前的x，y，z坐标，以及径向速度，计算其实际速度（统一默认车辆沿着车道行驶，在误差可接受的范围内，若默认车道方向就是x轴正向方向，此时alpha=1）
extern double v_true_cal(double x, double y, double z, double v_r, double alpha);
// 该部分用于检测径向速度相近且位置接近的雷达数据
extern void find_relate_data(int i, int j, const vector<Radar>& RadarData, const vector<int>& radarFrameTimeIdx, const vector<int>& Lane2FrameIdx, int DotIdx, int n_Frame, const vector<int>& idx, double& radar_x, double& radar_y);
// 利用最小二乘法，拟合数据LaneRadarTrack_x, LaneRadarTrack1_y为y = kx + b的形式
extern void line_plofit(const vector<double>& LaneRadarTrack_x, const vector<double>& LaneRadarTrack_y, double& k, double& b);
// 通过标定数据，确定雷达坐标系的原点的经纬度
extern void cal_ori_lat_and_long(double& ori_longitude, double& ori_latitude, double theta0, double latitudeMean, const vector<vector<double>>& LaneRadarTrack1, const vector<vector<double>>& LaneRadarTrack2, const vector<vector<double>>& LaneRadarTrack3);
// 求出两条界限直线的截距
extern void get_intercept(double k, double b1, double b3, double& b_left, double& b_right);
// 设置同一辆车的在前后帧的在车道上的最大纵向距离偏差和最大横向距离偏差
extern void getMaxVarX_MaxVarY(double maxCarX, double maxCarY, double theta2, double& maxVarX, double& maxVarY);
// 检查指定点是否位于区域内
extern bool check_in_zone(double k, double b_left, double b_right, double x, double y);

// PI常数;
extern const double PI;

/**
* @class <Solution> [solution.h]
* @brief 用于全局统筹的一个类
*/
class Solution {
public:

	vector<rtk> rtkLine1;	/**< 标定数据1*/
	vector<rtk> rtkLine2;	/**< 标定数据2*/
	vector<rtk> rtkLine3;	/**< 标定数据3*/
	vector<Radar> RadarData;	/**< 全体雷达数据*/
	vector<res> Results;	/**< 存放全体结果*/

	int n_Gap;	/**< 全体雷达数据中的帧数*/
	double theta2;	/**< 车道与雷达坐标系之间的角度偏差*/
	double cosTheta2;	/**< theta2余弦值*/
	double sinTheta2;	/**< theta2正弦值*/
	double theta0;	/**< 经纬度与车道之间的角度偏差*/
	double latitudeMean;	/**< 平均纬度*/
	double ori_longitude;	/**< 雷达远点经度*/
	double ori_latitude;	/**< 雷达远点纬度*/
	double b_left;	/**< 左侧路基截距*/
	double b_right;	/**< 右侧路基截距*/
	double k;	/**< 车道斜率*/
	double kAti;	/**< 海拔斜率*/
	double bAti;	/**< 海拔截距*/

	void init();	/**< 初始化程序*/
	void run();		/**< 主算法*/
	void writeSingleResult(double nowT, int carUniqueId, double X_mean, double Y_mean, double carLat, double RadarHeight, double sp_mean, double RCS_mean, int RadarDataID, vector<res>& all_res, int data_idx, float maxCarLen);	/**< 写下单条结果*/
	vector<vector<double>> mapLane2Radar(vector<rtk>::iterator begin, vector<rtk>::iterator end);	/**< 创建标定数据与雷达数据之间的映射*/
	void getCoordinate(double distLong, double distLati, double& longitude, double& latitude);	/**< 根据横距纵向距离与横向距离，获得相应的经纬度坐标*/
};

extern Solution solution;
