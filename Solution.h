#pragma once
#include <vector>
#include <string>
#include "struct.h"

struct rtk;
struct Radar;
struct targetInfo;
struct res;
using std::vector;


// �ַ���תС��1
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

// �ַ���תС��2
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
// GPSTʱ��ת��ΪUNIXʱ��
extern double GPST2UNIX(int weeks, double second_in_weeks);
// Ԥ�⳵������һ֡���ֵ�λ�ã�����ʷ�ٶ��뵱ǰ�ٶ���һ��Ȩ�⣬��Ȩ�����ٶȳ��Ծ�����ʱ�䣬����ԭ����λ�ã����õ�Ԥ��λ��
extern void predict_lot(int LastOKIDX, int tmpDotIdx, vector<Radar>::iterator radarBegin, const vector<vector<double>>& laneRadarTrackBegin, int lastLastOKIdx, double& predict_x, double& predict_y);
// ͨ��������ǰ��x��y��z���꣬�Լ������ٶȣ�������ʵ���ٶȣ�ͳһĬ�ϳ������ų�����ʻ�������ɽ��ܵķ�Χ�ڣ���Ĭ�ϳ����������x�������򣬴�ʱalpha=1��
extern double v_true_cal(double x, double y, double z, double v_r, double alpha);
// �ò������ڼ�⾶���ٶ������λ�ýӽ����״�����
extern void find_relate_data(int i, int j, const vector<Radar>& RadarData, const vector<int>& radarFrameTimeIdx, const vector<int>& Lane2FrameIdx, int DotIdx, int n_Frame, const vector<int>& idx, double& radar_x, double& radar_y);
// ������С���˷����������LaneRadarTrack_x, LaneRadarTrack1_yΪy = kx + b����ʽ
extern void line_plofit(const vector<double>& LaneRadarTrack_x, const vector<double>& LaneRadarTrack_y, double& k, double& b);
// ͨ���궨���ݣ�ȷ���״�����ϵ��ԭ��ľ�γ��
extern void cal_ori_lat_and_long(double& ori_longitude, double& ori_latitude, double theta0, double latitudeMean, const vector<vector<double>>& LaneRadarTrack1, const vector<vector<double>>& LaneRadarTrack2, const vector<vector<double>>& LaneRadarTrack3);
// �����������ֱ�ߵĽؾ�
extern void get_intercept(double k, double b1, double b3, double& b_left, double& b_right);
// ����ͬһ��������ǰ��֡���ڳ����ϵ�����������ƫ������������ƫ��
extern void getMaxVarX_MaxVarY(double maxCarX, double maxCarY, double theta2, double& maxVarX, double& maxVarY);
// ���ָ�����Ƿ�λ��������
extern bool check_in_zone(double k, double b_left, double b_right, double x, double y);

// PI����;
extern const double PI;

/**
* @class <Solution> [solution.h]
* @brief ����ȫ��ͳ���һ����
*/
class Solution {
public:

	vector<rtk> rtkLine1;	/**< �궨����1*/
	vector<rtk> rtkLine2;	/**< �궨����2*/
	vector<rtk> rtkLine3;	/**< �궨����3*/
	vector<Radar> RadarData;	/**< ȫ���״�����*/
	vector<res> Results;	/**< ���ȫ����*/

	int n_Gap;	/**< ȫ���״������е�֡��*/
	double theta2;	/**< �������״�����ϵ֮��ĽǶ�ƫ��*/
	double cosTheta2;	/**< theta2����ֵ*/
	double sinTheta2;	/**< theta2����ֵ*/
	double theta0;	/**< ��γ���복��֮��ĽǶ�ƫ��*/
	double latitudeMean;	/**< ƽ��γ��*/
	double ori_longitude;	/**< �״�Զ�㾭��*/
	double ori_latitude;	/**< �״�Զ��γ��*/
	double b_left;	/**< ���·���ؾ�*/
	double b_right;	/**< �Ҳ�·���ؾ�*/
	double k;	/**< ����б��*/
	double kAti;	/**< ����б��*/
	double bAti;	/**< ���νؾ�*/

	void init();	/**< ��ʼ������*/
	void run();		/**< ���㷨*/
	void writeSingleResult(double nowT, int carUniqueId, double X_mean, double Y_mean, double carLat, double RadarHeight, double sp_mean, double RCS_mean, int RadarDataID, vector<res>& all_res, int data_idx, float maxCarLen);	/**< д�µ������*/
	vector<vector<double>> mapLane2Radar(vector<rtk>::iterator begin, vector<rtk>::iterator end);	/**< �����궨�������״�����֮���ӳ��*/
	void getCoordinate(double distLong, double distLati, double& longitude, double& latitude);	/**< ���ݺ����������������룬�����Ӧ�ľ�γ������*/
};

extern Solution solution;
