#include "Solution.h"
#include "struct.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

Solution solution = Solution();
const double PI = 3.141592653589793;

// �����궨�������״�����֮���ӳ��
vector<vector<double>> Solution::mapLane2Radar(vector<rtk>::iterator begin, vector<rtk>::iterator end) {
    vector<double> radarFrameTime(n_Gap, 0);    // �״������в�ͬ֡��ʱ��
    vector<int> radarFrameTimeIdx(n_Gap, 0); // �״������в�ͬ֡��ʱ���Ӧ�ĵ�һ���±�
    vector<int> radarFrameCnt(n_Gap, 0);     // �״������в�ͬ֡�е��Ƶ�����
    radarFrameTime[0] = RadarData[0].timestamp;
    radarFrameTimeIdx[0] = 0;

    int n_RadarData = RadarData.size();  // �״���������
    for (int i = 1, cnt = 1; i < n_RadarData; ++i) {
        if (RadarData[i].timestamp == radarFrameTime[cnt-1])
            continue;
        radarFrameTime[cnt] = RadarData[i].timestamp;
        radarFrameTimeIdx[cnt] = i;
        radarFrameCnt[cnt-1] = i - radarFrameTimeIdx[cnt-1];
        ++cnt;
        if (cnt >= n_Gap)
            break;
    }
    radarFrameCnt[n_Gap - 1] = n_RadarData - radarFrameTimeIdx[n_Gap - 1];

    int n_Lane = end - begin;
    vector<int> Lane2RadarFrame(n_Lane, 0);     // �궨���ݵ�ʱ�̵��״����ݵ�֡�ĵ�һ���±��ӳ��
    vector<int> Lane2FrameIdx(n_Lane, 0);     // �궨���ݵ�ʱ�̵��״����ݵ�֡���±��ӳ��
    vector<double> Lane2FrameErr(n_Lane, 0);     // �궨���ݵ�ʱ�̵��״����ݵ�֡���±��ӳ������
    int weeks = (*begin).Week; // ������ͳһ��GPST����
    int startGapIdx = 0;
    int n_map = 1;  // ��������ӳ�������

    for (int i = 0; i < n_Lane; ++i) {
        double second_in_weeks = (*(begin + i)).Seconds;
        double UNIX_time = GPST2UNIX(weeks, second_in_weeks);
        while (startGapIdx < n_Gap - 1) {
            if (UNIX_time >= radarFrameTime[startGapIdx] && UNIX_time <= radarFrameTime[startGapIdx+1]) { // �����䣬�Ը�����������
                if (std::abs(radarFrameTime[startGapIdx] - UNIX_time) < std::abs(radarFrameTime[startGapIdx+1] - UNIX_time)) {
                    Lane2RadarFrame[i] = radarFrameTimeIdx[startGapIdx];
                    Lane2FrameIdx[i] = startGapIdx;
                    Lane2FrameErr[i] = std::abs(radarFrameTime[startGapIdx] - UNIX_time);
                }
                else {
                    Lane2RadarFrame[i] = radarFrameTimeIdx[startGapIdx+1];
                    Lane2FrameIdx[i] = startGapIdx + 1;
                    Lane2FrameErr[i] = std::abs(radarFrameTime[startGapIdx+1] - UNIX_time);
                }
                if (i > 0 && (Lane2FrameIdx[i] != Lane2FrameIdx[i-1]))
                    ++n_map;
                break;
            }
            // ��ǰ������û�ܶ�Ӧ�ϱ궨���ݵ�ʱ�������ת����һ������
            startGapIdx = startGapIdx + 1;
        }
    }
    vector<int> LaneIdx2Map(n_map, 0);  // �洢��Ҫ����ӳ���Lane���ݵ��±�
    for (int i = 0, cnt = 0; i < n_Lane;) { // ����ж���궨���ݵ�ʱ���Ӧ��ͬһ���״����ݵ�ʱ�������ôȡʱ�����С��
        int lastFrameIdx = Lane2FrameIdx[i];
        float min_error = 10000.0;
        int min_err_idx = i;
        while (i < n_Lane && Lane2FrameIdx[i] == lastFrameIdx) {
            if (Lane2FrameErr[i] < min_error) {
                min_error = Lane2FrameErr[i];
                min_err_idx = i;
            }
            ++i;
        }
        LaneIdx2Map[cnt] = min_err_idx;
        ++cnt;
    }

    vector<double> Lane_sp(n_Lane, 0);
    for (int i = 0; i < n_Lane; ++i)
        Lane_sp[i] = sqrt((*(begin + i)).EastVelocity * (*(begin + i)).EastVelocity + (*(begin + i)).NorthVelocity * (*(begin + i)).NorthVelocity);
    vector<vector<double>> LaneRadarTrack(n_map, vector<double>(9, 0));
    int LastOKIDX = 0, lastLastOKIdx = 0, cnt = 0;

    for (int mapIdx = 0, i = 0; mapIdx < n_map; ++mapIdx) {

        i = LaneIdx2Map[mapIdx];
        int n_Frame = radarFrameCnt[Lane2FrameIdx[i]], DotIdx = 0;
        vector<double> sp_gap(n_Frame, 0);
        for (int j = 0; j < n_Frame; ++j) {
            sp_gap[j] = std::abs(Lane_sp[i] - RadarData[radarFrameTimeIdx[Lane2FrameIdx[i]] + j].VeloRadial);
        }
        vector<int> idx(n_Frame);   std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(), [&sp_gap](int a, int b) {
            if (sp_gap[a] != sp_gap[b])
                return sp_gap[a] < sp_gap[b];
            else
                return a < b;
        });
        bool OKFLAG = false;
        double radar_x, radar_y, predict_x = 0.0, predict_y = 0.0;
        double sp_mean = 0.0;

        // ��һ����������������ҵ��ٶ�������Ҿ�����һ��OK�ĵ�ľ���С��5m�������㣬�������ɹ��ҵ���Ӧ��
        for (int j = 0; j < n_Frame; ++j) {
            if (sp_gap[idx[j]] > 1) // �ٶȲ�������ǰ������һ����
                break;
            if (j < n_Frame - 1) {
                if (std::abs(sp_gap[idx[j]] - sp_gap[idx[j + 1]]) < 0.01) {
                    int tmpDotIdx = radarFrameTimeIdx[Lane2FrameIdx[i]] + idx[j];

                    predict_lot(LastOKIDX, tmpDotIdx, RadarData.begin(), LaneRadarTrack, lastLastOKIdx, predict_x, predict_y);

                    if (cnt == 0 || (RadarData[tmpDotIdx].DistLong - predict_x) * (RadarData[tmpDotIdx].DistLong - predict_x) +
                        (RadarData[tmpDotIdx].DistLat - predict_y) * (RadarData[tmpDotIdx].DistLat - predict_y) < 25 &&
                        RadarData[tmpDotIdx].RCS > 0) {

                        OKFLAG = true;
                        DotIdx = tmpDotIdx;
                        radar_x = RadarData[DotIdx].DistLong;
                        radar_y = RadarData[DotIdx].DistLat;

                        // �ò������ڼ�⾶���ٶ������λ�ýӽ����״�����
                        find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
                        lastLastOKIdx = LastOKIDX;
                        LastOKIDX = cnt;
                        break;
                    }
                }
            }
        }

        // �ڶ�����������������Ҿ�����һ��OK�ĵ�ľ���С��5m��7m�ĵ㣬�������ɹ��ҵ���Ӧ��
        if (cnt == 0 && !OKFLAG) {
            DotIdx = radarFrameTimeIdx[Lane2FrameIdx[i]] + idx[0];
            predict_lot(LastOKIDX, DotIdx, RadarData.begin(), LaneRadarTrack, lastLastOKIdx, predict_x, predict_y);
            // �ò������ڼ�⾶���ٶ������λ�ýӽ����״�����
            find_relate_data(i, 0, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
            LastOKIDX = cnt;
            OKFLAG = 1;
        }
        else if (!OKFLAG) { // �����һ��������һ����ľ���С��5m������������
            for (int limit = 25; limit < 26; limit += 25) {
                if (OKFLAG)
                    break;
                else {
                    for (int j = 0; j < n_Frame; ++j) {
                        int tmpDotIdx = radarFrameTimeIdx[Lane2FrameIdx[i]] + idx[j];
                        predict_lot(LastOKIDX, tmpDotIdx, RadarData.begin(), LaneRadarTrack, lastLastOKIdx, predict_x, predict_y);
                        if ((RadarData[tmpDotIdx].DistLong - predict_x) * (RadarData[tmpDotIdx].DistLong - predict_x) +
                            (RadarData[tmpDotIdx].DistLat - predict_y) * (RadarData[tmpDotIdx].DistLat - predict_y) < limit &&
                            RadarData[tmpDotIdx].RCS > 0) {

                            OKFLAG = true;
                            DotIdx = tmpDotIdx;
                            radar_x = RadarData[DotIdx].DistLong;
                            radar_y = RadarData[DotIdx].DistLat;
                            // �ò������ڼ�⾶���ٶ������λ�ýӽ����״�����
                            find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
                            break;
                        }
                    }
                }
            }
        }

        // �����������������һֱ�Ҳ�����һ���㣬��ֱ��ȡ·��ǰ�������ϵģ�������һ��OK�ĵ������С�ĵ�
        if (!OKFLAG) {
            double min_dist = 1000000.0;
            int min_idx = 1;
            for (int j = 0; j < n_Frame; ++j) {
                int tmpDotIdx = radarFrameTimeIdx[Lane2FrameIdx[i]] + j;
                predict_lot(LastOKIDX, tmpDotIdx, RadarData.begin(), LaneRadarTrack, lastLastOKIdx, predict_x, predict_y);
                double dist = (RadarData[tmpDotIdx].DistLong - predict_x) * (RadarData[tmpDotIdx].DistLong - predict_x) +
                    (RadarData[tmpDotIdx].DistLat - predict_y) * (RadarData[tmpDotIdx].DistLat - predict_y);
                if (dist < min_dist) {
                    if (RadarData[tmpDotIdx].DistLong - LaneRadarTrack[LastOKIDX][0] > 0) {
                        min_dist = dist;
                        min_idx = j;
                    }
                }
            }
            DotIdx = radarFrameTimeIdx[Lane2FrameIdx[i]] + min_idx;
            radar_x = RadarData[DotIdx].DistLong;
            radar_y = RadarData[DotIdx].DistLat;
            // �ò������ڼ�⾶���ٶ������λ�ýӽ����״�����
            find_relate_data(i, min_idx, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
            OKFLAG = true;
        }
        if (!OKFLAG) {
            LaneRadarTrack.pop_back();
            --n_map; --mapIdx; continue;
        }

        LaneRadarTrack[cnt][0] = radar_x;   // �����״���������
        LaneRadarTrack[cnt][1] = radar_y;   // �����״�ĺ������
        LaneRadarTrack[cnt][2] = (*(begin + i)).Lat;    // γ��
        LaneRadarTrack[cnt][3] = (*(begin + i)).Lon;    // ����
        LaneRadarTrack[cnt][4] = (*(begin + i)).NorthVelocity;  // �����ٶ�
        LaneRadarTrack[cnt][5] = (*(begin + i)).EastVelocity;   // �����ٶ�
        LaneRadarTrack[cnt][6] = RadarData[radarFrameTimeIdx[Lane2FrameIdx[i]]].timestamp;  // ʱ��
        LaneRadarTrack[cnt][7] = (*(begin + i)).Hgt;    // ����
        LaneRadarTrack[cnt][8] = sp_mean;  // �����ٶ�
        ++cnt;
    }
    return LaneRadarTrack;
}

// ��ʼ������
void Solution::init() {
    // rtk::readRtk("Lane1_rtk.dat", rtkLine1);
    // rtk::readRtk("Lane2_rtk.dat", rtkLine2);
    // rtk::readRtk("Lane3_rtk.dat", rtkLine3);
    // n_Gap = Radar::readRadarData("RadarData.csv", RadarData);
    // vector<vector<double>> LaneRadarTrack1 = mapLane2Radar(rtkLine1.begin(), rtkLine1.end());
    // vector<vector<double>> LaneRadarTrack2 = mapLane2Radar(rtkLine2.begin(), rtkLine2.end());
    // vector<vector<double>> LaneRadarTrack3 = mapLane2Radar(rtkLine3.begin(), rtkLine3.end());
    // double k1 = 0.0, k2 = 0.0, k3 = 0.0, b1 = 0.0, b2 = 0.0, b3 = 0.0, arcTan = 0.0;
    // int nLane1 = LaneRadarTrack1.size(), nLane2 = LaneRadarTrack2.size(), nLane3 = LaneRadarTrack3.size();
    // vector<double> LaneRadarTrackX1(nLane1);
    // vector<double> LaneRadarTrackY1(nLane1);
    // vector<double> LaneRadarTrackAti1(nLane1);
    // vector<double> LaneRadarTrackX2(nLane2);
    // vector<double> LaneRadarTrackY2(nLane2);
    // vector<double> LaneRadarTrackAti2(nLane2);
    // vector<double> LaneRadarTrackX3(nLane3);
    // vector<double> LaneRadarTrackY3(nLane3);
    // vector<double> LaneRadarTrackAti3(nLane3);
    // for (int i = 0; i < nLane1; ++i) {
    //    LaneRadarTrackX1[i] = LaneRadarTrack1[i][0];
    //    LaneRadarTrackY1[i] = LaneRadarTrack1[i][1];
    //    LaneRadarTrackAti1[i] = LaneRadarTrack1[i][7];
    //    arcTan += atan(LaneRadarTrack1[i][4] / LaneRadarTrack1[i][5]);
    //    latitudeMean += LaneRadarTrack1[i][2];
    // }
    // for (int i = 0; i < nLane2; ++i) {
    //    LaneRadarTrackX2[i] = LaneRadarTrack2[i][0];
    //    LaneRadarTrackY2[i] = LaneRadarTrack2[i][1];
    //    LaneRadarTrackAti2[i] = LaneRadarTrack2[i][7];
    //    arcTan += atan(LaneRadarTrack2[i][4] / LaneRadarTrack2[i][5]);
    //    latitudeMean += LaneRadarTrack2[i][2];
    // }
    // for (int i = 0; i < nLane3; ++i) {
    //    LaneRadarTrackX3[i] = LaneRadarTrack3[i][0];
    //    LaneRadarTrackY3[i] = LaneRadarTrack3[i][1];
    //    LaneRadarTrackAti3[i] = LaneRadarTrack3[i][7];
    //    arcTan += atan(LaneRadarTrack3[i][4] / LaneRadarTrack3[i][5]);
    //    latitudeMean += LaneRadarTrack3[i][2];
    // }
    // line_plofit(LaneRadarTrackX1, LaneRadarTrackY1, k1, b1);
    // line_plofit(LaneRadarTrackX2, LaneRadarTrackY2, k2, b2);
    // line_plofit(LaneRadarTrackX3, LaneRadarTrackY3, k3, b3);
    // k = (k1 + k2 + k3) / 3;  // �������״�����ϵ�ϵ�б��
    // double theta1 = arcTan / (nLane1 + nLane2 + nLane3);    // ��γ�����״�����ϵ֮��ĽǶ�ƫ��
    // theta2 = atan(k);    // �������״�����ϵ֮��ĽǶ�ƫ��
    // theta0 = theta1 - theta2 + 0.003121;
    // latitudeMean = latitudeMean / (nLane1 + nLane2 + nLane3);   // ƽ��γ��
    // ori_longitude = 0.0, ori_latitude = 0.0;
    // cal_ori_lat_and_long(ori_longitude, ori_latitude, theta0, latitudeMean, LaneRadarTrack1, LaneRadarTrack2, LaneRadarTrack3);
    // b_left = 0.0, b_right = 0.0;
    // get_intercept(k, b1, b3, b_left, b_right);
    // cosTheta2 = cos(atan(k)), sinTheta2 = sin(atan(k));

    // double kAti1 = 0.0, kAti2 = 0.0, kAti3 = 0.0, bAti1 = 0.0, bAti2 = 0.0, bAti3 = 0.0;
    // line_plofit(LaneRadarTrackX1, LaneRadarTrackAti1, kAti1, bAti1);
    // line_plofit(LaneRadarTrackX2, LaneRadarTrackAti2, kAti2, bAti2);
    // line_plofit(LaneRadarTrackX3, LaneRadarTrackAti3, kAti3, bAti3);
    // kAti = (kAti1 + kAti2 + kAti3) / 3;
    // bAti = (bAti1 + bAti2 + bAti3) / 3;

    // �궨����Ԥ������
    n_Gap = Radar::readRadarData("RadarData.csv", RadarData);
    theta2 = 0;
    cosTheta2 = 1;
    sinTheta2 = 0;
    theta0 = 0.504648618725381;
    latitudeMean = 23.264113717225055;
    ori_longitude = 113.5252998913331;
    ori_latitude = 23.263563972364057;
    b_left = 7.589836551857512;
    b_right = -6.810346785925899;
    k = 0;
    kAti = 0.004225344416807;
    bAti = 26.899624552922948;
}

// ���㷨
void Solution::run() {

    // ***********************************�������˲�����ʼ��***********************************
    double deltat = 1;
    vector<vector<double>> A = { {1, deltat}, {0, 1} };   // ״̬ת�ƾ�����һʱ�̵�״̬ת�Ƶ���ǰʱ��
    vector<vector<double>> Q = { {0.5, 0}, {0, 0.01} };   // ��������Э�������Q��p(w)~N(0, Q)������������ʵ�����еĲ�ȷ����
    vector<vector<double>> R = { {4, 0}, {0, 0.04} };     // �۲�����Э�������R��p(v)~N(0, R)
    vector<vector<double>> H = { {1, 0}, {0, 1} };        // ״̬�۲����
    vector<vector<double>> P = { {4, 0}, {0, 0.04} };     // ״̬����Э�������P 
    vector<vector<double>> P_posterior = { {4, 0}, {0, 0.04} };   // ״̬�������Э�������
    vector<vector<double>> eyeMatrix = { {1.0, 0}, {0, 1.0} };  // ������λ����

    // ************************************��ʼ���Ͳ�������************************************
    int n_radar_data = RadarData.size();
    vector<int> frameGapIdx(n_Gap + 1, 0);      // Ԥ�����ÿһ֡�ĵ�һ���������״������г��ֵ�λ��
    frameGapIdx[n_Gap] = n_radar_data;

    double carA = 5.0;    // ����������һ������µ������ٶ�ΪcarAm / s ^ 2�����������븺���
    double maxCarX = 10.0;// Ԥ�賵������ΪmaxCarXm
    double maxCarY = 2.0; // Ԥ�賵������ΪmaxCarYm
    double maxVarX = 0.0, maxVarY = 0.0;
    getMaxVarX_MaxVarY(maxCarX, maxCarY, theta2, maxVarX, maxVarY);    // ����ͬһ��������ǰ��֡���ڳ����ϵ�����������ƫ������������ƫ��

    double maxVarRCS = 1500.0;    // ����ͬһ��������ǰ��֡��RCSƫ��
    double RadarHeight = 7.0;   // �״�߶�
    double RCSMin = 5.0;        // ��������СRCS
    double RCSMinZero = 10.0;   // ���״����ݵ�ľ����ٶ�Ϊ0ʱ����������СRCS
    double RCSMinSingle = 5.0; // ��ֻ��һ����Ч���״����ݵ㱻̽�⵽ʱ����������СRCS
    double carSpeedVar = 0.4;   // �������ͬһ�����ģ�ͬһ֡�ڵģ��״�ľ����ٶȵ����ƫ��
    int interpolationLimCnt = 1;// ��֡���ƣ��˴�����ʾ������֡����interpolationLimCnt�󣬲��ٲ�֡
    int interpolationLimM = 400;// ��֡���ƣ��ף���ʾ����interpolationLimM�󣬲��ٲ�֡
    int maxFailTime = 20;        // ����׷��ʧ�ܵ�������
    // ****************************************************************************************

    double lastTime = 0;
    for (int i = 0, cnt = 0; i < n_radar_data; ++i) {
        if (lastTime == RadarData[i].timestamp)
            continue;
        frameGapIdx[cnt] = i;
        lastTime = RadarData[i].timestamp;
        ++cnt;
    }

    vector<int> carID_buffer(10000, 1); // ��¼��carID���ֵĴ���
    vector<vector<double>> tracer_Pbuffer(500, vector<double>(2, 0));
    vector<vector<int>> tracer_buffer(500, vector<int>(4, 0)); // ��1�м�¼��ǰһ֡׷�ٵĴ����all_res�еı�ţ���2�м�¼��Ӧ����RadarData�еı�ţ���3�м�¼����׷��ʧ�ܵĴ�������4�м�¼��ǰ����׷�ٵ���
    int tracer_pointer = -1;     // tracer_pointer��Զָ��buffer�е����һ����ЧԪ�أ�����ǰ���Ϊ��ЧԪ��
    int data_idx = -1;
    vector<res> all_res(n_radar_data / 10);         // ��¼ȫ����
    vector<float> maxCarsLen(n_radar_data / 10);    // ��¼��󳵳���Ϣ
    int carUniqueId = -1;
    for (int cnt = 0; cnt < n_Gap; ++cnt) {
        int frameStart = frameGapIdx[cnt];  // ��ǰ֡�����״����ݵ���ʼλ
        int nFrame = frameGapIdx[cnt + 1] - frameGapIdx[cnt]; // ��֡��֡��
        vector<int> OKIndex;
        for (int i = 0; i < nFrame; ++i) {
            double y = RadarData[frameStart + i].DistLat, x = RadarData[frameStart + i].DistLong;
            double rcs = RadarData[frameStart + i].RCS;
            if (check_in_zone(k, b_left, b_right, x, y) && rcs > RCSMin) {
                if (RadarData[frameStart + i].VeloRadial == 0 && RadarData[frameStart + i].RCS < RCSMinZero) // ����õ��ٶ�Ϊ0����RCS < RCSMinZero��ȥ��
                    continue;
                OKIndex.push_back(i);
            }
        }
        int OKIndexPointer_len = OKIndex.size();
        vector<bool> BlockIndex(OKIndexPointer_len, false);
        vector<int> sortedIdx(OKIndexPointer_len);
        vector<Radar> curFrameData(OKIndexPointer_len);
        for (int i = 0; i < OKIndexPointer_len; ++i)
            curFrameData[i] = RadarData[frameStart + OKIndex[i]];

        std::iota(sortedIdx.begin(), sortedIdx.end(), 0);
        std::sort(sortedIdx.begin(), sortedIdx.end(), [&curFrameData](int i, int j) {
            if (curFrameData[i].VeloRadial != curFrameData[j].VeloRadial) {
                return curFrameData[i].VeloRadial < curFrameData[j].VeloRadial;
            }
            if (curFrameData[i].DistLong != curFrameData[j].DistLong) {
                return curFrameData[i].DistLong < curFrameData[j].DistLong;
            }
            if (curFrameData[i].DistLat != curFrameData[j].DistLat) {
                return curFrameData[i].DistLat < curFrameData[j].DistLat;
            }
            if (curFrameData[i].RCS != curFrameData[j].RCS) {
                return curFrameData[i].RCS < curFrameData[j].RCS;
            }
            return i < j;
            });
        if (OKIndexPointer_len == 0)
            continue;
        double nowT = curFrameData[0].timestamp;    // ��¼���ڵ�ʱ��

        // �����㷨
        int i = 0;
        while (i <= tracer_pointer && tracer_pointer > -1) {  // �ڸ��ٶ����һ�����ȶԵ�ǰ֡�����ݣ�ƥ��ɹ������ݵ㣬��������
            int dataID = tracer_buffer[i][0];
            int radarDataID = tracer_buffer[i][1];
            int carID = all_res[dataID].Object_ID;
            double carDisLog = all_res[dataID].Object_DistLong;
            double carDisLat = all_res[dataID].Object_DistLat;
            double carSpeed = RadarData[radarDataID].VeloRadial;
            double carRCS = all_res[dataID].Object_RCS;
            int carClass = all_res[dataID].Object_Class;
            double deltaT = nowT - all_res[dataID].Timestamp;
            float maxCarLen = maxCarsLen[dataID];
            P_posterior[0] = tracer_Pbuffer[i * 2];
            P_posterior[1] = tracer_Pbuffer[i * 2 + 1];
            bool coupleFlag = false;
            int j = 0;  // jָ��curFrameData�����е����ݵ�

            while (j < OKIndexPointer_len) {    // ��OKIndex��Ѱ����������׷�ٵĳ���ƥ��ĵ㣬�ҵ�֮�󣬰�����OKIndex��ɾ��
                if (BlockIndex[j]) {
                    ++j; continue;
                }
                if (std::abs(carSpeed - curFrameData[sortedIdx[j]].VeloRadial) / cosTheta2 > deltaT * carA) { // (deltaT��0.025~0.2֮��)
                    ++j; continue;
                }
                if (std::abs(carDisLat - curFrameData[sortedIdx[j]].DistLat) > maxVarY) {
                    ++j; continue;
                }
                if (std::abs(carRCS - curFrameData[sortedIdx[j]].RCS) > maxVarRCS) {
                    ++j; continue;
                }
                double v_true = v_true_cal(carDisLog, carDisLat, RadarHeight, carSpeed, cosTheta2); // ���㳵����ʵ���ٶȣ�Ĭ�ϳ������ų���������ʻ
                double X_predict = carDisLog + deltaT * v_true * cosTheta2;    // ������Ԥ������λ��
                if (std::abs(curFrameData[sortedIdx[j]].DistLong - X_predict) > maxVarX) {
                    ++j; continue;
                }

                // ƥ��ɹ�
                coupleFlag = true;
                double X_mean = curFrameData[sortedIdx[j]].DistLong;  double X_sum = X_mean;
                double Y_mean = curFrameData[sortedIdx[j]].DistLat;  double Y_sum = Y_mean;
                double sp_mean = curFrameData[sortedIdx[j]].VeloRadial; double sp_sum = sp_mean;
                double RCS_mean = curFrameData[sortedIdx[j]].RCS; double RCS_sum = RCS_mean;
                double Xmin = X_mean, Xmax = X_mean;
                BlockIndex[j] = true;   // ƥ��ɹ������ݵ㣬��������
                int jStart = j, tmpCnt = 1;
                ++j;
                while (j < OKIndexPointer_len) {
                    if (BlockIndex[j]) {
                        ++j; continue;
                    }
                    if (std::abs(X_mean - curFrameData[sortedIdx[j]].DistLong) > maxVarX) {
                        ++j; continue;
                    }
                    if (std::abs(Y_mean - curFrameData[sortedIdx[j]].DistLat) > maxVarY) {
                        ++j; continue;
                    }
                    if (std::abs(sp_mean - curFrameData[sortedIdx[j]].VeloRadial) > carSpeedVar) {
                        ++j; continue;
                    }
                    if (std::abs(RCS_mean - curFrameData[sortedIdx[j]].RCS) > maxVarRCS) {
                        ++j; continue;
                    }
                    if (curFrameData[sortedIdx[j]].DistLong < Xmin)
                        Xmin = curFrameData[sortedIdx[j]].DistLong;
                    if (curFrameData[sortedIdx[j]].DistLong > Xmax)
                        Xmax = curFrameData[sortedIdx[j]].DistLong;
                    float carLen = (Xmax - Xmin) / cosTheta2;
                    if (maxCarLen < carLen)
                        maxCarLen = carLen;
                    ++tmpCnt;
                    X_sum = X_sum + curFrameData[sortedIdx[j]].DistLong;   X_mean = X_sum / tmpCnt;
                    Y_sum = Y_sum + curFrameData[sortedIdx[j]].DistLat;   Y_mean = Y_sum / tmpCnt;
                    sp_sum = sp_sum + curFrameData[sortedIdx[j]].VeloRadial;  sp_mean = sp_sum / tmpCnt;
                    RCS_sum = RCS_sum + curFrameData[sortedIdx[j]].RCS; RCS_mean = RCS_sum / tmpCnt;
                    BlockIndex[j] = true;  // ƥ��ɹ������ݵ㣬��������
                    ++j;
                }
                ++data_idx;
                int RadarDataID = frameStart + OKIndex[sortedIdx[jStart]];
                if (sp_mean == 0) {
                    X_mean = carDisLog;
                    Y_mean = carDisLat;
                }

                //// ----------------------�����������---------------------
                //A = { {1, deltaT}, {0, 1} };
                //vector<vector<double>>X_last = { { carDisLog }, { carSpeed } };
                //vector<vector<double>>X_prior = matrixMultiply(A, X_last);
                //// -----------------����״̬����Э�������P----------------
                //vector<vector<double>> A_transpose = transposeMatrix(A);
                //vector<vector<double>> P_prior_tmp1 = matrixMultiply(A, P_posterior);
                //vector<vector<double>> P_prior_tmp2 = matrixMultiply(P_prior_tmp1, A_transpose);
                //vector<vector<double>> P_prior = matrixAddition(P_prior_tmp2, Q);
                //// ----------------------���㿨��������-------------------
                //R = { {std::max(maxCarLen * maxCarLen / 4.0, 9.0), 0}, {0.0, 0.0} };    // �۲�����Э�������R��p(v)~N(0,R)
                //vector<vector<double>> H_transpose = transposeMatrix(H);
                //vector<vector<double>> K_tmp1 = matrixMultiply(P_prior, H_transpose);
                //vector<vector<double>> K_tmp2 = matrixInverse(matrixAddition(matrixMultiply(matrixMultiply(H, P_prior), H_transpose), R));
                //vector<vector<double>> K = matrixMultiply(K_tmp1, K_tmp2);
                //// ------------------------�������-----------------------
                //vector<vector<double>> Z_measure = { {X_mean}, {sp_mean} };
                //vector<vector<double>> X_posterior = matrixAddition(X_prior, matrixMultiply(K, matrixSubtraction(Z_measure, matrixMultiply(H, X_prior))));

                //if (X_posterior[0][0] < 400) {
                //    X_mean = X_posterior[0][0];
                //    sp_mean = X_posterior[1][0];
                //}
                //else {      // ����˲�ֵ>400���򲻲����˲�ֵ����ǿ�н���׷��
                //    coupleFlag = 0;
                //    tracer_buffer[i][2] = maxFailTime + 1;
                //}

                //// --------------- ����״̬����Э�������P-----------------
                //P_posterior = matrixMultiply(matrixSubtraction(eyeMatrix, matrixMultiply(K, H)), P_prior);

                maxCarsLen[data_idx] = maxCarLen;
                ++carID_buffer[carID];
                writeSingleResult(nowT, carID, X_mean, Y_mean, carDisLat, RadarHeight, sp_mean, RCS_mean, RadarDataID, all_res, data_idx, maxCarLen);
                // �����ڻ�����������
                tracer_buffer[i][0] = data_idx;
                tracer_buffer[i][1] = RadarDataID;
                tracer_buffer[i][2] = 0;    // ����׷��ʧ�ܴ�������
                ++tracer_buffer[i][3];
                setRows(tracer_Pbuffer, P_posterior, i);
                break;
            }
            if (!coupleFlag) {  // ƥ��ʧ�ܣ����������ڸ��ٶ�����������ʧ�ܣ��ø������ݴӻ������б��Ƴ�
                if (tracer_buffer[i][2] > maxFailTime) {
                    tracer_buffer[i][0] = tracer_buffer[tracer_pointer][0];
                    tracer_buffer[i][1] = tracer_buffer[tracer_pointer][1];
                    tracer_buffer[i][2] = tracer_buffer[tracer_pointer][2];
                    tracer_buffer[i][3] = tracer_buffer[tracer_pointer][3];
                    vector<vector<double>> tmpMat(2, vector<double>(2, 0));
                    tmpMat[0] = tracer_Pbuffer[tracer_pointer * 2];
                    tmpMat[1] = tracer_Pbuffer[tracer_pointer * 2 + 1];
                    setRows(tracer_Pbuffer, tmpMat, i);
                    --tracer_pointer;
                }
                else {
                    ++tracer_buffer[i][2];
                    ++i;
                }
            }
            else
                ++i;
        }

        // ʶ���㷨
        int j = 0;  // jָ��curFrameData�����е����ݵ�
        while (j < OKIndexPointer_len) {
            if (BlockIndex[j]) {
                ++j; continue;
            }
            if (curFrameData[sortedIdx[j]].VeloRadial == 0) {
                BlockIndex[j] = 1; ++j;  continue;
            }
            if (curFrameData[sortedIdx[j]].RCS < RCSMinSingle) {
                BlockIndex[j] = 1; ++j;  continue;
            }

            // ����������Ϊ�п��ܷ��ֳ���
            double X_mean = curFrameData[sortedIdx[j]].DistLong;  double X_sum = X_mean;
            double Y_mean = curFrameData[sortedIdx[j]].DistLat;  double Y_sum = Y_mean;
            double sp_mean = curFrameData[sortedIdx[j]].VeloRadial; double sp_sum = sp_mean;
            double RCS_mean = curFrameData[sortedIdx[j]].RCS; double RCS_sum = RCS_mean;
            bool coupleFlag = false;
            int jStart = j;
            double Xmin = X_mean, Xmax = X_mean;
            ++j;
            int tmpCnt = 1;
            while (j < OKIndexPointer_len) {
                if (BlockIndex[j]) {
                    ++j; continue;
                }
                if (std::abs(X_mean - curFrameData[sortedIdx[j]].DistLong) > maxVarX) {
                    ++j; continue;
                }
                if (std::abs(Y_mean - curFrameData[sortedIdx[j]].DistLat) > maxVarY) {
                    ++j; continue;
                }
                if (std::abs(sp_mean - curFrameData[sortedIdx[j]].VeloRadial) > carSpeedVar) {
                    ++j; continue;
                }
                if (curFrameData[sortedIdx[j]].DistLong < Xmin) {
                    Xmin = curFrameData[sortedIdx[j]].DistLong;
                }
                if (curFrameData[sortedIdx[j]].DistLong > Xmax) {
                    Xmax = curFrameData[sortedIdx[j]].DistLong;
                }
                coupleFlag = true; ++tmpCnt;
                X_sum = X_sum + curFrameData[sortedIdx[j]].DistLong;   X_mean = X_sum / tmpCnt;
                Y_sum = Y_sum + curFrameData[sortedIdx[j]].DistLat;   Y_mean = Y_sum / tmpCnt;
                sp_sum = sp_sum + curFrameData[sortedIdx[j]].VeloRadial;  sp_mean = sp_sum / tmpCnt;
                RCS_sum = RCS_sum + curFrameData[sortedIdx[j]].RCS; RCS_mean = RCS_sum / tmpCnt;
                BlockIndex[j] = true; // ƥ��ɹ������ݵ㣬��������
                ++j;
            }
            if (coupleFlag) {
                BlockIndex[jStart] = true;
                ++data_idx;
                ++carUniqueId;
                float maxCarLen = (Xmax - Xmin) / cosTheta2;
                maxCarsLen[data_idx] = maxCarLen;
                int RadarDataID = frameStart + OKIndex[sortedIdx[jStart]];
                carID_buffer[carUniqueId] = 1;
                writeSingleResult(nowT, carUniqueId, X_mean, Y_mean, Y_mean, RadarHeight, sp_mean, RCS_mean, RadarDataID, all_res, data_idx, maxCarLen);
                ++tracer_pointer;
                tracer_buffer[tracer_pointer][0] = data_idx;
                tracer_buffer[tracer_pointer][1] = RadarDataID;
                tracer_buffer[tracer_pointer][2] = 0;
                tracer_buffer[tracer_pointer][3] = 1;
                setRows(tracer_Pbuffer, P_posterior, tracer_pointer);
            }
            j = jStart + 1;
        }
    }
     res::writeResult("result.csv", all_res, carID_buffer);
}

// д�µ������
void Solution::writeSingleResult(double nowT, int carUniqueId, double X_mean, double Y_mean, double carLat, double RadarHeight, double sp_mean, double RCS_mean, int RadarDataID, vector<res>& all_res, int data_idx, float maxCarLen) {
    all_res[data_idx].Timestamp = nowT;
    all_res[data_idx].Object_ID = carUniqueId;
    all_res[data_idx].Object_DistLong = X_mean;
    all_res[data_idx].Object_DistLat = Y_mean;
    double sp_true = v_true_cal(X_mean, Y_mean, RadarHeight, sp_mean, cosTheta2);
    all_res[data_idx].Object_VeloLong = sp_true * cosTheta2;
    if (std::abs(Y_mean - carLat) < 0.0001)
        all_res[data_idx].Object_VeloLat = 0;
    else if (Y_mean > carLat)
        all_res[data_idx].Object_VeloLat = std::abs(sp_true * sinTheta2);
    else
        all_res[data_idx].Object_VeloLat = -std::abs(sp_true * sinTheta2);

    all_res[data_idx].Object_RCS = RCS_mean;
    if (maxCarLen < 7.5)
        all_res[data_idx].Object_Class = 0;
    else
        all_res[data_idx].Object_Class = 1;
    double longitude = 0.0, latitude = 0.0;
    getCoordinate(X_mean, Y_mean, longitude, latitude);
    all_res[data_idx].Object_Latitude = latitude;
    all_res[data_idx].Object_Longitude = longitude;
    all_res[data_idx].Object_Altitude = kAti * X_mean + bAti;
    if (std::abs(sp_true) == 0)
        all_res[data_idx].Object_parking = 1;
    else
        all_res[data_idx].Object_parking = 0;
    if (sp_true > 0)
        all_res[data_idx].Object_retrograde = 1;
    else
        all_res[data_idx].Object_retrograde = 0;
    if (std::abs(sp_true) > 16.6667)
        all_res[data_idx].Object_overspeed = 1;
    else
        all_res[data_idx].Object_overspeed = 0;

}

// ���ݺ����������������룬�����Ӧ�ľ�γ������
void Solution::getCoordinate(double distLong, double distLati, double& longitude, double& latitude) {
    double R = 6371393.0;   // ����ƽ���뾶
    double longitude_gap_per_meter = 360 / (2 * PI * R * cos(latitudeMean / 180 * PI)); // ���������һ���ھ����Ͽ�Խ�Ķ���
    double latitude_gap_per_meter = 360 / (2 * PI * R); // �ϱ������һ����γ���Ͽ���Ķ���
    double westDeg = longitude_gap_per_meter * (distLong * cos(theta0) - distLati * sin(theta0));
    double southDeg = latitude_gap_per_meter * (distLong * sin(theta0) + distLati * cos(theta0));
    longitude = ori_longitude + westDeg;
    latitude = ori_latitude + southDeg;
}

// GPSTʱ��ת��ΪUNIXʱ��
double GPST2UNIX(int weeks, double second_in_weeks) {
    int UNIX_TO_GPST = 315964800;
    return UNIX_TO_GPST + 24 * 60 * 60 * 7 * weeks + second_in_weeks - 18;
}

// Ԥ�⳵������һ֡���ֵ�λ�ã�����ʷ�ٶ��뵱ǰ�ٶ���һ��Ȩ�⣬��Ȩ�����ٶȳ��Ծ�����ʱ�䣬����ԭ����λ�ã����õ�Ԥ��λ��
void predict_lot(int LastOKIDX, int tmpDotIdx, vector<Radar>::iterator radarBegin, const vector<vector<double>>& LaneRadarTrack, int lastLastOKIdx, double& predict_x, double& predict_y) {
    if (lastLastOKIdx == LastOKIDX) {
        predict_x = (*(radarBegin + tmpDotIdx)).DistLong;
        predict_y = (*(radarBegin + tmpDotIdx)).DistLat;
    }
    else {
        double delta_t1 = (*(radarBegin + tmpDotIdx)).timestamp - LaneRadarTrack[LastOKIDX][6]; // ��ǰʱ������һ�ɿ��켣���ʱ��֮��
        double delta_t2 = LaneRadarTrack[LastOKIDX][6] - LaneRadarTrack[0][6];  // ��һ�ɿ��켣�����һ���ɿ��켣��ʱ��֮��
        double delta_t3 = LaneRadarTrack[LastOKIDX][6] - LaneRadarTrack[lastLastOKIdx][6];  // ��һ�ɿ��켣����ǰǰ�ɿ��켣��ʱ��֮��

        double delta_x_his = (LaneRadarTrack[LastOKIDX][0] - LaneRadarTrack[0][0]) / delta_t2 * delta_t1;
        double delta_y_his = (LaneRadarTrack[LastOKIDX][1] - LaneRadarTrack[0][1]) / delta_t2 * delta_t1;
        double delta_x_cur = (LaneRadarTrack[LastOKIDX][0] - LaneRadarTrack[lastLastOKIdx][0]) / delta_t3 * delta_t1;
        double delta_y_cur = (LaneRadarTrack[LastOKIDX][1] - LaneRadarTrack[lastLastOKIdx][1]) / delta_t2 * delta_t1;

        float p = 1.0;
        double delta_x = delta_x_his * p + delta_x_cur * (1 - p);
        double delta_y = delta_y_his * p + delta_y_cur * (1 - p);
        double v = v_true_cal(LaneRadarTrack[LastOKIDX][0], LaneRadarTrack[LastOKIDX][1], 7, LaneRadarTrack[LastOKIDX][8], 1);
        delta_x = delta_t1 * v;

        predict_x = delta_x + LaneRadarTrack[LastOKIDX][0];
        predict_y = delta_y + LaneRadarTrack[LastOKIDX][1];
    }
}

// ͨ��������ǰ��x��y��z���꣬�Լ������ٶȣ�������ʵ���ٶȣ�ͳһĬ�ϳ������ų�����ʻ�������ɽ��ܵķ�Χ�ڣ���Ĭ�ϳ����������x�������򣬴�ʱalpha=1��
double v_true_cal(double x, double y, double z, double v_r, double alpha) {
    double cosTheta;
    if(x<0){
        cosTheta = x*-1.0 / sqrt(x*x + y*y + z*z);
    }else{
        cosTheta = x / sqrt(x*x + y*y + z*z); // ���״������x��Ȼ����0
    }
    
    // double temp=sqrt(x*x + y*y + z*z);
    double curx=std::abs(x);
    // size_t len=sizeof(long int);
    double v = (v_r / cosTheta) / alpha;
    return v;
}

// �ò������ڼ�⾶���ٶ������λ�ýӽ����״�����
void find_relate_data(int i, int j, const vector<Radar>& RadarData, const vector<int>& radarFrameTimeIdx, const vector<int>& Lane2FrameIdx, int DotIdx, int n_Frame, const vector<int>& idx, double& radar_x, double& radar_y, const vector<vector<double>>& LaneRadarTrack, int LastOKIDX, double& sp_mean) {
    
    // ***********************************�������˲�����ʼ��***********************************
    double deltat = 1;
    static vector<vector<double>> A = { {1, deltat}, {0, 1} };   // ״̬ת�ƾ�����һʱ�̵�״̬ת�Ƶ���ǰʱ��
    static vector<vector<double>> Q = { {0.5, 0}, {0, 0.01} };   // ��������Э�������Q��p(w)~N(0, Q)������������ʵ�����еĲ�ȷ����
    static vector<vector<double>> R = { {4, 0}, {0, 0.04} };     // �۲�����Э�������R��p(v)~N(0, R)
    static vector<vector<double>> H = { {1, 0}, {0, 1} };        // ״̬�۲����
    static vector<vector<double>> P = { {4, 0}, {0, 0.04} };     // ״̬����Э�������P 
    static vector<vector<double>> P_posterior = { {4, 0}, {0, 0.04} };   // ״̬�������Э�������
    static vector<vector<double>> eyeMatrix = { {1.0, 0}, {0, 1.0} };  // ������λ����   
    
    int tmp_cnt = 1, tmpDotIdx1 = 0;
    double sum_radar_x = radar_x, sum_radar_y = radar_y;
    sp_mean = RadarData[DotIdx].VeloRadial;
    double speedSum = sp_mean;
    ++j;
    double sp_last = RadarData[DotIdx].VeloRadial;
    if (j < n_Frame - 1)
        tmpDotIdx1 = radarFrameTimeIdx[Lane2FrameIdx[i]] + idx[j];
    else
        return;
    ++tmp_cnt;
    while (j < n_Frame - 1 && std::abs(sp_last - RadarData[tmpDotIdx1].VeloRadial) < 0.001) {
        if ((RadarData[tmpDotIdx1].DistLong - radar_x)*(RadarData[tmpDotIdx1].DistLong - radar_x) + 
            (RadarData[tmpDotIdx1].DistLat - radar_y) * (RadarData[tmpDotIdx1].DistLat - radar_y) < 25) {

            speedSum += RadarData[tmpDotIdx1].VeloRadial;
            sum_radar_x = sum_radar_x + RadarData[tmpDotIdx1].DistLong;
            sum_radar_y = sum_radar_y + RadarData[tmpDotIdx1].DistLat;
            radar_x = sum_radar_x / tmp_cnt;
            radar_y = sum_radar_y / tmp_cnt;
            ++tmp_cnt;
            if (tmp_cnt > 5)
                break;
        }
        ++j;
        tmpDotIdx1 = radarFrameTimeIdx[Lane2FrameIdx[i]] + idx[j];
    }

    if (LastOKIDX > 0) {
        sp_mean = speedSum / (tmp_cnt - 1);
        // ----------------------�����������---------------------
        double deltaT = RadarData[DotIdx].timestamp - LaneRadarTrack[LastOKIDX][6];
        A = { {1, deltaT}, {0, 1} };
        double carDisLog = LaneRadarTrack[LastOKIDX][0];
        double carSpeed = LaneRadarTrack[LastOKIDX][8];
        vector<vector<double>>X_last = { { carDisLog }, { carSpeed } };
        vector<vector<double>>X_prior = matrixMultiply(A, X_last);
        // -----------------����״̬����Э�������P----------------
        vector<vector<double>> A_transpose = transposeMatrix(A);
        vector<vector<double>> P_prior_tmp1 = matrixMultiply(A, P_posterior);
        vector<vector<double>> P_prior_tmp2 = matrixMultiply(P_prior_tmp1, A_transpose);
        vector<vector<double>> P_prior = matrixAddition(P_prior_tmp2, Q);
        // ----------------------���㿨��������-------------------
        R = { {9.0, 0.0}, {0.0, 0.0} };    // �۲�����Э�������R��p(v)~N(0,R)
        vector<vector<double>> H_transpose = transposeMatrix(H);
        vector<vector<double>> K_tmp1 = matrixMultiply(P_prior, H_transpose);
        vector<vector<double>> K_tmp2 = matrixInverse(matrixAddition(matrixMultiply(matrixMultiply(H, P_prior), H_transpose), R));
        vector<vector<double>> K = matrixMultiply(K_tmp1, K_tmp2);
        // ------------------------�������-----------------------
        vector<vector<double>> Z_measure = { {radar_x}, {sp_mean} };
        vector<vector<double>> X_posterior = matrixAddition(X_prior, matrixMultiply(K, matrixSubtraction(Z_measure, matrixMultiply(H, X_prior))));
        radar_x = X_posterior[0][0];
        sp_mean = X_posterior[1][0];
        // --------------- ����״̬����Э�������P-----------------
        P_posterior = matrixMultiply(matrixSubtraction(eyeMatrix, matrixMultiply(K, H)), P_prior);
    }
}

// ������С���˷����������LaneRadarTrack_x, LaneRadarTrack1_yΪy = kx + b����ʽ
void line_plofit(const vector<double>& LaneRadarTrack_x, const vector<double>& LaneRadarTrack_y, double& k, double& b) {
    int num_point = LaneRadarTrack_x.size();
    double sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0, sum_xy = 0.0;
    for (int i = 0; i < num_point; ++i) {
        sum_x += LaneRadarTrack_x[i];
        sum_y += LaneRadarTrack_y[i];
        sum_x2 += LaneRadarTrack_x[i] * LaneRadarTrack_x[i];
        sum_xy += LaneRadarTrack_x[i] * LaneRadarTrack_y[i];
    }
    double denominator = num_point * sum_x2 - sum_x * sum_x;
    if (std::abs(denominator) > 0.00000001) {
        k = (num_point * sum_xy - sum_x * sum_y) / denominator;
        b = (sum_x2 * sum_y - sum_x * sum_xy) / denominator;
    }
    else {
        k = 0;
        b = 0;
    }
}

// ͨ���궨���ݣ�ȷ���״�����ϵ��ԭ��ľ�γ��
void cal_ori_lat_and_long(double& ori_longitude, double& ori_latitude, double theta0, double latitudeMean, const vector<vector<double>>& LaneRadarTrack1, const vector<vector<double>>& LaneRadarTrack2, const vector<vector<double>>& LaneRadarTrack3) {
    double R = 6371393.0;   // ����ƽ���뾶
    double longitude_gap_per_meter = 360 / (2 * PI * R * cos(latitudeMean / 180 * PI)); // ���������һ���ھ����Ͽ�Խ�Ķ���
    double latitude_gap_per_meter = 360 / (2 * PI * R); // �ϱ������һ����γ���Ͽ���Ķ���
    int nLane1 = LaneRadarTrack1.size(), nLane2 = LaneRadarTrack2.size(), nLane3 = LaneRadarTrack3.size();
    double ori_coordinateLong = 0.0, ori_coordinateLat = 0.0;
    for (int i = 0; i < nLane1; ++i) {
        double westDeg = longitude_gap_per_meter * (LaneRadarTrack1[i][0] * cos(theta0) - LaneRadarTrack1[i][1] * sin(theta0));
        double southDeg = latitude_gap_per_meter * (LaneRadarTrack1[i][0] * sin(theta0) + LaneRadarTrack1[i][1] * cos(theta0));
        ori_coordinateLong += (LaneRadarTrack1[i][3] + westDeg);
        ori_coordinateLat += (LaneRadarTrack1[i][2] + southDeg);
    }
    for (int i = 0; i < nLane2; ++i) {
        double westDeg = longitude_gap_per_meter * (LaneRadarTrack2[i][0] * cos(theta0) - LaneRadarTrack2[i][1] * sin(theta0));
        double southDeg = latitude_gap_per_meter * (LaneRadarTrack2[i][0] * sin(theta0) + LaneRadarTrack2[i][1] * cos(theta0));
        ori_coordinateLong += (LaneRadarTrack2[i][3] + westDeg);
        ori_coordinateLat += (LaneRadarTrack2[i][2] + southDeg);
    }
    for (int i = 0; i < nLane3; ++i) {
        double westDeg = longitude_gap_per_meter * (LaneRadarTrack3[i][0] * cos(theta0) - LaneRadarTrack3[i][1] * sin(theta0));
        double southDeg = latitude_gap_per_meter * (LaneRadarTrack3[i][0] * sin(theta0) + LaneRadarTrack3[i][1] * cos(theta0));
        ori_coordinateLong += (LaneRadarTrack3[i][3] + westDeg);
        ori_coordinateLat += (LaneRadarTrack3[i][2] + southDeg);
    }
    ori_longitude = ori_coordinateLong / (nLane1 + nLane2 + nLane3);
    ori_latitude = ori_coordinateLat / (nLane1 + nLane2 + nLane3);
}

// �����������ֱ�ߵĽؾ�
extern void get_intercept(double k, double b1, double b3, double& b_left, double& b_right) {
    double b = b1;
    double lineGap1 = 1.8 + 0; // �����һ���������������·������
    double lineGap2 = 12.6 + 0;    // �����һ�������������Ҳ�·������

    double b_left_tmp1 = b + lineGap1 * sqrt(1 + k*k);
    double b_left_tmp2 = b - lineGap1 * sqrt(1 + k*k);
    if (std::abs(b_left_tmp1 - b3) > std::abs(b_left_tmp2 - b3))
        b_left = b_left_tmp1;
    else
        b_left = b_left_tmp2;

    double b_right_tmp1 = b + lineGap2 * sqrt(1 + k*k);
    double b_right_tmp2 = b - lineGap2 * sqrt(1 + k*k);
    if (std::abs(b_right_tmp1 - b1) > std::abs(b_right_tmp2 - b1))
        b_right = b_right_tmp1;
    else
        b_right = b_right_tmp2;
}

// ����ͬһ��������ǰ��֡���ڳ����ϵ�����������ƫ������������ƫ��
void getMaxVarX_MaxVarY(double maxCarX, double maxCarY, double theta2, double& maxVarX, double& maxVarY) {
    double theta = atan(maxCarY / maxCarX);
    double maxCarLen = sqrt(maxCarX * maxCarX + maxCarY * maxCarY);
    maxVarX = std::abs(maxCarLen * cos(theta2 - theta));
    if (maxVarX < std::abs(maxCarLen* cos(theta2 + theta)))
        maxVarX = std::abs(maxCarLen* cos(theta2 + theta));
    maxVarY = std::abs(maxCarLen * sin(theta2 - theta));
    if (maxVarY < std::abs(maxCarLen* sin(theta2 + theta)))
        maxVarY = std::abs(maxCarLen* sin(theta2 + theta));
}

// ���ָ�����Ƿ�λ��������
bool check_in_zone(double k, double b_left, double b_right, double x, double y) {
    double b_tmp;
    if (b_left < b_right) {
        b_tmp = b_left;
        b_left = b_right;
        b_right = b_tmp;
    }
    if (k * x + b_left > y && k * x + b_right < y) // �ڽ���
        return true;
    else    // �ڽ���
        return false;
}

// ִ�о���˷��ĺ���
vector<vector<double>> matrixMultiply(const vector<vector<double>>& A, const vector<vector<double>>& B) {
    vector<vector<double>> C(A.size(), vector<double>(B[0].size(), 0.0));
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < B[0].size(); ++j)
            for (size_t k = 0; k < B.size(); ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// ִ�о�������ĺ���
vector<vector<double>> matrixInverse(const vector<vector<double>>& A) {

    // ����ά��
    size_t n = A.size();

    // �õ�λ����������þ���
    vector<vector<double>> augmentedMatrix(n, vector<double>(2 * n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            augmentedMatrix[i][j] = A[i][j];
            augmentedMatrix[i][j + n] = (i == j) ? 1.0 : 0.0;
        }
    }

    // ��˹��Ԫ��
    for (size_t i = 0; i < n; ++i) {
        // Make the diagonal contain 1
        double diagonal = augmentedMatrix[i][i];
        for (size_t j = 0; j < 2 * n; ++j) {
            augmentedMatrix[i][j] /= diagonal;
        }

        // ������������һ�б�Ϊ0
        for (size_t k = 0; k < n; ++k) {
            if (k != i) {
                double factor = augmentedMatrix[k][i];
                for (size_t j = 0; j < 2 * n; ++j) {
                    augmentedMatrix[k][j] -= factor * augmentedMatrix[i][j];
                }
            }
        }
    }

    // ȡ�������
    vector<vector<double>> A_inverse(n, vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            A_inverse[i][j] = augmentedMatrix[i][j + n];
    return A_inverse;

}

// ִ�о���ת�õĺ���
vector<vector<double>> transposeMatrix(const vector<vector<double>>& A) {
    size_t rows = A.size();
    size_t cols = A[0].size();

    vector<vector<double>> transpose(cols, vector<double>(rows, 0.0));

    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            transpose[j][i] = A[i][j];

    return transpose;
}

// ִ�о���ӷ��ĺ���
vector<vector<double>> matrixAddition(const vector<vector<double>>& A, const vector<vector<double>>& B) {

    size_t rows = A.size();
    size_t cols = A[0].size();
    vector<vector<double>> result(rows, vector<double>(cols, 0.0));
    for (size_t i = 0; i < rows; ++i) 
        for (size_t j = 0; j < cols; ++j)
            result[i][j] = A[i][j] + B[i][j];
    return result;
}

// ִ�о�������ĺ���
vector<vector<double>> matrixSubtraction(const vector<vector<double>>& A, const vector<vector<double>>& B) {

    size_t rows = A.size();
    size_t cols = A[0].size();
    vector<vector<double>> result(rows, vector<double>(cols, 0.0));
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            result[i][j] = A[i][j] - B[i][j];
    return result;
}

// ִ�о����ڶ�Ӧ���ϸ�ֵ�ĺ���
void setRows(vector<vector<double>>& tracer_Pbuffer, const vector<vector<double>>& P_posterior, int start_row) {
    int rows_to_copy = P_posterior.size();
    for (int i = 0; i < rows_to_copy; ++i) {
        for (int j = 0; j < P_posterior[i].size(); ++j) {
            tracer_Pbuffer[start_row * 2 + i][j] = P_posterior[i][j];
        }
    }
}
