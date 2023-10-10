#include "Solution.h"
#include "struct.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

Solution solution = Solution();
const double PI = 3.141592653589793;

// 创建标定数据与雷达数据之间的映射
vector<vector<double>> Solution::mapLane2Radar(vector<rtk>::iterator begin, vector<rtk>::iterator end) {
    vector<double> radarFrameTime(n_Gap, 0);    // 雷达数据中不同帧的时间
    vector<int> radarFrameTimeIdx(n_Gap, 0); // 雷达数据中不同帧的时间对应的第一个下标
    vector<int> radarFrameCnt(n_Gap, 0);     // 雷达数据中不同帧中点云的数量
    radarFrameTime[0] = RadarData[0].timestamp;
    radarFrameTimeIdx[0] = 0;

    int n_RadarData = RadarData.size();  // 雷达数据总数
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
    vector<int> Lane2RadarFrame(n_Lane, 0);     // 标定数据的时刻到雷达数据的帧的第一个下标的映射
    vector<int> Lane2FrameIdx(n_Lane, 0);     // 标定数据的时刻到雷达数据的帧的下标的映射
    vector<double> Lane2FrameErr(n_Lane, 0);     // 标定数据的时刻到雷达数据的帧的下标的映射的误差
    int weeks = (*begin).Week; // 数据中统一的GPST周数
    int startGapIdx = 0;
    int n_map = 1;  // 最终生成映射的数量

    for (int i = 0; i < n_Lane; ++i) {
        double second_in_weeks = (*(begin + i)).Seconds;
        double UNIX_time = GPST2UNIX(weeks, second_in_weeks);
        while (startGapIdx < n_Gap - 1) {
            if (UNIX_time >= radarFrameTime[startGapIdx] && UNIX_time <= radarFrameTime[startGapIdx+1]) { // 闭区间，以覆盖整个区域
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
            // 当前闭区间没能对应上标定数据的时间戳，则转向下一闭区间
            startGapIdx = startGapIdx + 1;
        }
    }
    vector<int> LaneIdx2Map(n_map, 0);  // 存储将要进行映射的Lane数据的下标
    for (int i = 0, cnt = 0; i < n_Lane;) { // 如果有多个标定数据的时间对应于同一个雷达数据的时间戳，那么取时间差最小的
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

        // 第一检验条件：如果能找到速度相近，且距离上一个OK的点的距离小于5m的两个点，则视作成功找到对应点
        for (int j = 0; j < n_Frame; ++j) {
            if (sp_gap[idx[j]] > 1) // 速度差距过大，提前结束这一检验
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

                        // 该部分用于检测径向速度相近且位置接近的雷达数据
                        find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
                        lastLastOKIdx = LastOKIDX;
                        LastOKIDX = cnt;
                        break;
                    }
                }
            }
        }

        // 第二检验条件：如果能找距离上一个OK的点的距离小于5m或7m的点，则视作成功找到对应点
        if (cnt == 0 && !OKFLAG) {
            DotIdx = radarFrameTimeIdx[Lane2FrameIdx[i]] + idx[0];
            predict_lot(LastOKIDX, DotIdx, RadarData.begin(), LaneRadarTrack, lastLastOKIdx, predict_x, predict_y);
            // 该部分用于检测径向速度相近且位置接近的雷达数据
            find_relate_data(i, 0, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
            LastOKIDX = cnt;
            OKFLAG = 1;
        }
        else if (!OKFLAG) { // 如果下一个点与上一个点的距离小于5m，则接受这个点
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
                            // 该部分用于检测径向速度相近且位置接近的雷达数据
                            find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
                            break;
                        }
                    }
                }
            }
        }

        // 第三检验条件：如果一直找不到下一个点，则直接取路径前进方向上的，距离上一个OK的点距离最小的点
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
            // 该部分用于检测径向速度相近且位置接近的雷达数据
            find_relate_data(i, min_idx, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, radar_x, radar_y, LaneRadarTrack, LastOKIDX, sp_mean);
            OKFLAG = true;
        }
        if (!OKFLAG) {
            LaneRadarTrack.pop_back();
            --n_map; --mapIdx; continue;
        }

        LaneRadarTrack[cnt][0] = radar_x;   // 距离雷达的纵向距离
        LaneRadarTrack[cnt][1] = radar_y;   // 距离雷达的横向距离
        LaneRadarTrack[cnt][2] = (*(begin + i)).Lat;    // 纬度
        LaneRadarTrack[cnt][3] = (*(begin + i)).Lon;    // 经度
        LaneRadarTrack[cnt][4] = (*(begin + i)).NorthVelocity;  // 北向速度
        LaneRadarTrack[cnt][5] = (*(begin + i)).EastVelocity;   // 东向速度
        LaneRadarTrack[cnt][6] = RadarData[radarFrameTimeIdx[Lane2FrameIdx[i]]].timestamp;  // 时间
        LaneRadarTrack[cnt][7] = (*(begin + i)).Hgt;    // 海拔
        LaneRadarTrack[cnt][8] = sp_mean;  // 径向速度
        ++cnt;
    }
    return LaneRadarTrack;
}

// 初始化程序
void Solution::init() {
    //rtk::readRtk("./data/Lane1_rtk.dat", rtkLine1);
    //rtk::readRtk("./data/Lane2_rtk.dat", rtkLine2);
    //rtk::readRtk("./data/Lane3_rtk.dat", rtkLine3);
    //n_Gap = Radar::readRadarData("./data/RadarData.csv", RadarData);
    //vector<vector<double>> LaneRadarTrack1 = mapLane2Radar(rtkLine1.begin(), rtkLine1.end());
    //vector<vector<double>> LaneRadarTrack2 = mapLane2Radar(rtkLine2.begin(), rtkLine2.end());
    //vector<vector<double>> LaneRadarTrack3 = mapLane2Radar(rtkLine3.begin(), rtkLine3.end());
    //double k1 = 0.0, k2 = 0.0, k3 = 0.0, b1 = 0.0, b2 = 0.0, b3 = 0.0, arcTan = 0.0;
    //int nLane1 = LaneRadarTrack1.size(), nLane2 = LaneRadarTrack2.size(), nLane3 = LaneRadarTrack3.size();
    //vector<double> LaneRadarTrackX1(nLane1);
    //vector<double> LaneRadarTrackY1(nLane1);
    //vector<double> LaneRadarTrackAti1(nLane1);
    //vector<double> LaneRadarTrackX2(nLane2);
    //vector<double> LaneRadarTrackY2(nLane2);
    //vector<double> LaneRadarTrackAti2(nLane2);
    //vector<double> LaneRadarTrackX3(nLane3);
    //vector<double> LaneRadarTrackY3(nLane3);
    //vector<double> LaneRadarTrackAti3(nLane3);
    //for (int i = 0; i < nLane1; ++i) {
    //    LaneRadarTrackX1[i] = LaneRadarTrack1[i][0];
    //    LaneRadarTrackY1[i] = LaneRadarTrack1[i][1];
    //    LaneRadarTrackAti1[i] = LaneRadarTrack1[i][7];
    //    arcTan += atan(LaneRadarTrack1[i][4] / LaneRadarTrack1[i][5]);
    //    latitudeMean += LaneRadarTrack1[i][2];
    //}
    //for (int i = 0; i < nLane2; ++i) {
    //    LaneRadarTrackX2[i] = LaneRadarTrack2[i][0];
    //    LaneRadarTrackY2[i] = LaneRadarTrack2[i][1];
    //    LaneRadarTrackAti2[i] = LaneRadarTrack2[i][7];
    //    arcTan += atan(LaneRadarTrack2[i][4] / LaneRadarTrack2[i][5]);
    //    latitudeMean += LaneRadarTrack2[i][2];
    //}
    //for (int i = 0; i < nLane3; ++i) {
    //    LaneRadarTrackX3[i] = LaneRadarTrack3[i][0];
    //    LaneRadarTrackY3[i] = LaneRadarTrack3[i][1];
    //    LaneRadarTrackAti3[i] = LaneRadarTrack3[i][7];
    //    arcTan += atan(LaneRadarTrack3[i][4] / LaneRadarTrack3[i][5]);
    //    latitudeMean += LaneRadarTrack3[i][2];
    //}
    //line_plofit(LaneRadarTrackX1, LaneRadarTrackY1, k1, b1);
    //line_plofit(LaneRadarTrackX2, LaneRadarTrackY2, k2, b2);
    //line_plofit(LaneRadarTrackX3, LaneRadarTrackY3, k3, b3);
    //k = (k1 + k2 + k3) / 3;  // 车道在雷达坐标系上的斜率
    //double theta1 = arcTan / (nLane1 + nLane2 + nLane3);    // 经纬度与雷达坐标系之间的角度偏差
    //theta2 = atan(k);    // 车道与雷达坐标系之间的角度偏差
    //theta0 = theta1 - theta2 + 0.003121;
    //latitudeMean = latitudeMean / (nLane1 + nLane2 + nLane3);   // 平均纬度
    //ori_longitude = 0.0, ori_latitude = 0.0;
    //cal_ori_lat_and_long(ori_longitude, ori_latitude, theta0, latitudeMean, LaneRadarTrack1, LaneRadarTrack2, LaneRadarTrack3);
    //b_left = 0.0, b_right = 0.0;
    //get_intercept(k, b1, b3, b_left, b_right);
    //cosTheta2 = cos(atan(k)), sinTheta2 = sin(atan(k));

    //double kAti1 = 0.0, kAti2 = 0.0, kAti3 = 0.0, bAti1 = 0.0, bAti2 = 0.0, bAti3 = 0.0;
    //line_plofit(LaneRadarTrackX1, LaneRadarTrackAti1, kAti1, bAti1);
    //line_plofit(LaneRadarTrackX2, LaneRadarTrackAti2, kAti2, bAti2);
    //line_plofit(LaneRadarTrackX3, LaneRadarTrackAti3, kAti3, bAti3);
    //kAti = (kAti1 + kAti2 + kAti3) / 3;
    //bAti = (bAti1 + bAti2 + bAti3) / 3;

    // 标定参数预先设置
    n_Gap = Radar::readRadarData("./data/RadarData.csv", RadarData);
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

// 主算法
void Solution::run() {

    // ***********************************卡尔曼滤波器初始化***********************************
    double deltat = 1;
    vector<vector<double>> A = { {1, deltat}, {0, 1} };   // 状态转移矩阵，上一时刻的状态转移到当前时刻
    vector<vector<double>> Q = { {0.5, 0}, {0, 0.01} };   // 过程噪声协方差矩阵Q，p(w)~N(0, Q)，噪声来自真实世界中的不确定性
    vector<vector<double>> R = { {4, 0}, {0, 0.04} };     // 观测噪声协方差矩阵R，p(v)~N(0, R)
    vector<vector<double>> H = { {1, 0}, {0, 1} };        // 状态观测矩阵
    vector<vector<double>> P = { {4, 0}, {0, 0.04} };     // 状态估计协方差矩阵P 
    vector<vector<double>> P_posterior = { {4, 0}, {0, 0.04} };   // 状态后验估计协方差矩阵
    vector<vector<double>> eyeMatrix = { {1.0, 0}, {0, 1.0} };  // 创建单位矩阵

    // ************************************初始化和参数设置************************************
    int n_radar_data = RadarData.size();
    vector<int> frameGapIdx(n_Gap + 1, 0);      // 预先算出每一帧的第一个数据在雷达数据中出现的位置
    frameGapIdx[n_Gap] = n_radar_data;

    double carA = 5.0;    // 设置汽车在一般情况下的最大加速度为carAm / s ^ 2，包括正向与负向的
    double maxCarX = 10.0;// 预设车辆长度为maxCarXm
    double maxCarY = 2.0; // 预设车辆宽度为maxCarYm
    double maxVarX = 0.0, maxVarY = 0.0;
    getMaxVarX_MaxVarY(maxCarX, maxCarY, theta2, maxVarX, maxVarY);    // 设置同一辆车的在前后帧的在车道上的最大纵向距离偏差和最大横向距离偏差

    double maxVarRCS = 1500.0;    // 设置同一辆车的在前后帧的RCS偏差
    double RadarHeight = 7.0;   // 雷达高度
    double RCSMin = 5.0;        // 允许的最小RCS
    double RCSMinZero = 10.0;   // 当雷达数据点的径向速度为0时，允许的最小RCS
    double RCSMinSingle = 5.0; // 当只有一个有效的雷达数据点被探测到时，允许的最小RCS
    double carSpeedVar = 0.4;   // 设置针对同一辆车的，同一帧内的，雷达的径向速度的最大偏差
    int interpolationLimCnt = 1;// 补帧限制，此处，表示连续补帧超过interpolationLimCnt后，不再补帧
    int interpolationLimM = 400;// 补帧限制，米，表示超过interpolationLimM后，不再补帧
    int maxFailTime = 20;        // 允许追踪失败的最大次数
    // ****************************************************************************************

    double lastTime = 0;
    for (int i = 0, cnt = 0; i < n_radar_data; ++i) {
        if (lastTime == RadarData[i].timestamp)
            continue;
        frameGapIdx[cnt] = i;
        lastTime = RadarData[i].timestamp;
        ++cnt;
    }

    vector<int> carID_buffer(10000, 1); // 记录各carID出现的次数
    vector<vector<double>> tracer_Pbuffer(500, vector<double>(2, 0));
    vector<vector<int>> tracer_buffer(500, vector<int>(4, 0)); // 第1列记录在前一帧追踪的存放在all_res中的编号，第2列记录对应的在RadarData中的编号，第3列记录连续追踪失败的次数，第4列记录当前连续追踪点数
    int tracer_pointer = -1;     // tracer_pointer永远指向buffer中的最后一个有效元素，且其前面均为有效元素
    int data_idx = -1;
    vector<res> all_res(n_radar_data / 10);         // 记录全体结果
    vector<float> maxCarsLen(n_radar_data / 10);    // 记录最大车长信息
    int carUniqueId = -1;
    for (int cnt = 0; cnt < n_Gap; ++cnt) {
        int frameStart = frameGapIdx[cnt];  // 当前帧的在雷达数据的起始位
        int nFrame = frameGapIdx[cnt + 1] - frameGapIdx[cnt]; // 该帧的帧数
        vector<int> OKIndex;
        for (int i = 0; i < nFrame; ++i) {
            double y = RadarData[frameStart + i].DistLat, x = RadarData[frameStart + i].DistLong;
            double rcs = RadarData[frameStart + i].RCS;
            if (check_in_zone(k, b_left, b_right, x, y) && rcs > RCSMin) {
                if (RadarData[frameStart + i].VeloRadial == 0 && RadarData[frameStart + i].RCS < RCSMinZero) // 如果该点速度为0，且RCS < RCSMinZero，去掉
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
        double nowT = curFrameData[0].timestamp;    // 记录现在的时刻

        // 跟踪算法
        int i = 0;
        while (i <= tracer_pointer && tracer_pointer > -1) {  // 在跟踪队列里，一个个比对当前帧的数据，匹配成功的数据点，将被拿走
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
            int j = 0;  // j指向curFrameData数据中的数据点

            while (j < OKIndexPointer_len) {    // 在OKIndex里寻找能与正在追踪的车辆匹配的点，找到之后，把它从OKIndex中删除
                if (BlockIndex[j]) {
                    ++j; continue;
                }
                if (std::abs(carSpeed - curFrameData[sortedIdx[j]].VeloRadial) / cosTheta2 > deltaT * carA) { // (deltaT在0.025~0.2之间)
                    ++j; continue;
                }
                if (std::abs(carDisLat - curFrameData[sortedIdx[j]].DistLat) > maxVarY) {
                    ++j; continue;
                }
                if (std::abs(carRCS - curFrameData[sortedIdx[j]].RCS) > maxVarRCS) {
                    ++j; continue;
                }
                double v_true = v_true_cal(carDisLog, carDisLat, RadarHeight, carSpeed, cosTheta2); // 计算车辆的实际速度，默认车辆沿着车道方向行驶
                double X_predict = carDisLog + deltaT * v_true * cosTheta2;    // 车辆的预测纵向位置
                if (std::abs(curFrameData[sortedIdx[j]].DistLong - X_predict) > maxVarX) {
                    ++j; continue;
                }

                // 匹配成功
                coupleFlag = true;
                double X_mean = curFrameData[sortedIdx[j]].DistLong;  double X_sum = X_mean;
                double Y_mean = curFrameData[sortedIdx[j]].DistLat;  double Y_sum = Y_mean;
                double sp_mean = curFrameData[sortedIdx[j]].VeloRadial; double sp_sum = sp_mean;
                double RCS_mean = curFrameData[sortedIdx[j]].RCS; double RCS_sum = RCS_mean;
                double Xmin = X_mean, Xmax = X_mean;
                BlockIndex[j] = true;   // 匹配成功的数据点，将被拿走
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
                    BlockIndex[j] = true;  // 匹配成功的数据点，将被拿走
                    ++j;
                }
                ++data_idx;
                int RadarDataID = frameStart + OKIndex[sortedIdx[jStart]];
                if (sp_mean == 0) {
                    X_mean = carDisLog;
                    Y_mean = carDisLat;
                }

                //// ----------------------进行先验估计---------------------
                //A = { {1, deltaT}, {0, 1} };
                //vector<vector<double>>X_last = { { carDisLog }, { carSpeed } };
                //vector<vector<double>>X_prior = matrixMultiply(A, X_last);
                //// -----------------计算状态估计协方差矩阵P----------------
                //vector<vector<double>> A_transpose = transposeMatrix(A);
                //vector<vector<double>> P_prior_tmp1 = matrixMultiply(A, P_posterior);
                //vector<vector<double>> P_prior_tmp2 = matrixMultiply(P_prior_tmp1, A_transpose);
                //vector<vector<double>> P_prior = matrixAddition(P_prior_tmp2, Q);
                //// ----------------------计算卡尔曼增益-------------------
                //R = { {std::max(maxCarLen * maxCarLen / 4.0, 9.0), 0}, {0.0, 0.0} };    // 观测噪声协方差矩阵R，p(v)~N(0,R)
                //vector<vector<double>> H_transpose = transposeMatrix(H);
                //vector<vector<double>> K_tmp1 = matrixMultiply(P_prior, H_transpose);
                //vector<vector<double>> K_tmp2 = matrixInverse(matrixAddition(matrixMultiply(matrixMultiply(H, P_prior), H_transpose), R));
                //vector<vector<double>> K = matrixMultiply(K_tmp1, K_tmp2);
                //// ------------------------后验估计-----------------------
                //vector<vector<double>> Z_measure = { {X_mean}, {sp_mean} };
                //vector<vector<double>> X_posterior = matrixAddition(X_prior, matrixMultiply(K, matrixSubtraction(Z_measure, matrixMultiply(H, X_prior))));

                //if (X_posterior[0][0] < 400) {
                //    X_mean = X_posterior[0][0];
                //    sp_mean = X_posterior[1][0];
                //}
                //else {      // 如果滤波值>400，则不采用滤波值，并强行结束追踪
                //    coupleFlag = 0;
                //    tracer_buffer[i][2] = maxFailTime + 1;
                //}

                //// --------------- 更新状态估计协方差矩阵P-----------------
                //P_posterior = matrixMultiply(matrixSubtraction(eyeMatrix, matrixMultiply(K, H)), P_prior);

                maxCarsLen[data_idx] = maxCarLen;
                ++carID_buffer[carID];
                writeSingleResult(nowT, carID, X_mean, Y_mean, carDisLat, RadarHeight, sp_mean, RCS_mean, RadarDataID, all_res, data_idx, maxCarLen);
                // 更新在缓冲区的数据
                tracer_buffer[i][0] = data_idx;
                tracer_buffer[i][1] = RadarDataID;
                tracer_buffer[i][2] = 0;    // 连续追踪失败次数归零
                ++tracer_buffer[i][3];
                setRows(tracer_Pbuffer, P_posterior, i);
                break;
            }
            if (!coupleFlag) {  // 匹配失败，先试着留在跟踪队列里，如果持续失败，该跟踪数据从缓冲区中被移除
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

        // 识别算法
        int j = 0;  // j指向curFrameData数据中的数据点
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

            // 在数据中认为有可能发现车辆
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
                BlockIndex[j] = true; // 匹配成功的数据点，将被拿走
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

// 写下单条结果
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

// 根据横距纵向距离与横向距离，获得相应的经纬度坐标
void Solution::getCoordinate(double distLong, double distLati, double& longitude, double& latitude) {
    double R = 6371393.0;   // 地球平均半径
    double longitude_gap_per_meter = 360 / (2 * PI * R * cos(latitudeMean / 180 * PI)); // 东西方向的一米在经度上跨越的度数
    double latitude_gap_per_meter = 360 / (2 * PI * R); // 南北方向的一米在纬度上跨域的度数
    double westDeg = longitude_gap_per_meter * (distLong * cos(theta0) - distLati * sin(theta0));
    double southDeg = latitude_gap_per_meter * (distLong * sin(theta0) + distLati * cos(theta0));
    longitude = ori_longitude + westDeg;
    latitude = ori_latitude + southDeg;
}

// GPST时间转换为UNIX时间
double GPST2UNIX(int weeks, double second_in_weeks) {
    int UNIX_TO_GPST = 315964800;
    return UNIX_TO_GPST + 24 * 60 * 60 * 7 * weeks + second_in_weeks - 18;
}

// 预测车辆在下一帧出现的位置，将历史速度与当前速度做一个权衡，用权衡后的速度乘以经过的时间，加上原本的位置，即得到预测位置
void predict_lot(int LastOKIDX, int tmpDotIdx, vector<Radar>::iterator radarBegin, const vector<vector<double>>& LaneRadarTrack, int lastLastOKIdx, double& predict_x, double& predict_y) {
    if (lastLastOKIdx == LastOKIDX) {
        predict_x = (*(radarBegin + tmpDotIdx)).DistLong;
        predict_y = (*(radarBegin + tmpDotIdx)).DistLat;
    }
    else {
        double delta_t1 = (*(radarBegin + tmpDotIdx)).timestamp - LaneRadarTrack[LastOKIDX][6]; // 当前时间与上一可靠轨迹点的时间之差
        double delta_t2 = LaneRadarTrack[LastOKIDX][6] - LaneRadarTrack[0][6];  // 上一可靠轨迹点与第一个可靠轨迹点时间之差
        double delta_t3 = LaneRadarTrack[LastOKIDX][6] - LaneRadarTrack[lastLastOKIdx][6];  // 上一可靠轨迹点与前前可靠轨迹点时间之差

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

// 通过车辆当前的x，y，z坐标，以及径向速度，计算其实际速度（统一默认车辆沿着车道行驶，在误差可接受的范围内，若默认车道方向就是x轴正向方向，此时alpha=1）
double v_true_cal(double x, double y, double z, double v_r, double alpha) {
    double cosTheta = std::abs(x) / sqrt(x*x + y*y + z*z); // 在雷达数据里，x必然大于0
    double v = (v_r / cosTheta) / alpha;
    return v;
}

// 该部分用于检测径向速度相近且位置接近的雷达数据
void find_relate_data(int i, int j, const vector<Radar>& RadarData, const vector<int>& radarFrameTimeIdx, const vector<int>& Lane2FrameIdx, int DotIdx, int n_Frame, const vector<int>& idx, double& radar_x, double& radar_y, const vector<vector<double>>& LaneRadarTrack, int LastOKIDX, double& sp_mean) {
    
    // ***********************************卡尔曼滤波器初始化***********************************
    double deltat = 1;
    static vector<vector<double>> A = { {1, deltat}, {0, 1} };   // 状态转移矩阵，上一时刻的状态转移到当前时刻
    static vector<vector<double>> Q = { {0.5, 0}, {0, 0.01} };   // 过程噪声协方差矩阵Q，p(w)~N(0, Q)，噪声来自真实世界中的不确定性
    static vector<vector<double>> R = { {4, 0}, {0, 0.04} };     // 观测噪声协方差矩阵R，p(v)~N(0, R)
    static vector<vector<double>> H = { {1, 0}, {0, 1} };        // 状态观测矩阵
    static vector<vector<double>> P = { {4, 0}, {0, 0.04} };     // 状态估计协方差矩阵P 
    static vector<vector<double>> P_posterior = { {4, 0}, {0, 0.04} };   // 状态后验估计协方差矩阵
    static vector<vector<double>> eyeMatrix = { {1.0, 0}, {0, 1.0} };  // 创建单位矩阵   
    
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
        // ----------------------进行先验估计---------------------
        double deltaT = RadarData[DotIdx].timestamp - LaneRadarTrack[LastOKIDX][6];
        A = { {1, deltaT}, {0, 1} };
        double carDisLog = LaneRadarTrack[LastOKIDX][0];
        double carSpeed = LaneRadarTrack[LastOKIDX][8];
        vector<vector<double>>X_last = { { carDisLog }, { carSpeed } };
        vector<vector<double>>X_prior = matrixMultiply(A, X_last);
        // -----------------计算状态估计协方差矩阵P----------------
        vector<vector<double>> A_transpose = transposeMatrix(A);
        vector<vector<double>> P_prior_tmp1 = matrixMultiply(A, P_posterior);
        vector<vector<double>> P_prior_tmp2 = matrixMultiply(P_prior_tmp1, A_transpose);
        vector<vector<double>> P_prior = matrixAddition(P_prior_tmp2, Q);
        // ----------------------计算卡尔曼增益-------------------
        R = { {9.0, 0.0}, {0.0, 0.0} };    // 观测噪声协方差矩阵R，p(v)~N(0,R)
        vector<vector<double>> H_transpose = transposeMatrix(H);
        vector<vector<double>> K_tmp1 = matrixMultiply(P_prior, H_transpose);
        vector<vector<double>> K_tmp2 = matrixInverse(matrixAddition(matrixMultiply(matrixMultiply(H, P_prior), H_transpose), R));
        vector<vector<double>> K = matrixMultiply(K_tmp1, K_tmp2);
        // ------------------------后验估计-----------------------
        vector<vector<double>> Z_measure = { {radar_x}, {sp_mean} };
        vector<vector<double>> X_posterior = matrixAddition(X_prior, matrixMultiply(K, matrixSubtraction(Z_measure, matrixMultiply(H, X_prior))));
        radar_x = X_posterior[0][0];
        sp_mean = X_posterior[1][0];
        // --------------- 更新状态估计协方差矩阵P-----------------
        P_posterior = matrixMultiply(matrixSubtraction(eyeMatrix, matrixMultiply(K, H)), P_prior);
    }
}

// 利用最小二乘法，拟合数据LaneRadarTrack_x, LaneRadarTrack1_y为y = kx + b的形式
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

// 通过标定数据，确定雷达坐标系的原点的经纬度
void cal_ori_lat_and_long(double& ori_longitude, double& ori_latitude, double theta0, double latitudeMean, const vector<vector<double>>& LaneRadarTrack1, const vector<vector<double>>& LaneRadarTrack2, const vector<vector<double>>& LaneRadarTrack3) {
    double R = 6371393.0;   // 地球平均半径
    double longitude_gap_per_meter = 360 / (2 * PI * R * cos(latitudeMean / 180 * PI)); // 东西方向的一米在经度上跨越的度数
    double latitude_gap_per_meter = 360 / (2 * PI * R); // 南北方向的一米在纬度上跨域的度数
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

// 求出两条界限直线的截距
extern void get_intercept(double k, double b1, double b3, double& b_left, double& b_right) {
    double b = b1;
    double lineGap1 = 1.8 + 0; // 假设第一车道的中央与左侧路基距离
    double lineGap2 = 12.6 + 0;    // 假设第一车道的中央与右侧路基距离

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

// 设置同一辆车的在前后帧的在车道上的最大纵向距离偏差和最大横向距离偏差
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

// 检查指定点是否位于区域内
bool check_in_zone(double k, double b_left, double b_right, double x, double y) {
    double b_tmp;
    if (b_left < b_right) {
        b_tmp = b_left;
        b_left = b_right;
        b_right = b_tmp;
    }
    if (k * x + b_left > y && k * x + b_right < y) // 在界内
        return true;
    else    // 在界外
        return false;
}

// 执行矩阵乘法的函数
vector<vector<double>> matrixMultiply(const vector<vector<double>>& A, const vector<vector<double>>& B) {
    vector<vector<double>> C(A.size(), vector<double>(B[0].size(), 0.0));
    for (size_t i = 0; i < A.size(); ++i)
        for (size_t j = 0; j < B[0].size(); ++j)
            for (size_t k = 0; k < B.size(); ++k)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// 执行矩阵求逆的函数
vector<vector<double>> matrixInverse(const vector<vector<double>>& A) {

    // 设置维度
    size_t n = A.size();

    // 用单位矩阵来增广该矩阵
    vector<vector<double>> augmentedMatrix(n, vector<double>(2 * n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            augmentedMatrix[i][j] = A[i][j];
            augmentedMatrix[i][j + n] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 高斯消元法
    for (size_t i = 0; i < n; ++i) {
        // Make the diagonal contain 1
        double diagonal = augmentedMatrix[i][i];
        for (size_t j = 0; j < 2 * n; ++j) {
            augmentedMatrix[i][j] /= diagonal;
        }

        // 让其余行在这一列变为0
        for (size_t k = 0; k < n; ++k) {
            if (k != i) {
                double factor = augmentedMatrix[k][i];
                for (size_t j = 0; j < 2 * n; ++j) {
                    augmentedMatrix[k][j] -= factor * augmentedMatrix[i][j];
                }
            }
        }
    }

    // 取出逆矩阵
    vector<vector<double>> A_inverse(n, vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            A_inverse[i][j] = augmentedMatrix[i][j + n];
    return A_inverse;

}

// 执行矩阵转置的函数
vector<vector<double>> transposeMatrix(const vector<vector<double>>& A) {
    size_t rows = A.size();
    size_t cols = A[0].size();

    vector<vector<double>> transpose(cols, vector<double>(rows, 0.0));

    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            transpose[j][i] = A[i][j];

    return transpose;
}

// 执行矩阵加法的函数
vector<vector<double>> matrixAddition(const vector<vector<double>>& A, const vector<vector<double>>& B) {

    size_t rows = A.size();
    size_t cols = A[0].size();
    vector<vector<double>> result(rows, vector<double>(cols, 0.0));
    for (size_t i = 0; i < rows; ++i) 
        for (size_t j = 0; j < cols; ++j)
            result[i][j] = A[i][j] + B[i][j];
    return result;
}

// 执行矩阵减法的函数
vector<vector<double>> matrixSubtraction(const vector<vector<double>>& A, const vector<vector<double>>& B) {

    size_t rows = A.size();
    size_t cols = A[0].size();
    vector<vector<double>> result(rows, vector<double>(cols, 0.0));
    for (size_t i = 0; i < rows; ++i)
        for (size_t j = 0; j < cols; ++j)
            result[i][j] = A[i][j] - B[i][j];
    return result;
}

// 执行矩阵在对应行上赋值的函数
void setRows(vector<vector<double>>& tracer_Pbuffer, const vector<vector<double>>& P_posterior, int start_row) {
    int rows_to_copy = P_posterior.size();
    for (int i = 0; i < rows_to_copy; ++i) {
        for (int j = 0; j < P_posterior[i].size(); ++j) {
            tracer_Pbuffer[start_row * 2 + i][j] = P_posterior[i][j];
        }
    }
}
