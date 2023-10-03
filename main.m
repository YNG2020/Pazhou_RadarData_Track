% 此部分尽量用C++风格写matlab
% 卡尔曼滤波器
deltat = 1;
A = [1 deltat; 0 1];    % 状态转移矩阵，上一时刻的状态转移到当前时刻
Q = [0.5 0; 0 0.01];   % 过程噪声协方差矩阵Q，p(w)~N(0,Q)，噪声来自真实世界中的不确定性
R = [4 0; 0 0.04];  % 观测噪声协方差矩阵R，p(v)~N(0,R)
H = [1 0; 0 1]; % 状态观测矩阵
P = R;
P_posterior = P;
LaneRadarTrack1 = LaneRadarTrack1(~isnan(LaneRadarTrack1(:, 1)), :);
LaneRadarTrack2 = LaneRadarTrack2(~isnan(LaneRadarTrack2(:, 2)), :);
LaneRadarTrack3 = LaneRadarTrack3(~isnan(LaneRadarTrack3(:, 3)), :);
[k1, b1] = line_plofit(LaneRadarTrack1(:, 2), LaneRadarTrack1(:, 3));
[k2, b2] = line_plofit(LaneRadarTrack2(:, 2), LaneRadarTrack2(:, 3));
[k3, b3] = line_plofit(LaneRadarTrack3(:, 2), LaneRadarTrack3(:, 3));
k = (k1 + k2 + k3) / 3;     % 车道在雷达坐标系上的斜率
tan1 = LaneRadarTrack1(:, 6) ./ LaneRadarTrack1(:, 7);
tan2 = LaneRadarTrack2(:, 6) ./ LaneRadarTrack2(:, 7);
tan3 = LaneRadarTrack3(:, 6) ./ LaneRadarTrack3(:, 7);
arcTan1 = atan(tan1);
arcTan2 = atan(tan2);
arcTan3 = atan(tan3);
theta1 = mean([arcTan1; arcTan2; arcTan3]); % 经纬度与雷达坐标系之间的角度偏差
theta2 = atan(k);   % 车道与雷达坐标系之间的角度偏差
delta = 0 + 2416 / 1000000;
theta0 = theta1 - theta2 + delta;   % 经纬度与车道之间的角度偏差
latitudeMean = mean([LaneRadarTrack1(:, 4); LaneRadarTrack2(:, 4); LaneRadarTrack3(:, 4)]); % 平均纬度
[ori_longitude, ori_latitude] = cal_ori_lat_and_long(theta0, LaneRadarTrack1, LaneRadarTrack2, LaneRadarTrack3);
[b_left, b_right] = get_intercept(k, b1, b3);
cosTheta2 = cos(atan(k));
sinTheta2 = sin(atan(k));

[kAti1, bAti1] = line_plofit(LaneRadarTrack1(:, 2), LaneRadarTrack1(:, 15));
[kAti2, bAti2] = line_plofit(LaneRadarTrack2(:, 2), LaneRadarTrack2(:, 15));
[kAti3, bAti3] = line_plofit(LaneRadarTrack3(:, 2), LaneRadarTrack3(:, 15));
kAti = (kAti1 + kAti2 + kAti3) / 3;
bAti = (bAti1 + bAti2 + bAti3) / 3;

n_radar_data = size(RadarData, 1);
n_Gap = length(unique(RadarData(:,1)));  % 预先算出帧数，以预先进行内存分配，matlab用vector类的话，不用这么这么做
frameGapIdx = zeros(n_Gap + 1, 1);  % 预先算出每一帧的第一个数据在雷达数据中出现的位置
frameGapIdx(n_Gap + 1) = n_radar_data + 1;

carA = 5;   % 设置汽车在一般情况下的最大加速度为carAm/s^2，包括正向与负向的
maxCarX = 10;   % 预设车辆长度为maxCarXm
maxCarY = 2;    % 预设车辆宽度为maxCarYm
[maxVarX, maxVarY] = getMaxVarX_MaxVarY(maxCarX, maxCarY, theta2);  % 设置同一辆车的在前后帧的在车道上的最大纵向距离偏差和最大横向距离偏差

maxVarRCS = 1500; % 设置同一辆车的在前后帧的RCS偏差
RadarHeight = 7;    % 雷达高度
RCSMin = 5;   % 允许的最小RCS
RCSMinZero = 10;    % 当雷达数据点的径向速度为0时，允许的最小RCS
RCSMinSingle = 5;  % 当只有一个有效的雷达数据点被探测到时，允许的最小RCS
carSpeedVar = 0.4;  % 设置针对同一辆车的，同一帧内的，雷达的径向速度的最大偏差
interpolationLimCnt = 1;  % 补帧限制，此处，表示连续补帧超过interpolationLimCnt后，不再补帧
interpolationLimM = 400;   % 补帧限制，米，表示超过interpolationLimM后，不再补帧
maxFailTime = 20;       % 允许追踪失败的最大次数

cnt = 1;
lastTime = 0;
for i = 1 : n_radar_data
    if (lastTime == RadarData(i, 1))
        continue;
    end
    frameGapIdx(cnt) = i;
    lastTime = RadarData(i, 1);
    cnt = cnt + 1;
end
cnt2 = 0;

carID_buffer = zeros(10000, 1); % 记录各carID出现的次数
tracer_Pbuffer = zeros(1000, 2);    
tracer_buffer = zeros(1000, 4); % 第1列记录在前一帧追踪的存放在all_res中的编号，第2列记录对应的在RadarData中的编号，第3列记录连续追踪失败的次数，第4列记录当前连续追踪点数
tracer_pointer = 0; % tracer_pointer永远指向buffer中的最后一个有效元素，且其前面均为有效元素
data_idx = 0;
all_res = zeros(n_radar_data, 16);
carUniqueId = -1;
for cnt = 1 : n_Gap
    frameStart = frameGapIdx(cnt);  % 当前帧的在雷达数据的起始位
    nFrame = frameGapIdx(cnt + 1) - frameGapIdx(cnt);   % 该帧的帧数
    tmpIndex = zeros(nFrame, 1);
    for i = 1 : nFrame
        if check_in_zone(k, b_left, b_right, RadarData(frameStart + i - 1, 3), RadarData(frameStart + i - 1, 4)) && RadarData(frameStart + i - 1, 6) > RCSMin  % 在区域内，且RCS > RCSMin
            if RadarData(frameStart + i - 1, 5) == 0 && RadarData(frameStart + i - 1, 6) < RCSMinZero   % 如果该点速度为0，且RCS<RCSMinZero，去掉
                continue;
            end
            tmpIndex(i) = 1;        % 在C++中，寻找OKIndex应该用push来代替
        end
    end
    OKIndex = find(tmpIndex) - 1;
    OKIndexPointer_len = length(OKIndex);
    BlockIndex = zeros(OKIndexPointer_len, 1);
    [curFrameData, sortedIdx] = sortrows(RadarData(frameStart + OKIndex, :), [5 3 4 6]); % 挑出合理的雷达数据点，同时先后对速度，纵向距离，横向距离，RCS升序排序
    OKIndex = OKIndex(sortedIdx);
    nowT = curFrameData(1, 1);  % 记录现在的时刻

    %%%%% 跟踪算法
    i = 1;
    while i <= tracer_pointer && tracer_pointer > 0  % 在跟踪队列里，一个个比对当前帧的数据，匹配成功的数据点，将被拿走
        dataID = tracer_buffer(i, 1);
        radarDataID = tracer_buffer(i, 2);
        carID = all_res(dataID, 2);
        carDisLog = all_res(dataID, 3);
        carDisLat = all_res(dataID, 4);
        carSpeed = RadarData(radarDataID, 5);
        carRCS = all_res(dataID, 7);
        carClass = all_res(dataID, 8);
        deltaT = nowT - all_res(dataID, 1);
        maxCarLen = all_res(dataID, 16);
        P_posterior = tracer_Pbuffer(i * 2 - 1 : i * 2, 1 : 2);
        coupleFlag = 0; j = 1;  % j指向curFrameData数据中的数据点

        while j <= OKIndexPointer_len  % 在OKIndex里寻找能与正在追踪的车辆匹配的点，找到之后，把它从OKIndex中删除
            if BlockIndex(j)
                j = j + 1; continue;
            end
            if abs(carSpeed - curFrameData(j, 5)) / cosTheta2 > deltaT * carA %(deltaT在0.025~0.2之间)
                j = j + 1; continue;
            end
            if abs(carDisLat - curFrameData(j, 4)) > maxVarY
                j = j + 1; continue;
            end
            if abs(carRCS - curFrameData(j, 6)) > maxVarRCS
                j = j + 1; continue;
            end
            v_true = v_true_cal(carDisLog, carDisLat, RadarHeight, carSpeed, cosTheta2);   % 计算车辆的实际速度，默认车辆沿着车道方向行驶
            X_predict = carDisLog + deltaT * v_true * cosTheta2;    % 车辆的预测纵向位置
            if abs(curFrameData(j, 3) - X_predict) > maxVarX
                j = j + 1; continue;
            end

            % 匹配成功
            coupleFlag = 1;
            X_mean = curFrameData(j, 3);  X_sum = X_mean;
            Y_mean = curFrameData(j, 4);  Y_sum = Y_mean;
            sp_mean = curFrameData(j, 5); sp_sum = sp_mean;
            RCS_mean = curFrameData(j, 6);RCS_sum = RCS_mean;
            Xmin = X_mean; Xmax = X_mean;
            BlockIndex(j) = 1;    % 匹配成功的数据点，将被拿走
            jStart = j; j = j + 1; tmpCnt = 1;
            while (j <= OKIndexPointer_len)
                if BlockIndex(j)
                    j = j + 1; continue;
                end
                if abs(X_mean - curFrameData(j, 3)) > maxVarX
                    j = j + 1; continue;
                end
                if abs(Y_mean - curFrameData(j, 4)) > maxVarY
                    j = j + 1; continue;
                end
                if abs(sp_mean - curFrameData(j, 5)) > carSpeedVar
                    j = j + 1; continue;
                end
                if (abs(RCS_mean - curFrameData(j, 6))) > maxVarRCS
                    j = j + 1; continue;
                end
                if curFrameData(j, 3) < Xmin
                    Xmin = curFrameData(j, 3);
                end
                if curFrameData(j, 3) > Xmax
                    Xmax = curFrameData(j, 3);
                end
                carLen = (Xmax - Xmin) / cosTheta2;
                if maxCarLen < carLen
                    maxCarLen = carLen;
                end
                tmpCnt = tmpCnt + 1;
                X_sum = X_sum + curFrameData(j, 3);   X_mean = X_sum / tmpCnt;       
                Y_sum = Y_sum + curFrameData(j, 4);   Y_mean = Y_sum / tmpCnt;
                sp_sum = sp_sum + curFrameData(j, 5);  sp_mean = sp_sum / tmpCnt;
                RCS_sum = RCS_sum + curFrameData(j, 6);RCS_mean = RCS_sum / tmpCnt;
                BlockIndex(j) = 1;    % 匹配成功的数据点，将被拿走
                j = j + 1;
            end
            data_idx = data_idx + 1;
            RadarDataID = frameStart + OKIndex(jStart);
            if sp_mean == 0
                X_mean = carDisLog;
                Y_mean = carDisLat;
            end
            
            % ----------------------进行先验估计---------------------
            A = [1 deltaT; 0 1];
            X_prior = A * [carDisLog; carSpeed];
            % 计算状态估计协方差矩阵P
            P_prior = A * P_posterior * A' + Q;
            % ----------------------计算卡尔曼增益
            R = [max(maxCarLen * maxCarLen / 4, 9) 0; 0 0];  % 观测噪声协方差矩阵R，p(v)~N(0,R)
            K = P_prior * H' * inv(H * P_prior * H' + R);
            % ---------------------后验估计------------
            Z_measure = zeros(2, 1);
            Z_measure(1) = X_mean;
            Z_measure(2) = sp_mean;
            X_posterior = X_prior + K * (Z_measure - H * X_prior);

            if X_posterior(1) < 400 
                X_mean = X_posterior(1);
                sp_mean = X_posterior(2);
            else                    % 如果滤波值>400，则不采用滤波值，并强行结束追踪
                coupleFlag = 0;
                tracer_buffer(i, 3) = maxFailTime + 1;
            end

            % 更新状态估计协方差矩阵P
            P_posterior = (eye(2, 2) - K * H) * P_prior;
            carID_buffer(carID + 1) = carID_buffer(carID + 1) + 1;
            all_res(data_idx, :) = writeResult(nowT, carID, X_mean, Y_mean, ...
                RadarHeight, sp_mean, cosTheta2, sinTheta2, carDisLat, RCS_mean, ...
                theta0, latitudeMean, ori_longitude, ori_latitude, maxCarLen,...
                kAti, bAti, RadarDataID);
            % 更新在缓冲区的数据
            tracer_buffer(i, 1) = data_idx;
            tracer_buffer(i, 2) = RadarDataID;
            tracer_buffer(i, 3) = 0;    % 连续追踪失败次数归零
            tracer_buffer(i, 4) = tracer_buffer(i, 4) + 1;
            tracer_Pbuffer(i * 2 - 1 : i * 2, 1 : 2) = P_posterior;
            break;
        end
        if coupleFlag == 0      % 匹配失败，先试着留在跟踪队列里，如果持续失败，该跟踪数据从缓冲区中被移除
            if tracer_buffer(i, 3) > maxFailTime
                tracer_buffer(i, 1) = tracer_buffer(tracer_pointer, 1);
                tracer_buffer(i, 2) = tracer_buffer(tracer_pointer, 2);
                tracer_buffer(i, 3) = tracer_buffer(tracer_pointer, 3);
                tracer_buffer(i, 4) = tracer_buffer(tracer_pointer, 4);
                tracer_Pbuffer(i * 2 - 1 : i * 2, 1 : 2) = tracer_Pbuffer(tracer_pointer * 2 - 1 : tracer_pointer * 2, 1 : 2);
                tracer_pointer = tracer_pointer - 1;
            else
                tracer_buffer(i, 3) = tracer_buffer(i, 3) + 1;  % 连续追踪失败次数+1
                i = i + 1;
            end
        else
            i = i + 1;
        end
    end
    
    %%%%% 识别算法
    j = 1;  % j指向curFrameData数据中的数据点
    while j <= OKIndexPointer_len
        if BlockIndex(j)
            j = j + 1; continue;
        end
        if curFrameData(j, 5) == 0
            BlockIndex(j) = 1; j = j + 1;  continue;
        end
        if curFrameData(j, 6) < RCSMinSingle
            BlockIndex(j) = 1; j = j + 1;  continue;
        end

        % 在数据中认为有可能发现车辆
        X_mean = curFrameData(j, 3);  X_sum = X_mean;
        Y_mean = curFrameData(j, 4);  Y_sum = Y_mean;
        sp_mean = curFrameData(j, 5); sp_sum = sp_mean;
        RCS_mean = curFrameData(j, 6);RCS_sum = RCS_mean;
        coupleFlag = 0; jStart = j; Xmin = X_mean; Xmax = X_mean;
        j = j + 1; tmpCnt = 1;
        while (j <= OKIndexPointer_len)
            if BlockIndex(j)
                j = j + 1; continue;
            end
            if abs(X_mean - curFrameData(j, 3)) > maxVarX
                j = j + 1; continue;
            end
            if abs(Y_mean - curFrameData(j, 4)) > maxVarY
                j = j + 1; continue;
            end
            if abs(sp_mean - curFrameData(j, 5)) > carSpeedVar
                j = j + 1; continue;
            end
            if curFrameData(j, 3) < Xmin
                Xmin = curFrameData(j, 3);
            end
            if curFrameData(j, 3) > Xmax
                Xmax = curFrameData(j, 3);
            end
            coupleFlag = 1;  tmpCnt = tmpCnt + 1;
            X_sum = X_sum + curFrameData(j, 3);   X_mean = X_sum / tmpCnt;       
            Y_sum = Y_sum + curFrameData(j, 4);   Y_mean = Y_sum / tmpCnt;
            sp_sum = sp_sum + curFrameData(j, 5);  sp_mean = sp_sum / tmpCnt;
            RCS_sum = RCS_sum + curFrameData(j, 6);RCS_mean = RCS_sum / tmpCnt;
            BlockIndex(j) = 1;    % 匹配成功的数据点，将被拿走
            j = j + 1;
        end
        if coupleFlag
            BlockIndex(jStart) = 1;
            data_idx = data_idx + 1;
            carUniqueId = carUniqueId + 1;
            maxCarLen = (Xmax - Xmin) / cosTheta2;
            RadarDataID = frameStart + OKIndex(jStart);
            carID_buffer(carUniqueId + 1) = 1;
            all_res(data_idx, :) = writeResult(nowT, carUniqueId, X_mean, Y_mean, ...
                RadarHeight, sp_mean, cosTheta2, sinTheta2, Y_mean, RCS_mean, ...
                theta0, latitudeMean, ori_longitude, ori_latitude, maxCarLen, ...
                kAti, bAti, RadarDataID);
            tracer_pointer = tracer_pointer + 1;
            tracer_buffer(tracer_pointer, 1) = data_idx;
            tracer_buffer(tracer_pointer, 2) = RadarDataID; 
            tracer_buffer(tracer_pointer, 3) = 0;
            tracer_buffer(tracer_pointer, 4) = 1;
            tracer_Pbuffer(tracer_pointer * 2 - 1 : tracer_pointer * 2, 1 : 2) = P_posterior;
        end
        j = jStart + 1;
    end
end

removeFlag = zeros(1, data_idx);
for i = 1 : data_idx
    if carID_buffer(all_res(i, 2) + 1) <= 1
        removeFlag(i) = 1;
    end
end

final_data = all_res(1 : data_idx, 1:14);
final_data = final_data(removeFlag == 0, :);
final_data2 = all_res(1 : data_idx, :);
final_data2 = final_data2(removeFlag == 0, :);
% writematrix(final_data, 'result.csv')