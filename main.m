% 此部分尽量用C++风格写matlab
[k1, b1] = line_plofit(LaneRadarTrack1(:, 2), LaneRadarTrack1(:, 3));
[k2, b2] = line_plofit(LaneRadarTrack2(:, 2), LaneRadarTrack2(:, 3));
[k3, b3] = line_plofit(LaneRadarTrack3(:, 2), LaneRadarTrack3(:, 3));
k = (k1 + k2 + k3) / 3;
tan1 = LaneRadarTrack1(:, 6) ./ LaneRadarTrack1(:, 7);
tan2 = LaneRadarTrack2(:, 6) ./ LaneRadarTrack2(:, 7);
tan3 = LaneRadarTrack3(:, 6) ./ LaneRadarTrack3(:, 7);
arcTan1 = atan(tan1);
arcTan2 = atan(tan2);
arcTan3 = atan(tan3);
theta1 = mean([arcTan1; arcTan2; arcTan3]); % 经纬度与雷达坐标系之间的角度偏差
theta2 = atan(k);   % 车道与雷达坐标系之间的角度偏差
theta0 = theta1 - theta2;
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

carA = 5;   % 设置汽车在一般情况下的最大加速度为5m/s^2，包括正向与负向的
maxVarX = 10;   % 设置同一辆车的在前后帧的在车道上的最大纵向距离偏差
maxVarY = 2;    % 设置同一辆车的在前后帧的在车道上的最大横向距离偏差
maxVarRCS = 10; % 设置同一辆车的在前后帧的RCS偏差
RadarHeight = 7;    % 雷达高度

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

tracer_buffer = zeros(1000, 2); % 第一列记录在前一帧追踪的存放在all_res中的编号，第二列记录对应的在RadarData中的编号
tracer_pointer = 0; % tracer_pointer永远指向buffer中的最后一个有效元素，且其前面均为有效元素
data_idx = 0;
all_res = zeros(n_radar_data, 14);
carUniqueId = -1;
for cnt = 1 : n_Gap
    frameStart = frameGapIdx(cnt);  % 当前帧的在雷达数据的起始位
    nFrame = frameGapIdx(cnt + 1) - frameGapIdx(cnt);   % 该帧的帧数
    nowT = curFrameData(1, 1);  % 记录现在的时刻
    tmpIndex = zeros(nFrame, 1);
    for i = 1 : nFrame
        if check_in_zone(k, b_left, b_right, RadarData(frameStart + i - 1, 3), RadarData(frameStart + i - 1, 4)) && RadarData(frameStart + i - 1, 6) > 0  % 在区域内，且RCS > 0
            tmpIndex(i) = 1;        % 在C++中，寻找OKIndex应该用push来代替
        end
    end
    OKIndex = find(tmpIndex);
    OKIndexPointer_len = length(OKIndex);
    BlockIndex = zeros(OKIndexPointer_len, 1);
    [curFrameData, ~] = sortrows(RadarData(frameStart + OKIndex - 1, :), [5 3 4]); % 挑出合理的雷达数据点，同时先后对速度，纵向距离，横向距离升序排序
    
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
        coupleFlag = 0; j = 1;
        while j <= OKIndexPointer_len  % 在OKIndex里寻找能与正在追踪的车辆匹配的点，找到之后，把它从OKIndex中删除
            if BlockIndex(j)
                j = j + 1; continue;
            end
            if abs(carSpeed - curFrameData(j, 5)) > deltaT * carA %(deltaT在0.025~0.2之间)
                j = j + 1; continue;
            end
            if abs(carDisLat - curFrameData(j, 4)) * cosTheta2 > maxVarY
                j = j + 1; continue;
            end
            if abs(carRCS - curFrameData(j, 6)) > maxVarRCS
                j = j + 1; continue;
            end
            v_true = v_true_cal(carDisLog, carDisLat, RadarHeight, carSpeed, cosTheta2);   % 计算车辆的实际速度，默认车辆沿着车道方向行驶
            if abs(curFrameData(j, 3) - (carDisLog + deltaT * v_true * cosTheta2)) * cosTheta2 > maxVarX
                j = j + 1; continue;
            end

            % 匹配成功
            coupleFlag = 1;
            X_mean = curFrameData(j, 3);  X_sum = X_mean;
            Y_mean = curFrameData(j, 4);  Y_sum = Y_mean;
            sp_mean = curFrameData(j, 5); sp_sum = sp_mean;
            RCS_mean = curFrameData(j, 6);RCS_sum = RCS_mean;
            BlockIndex(j) = 1;    % 匹配成功的数据点，将被拿走
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
                if abs(sp_mean - curFrameData(j, 5)) > 0.1
                    j = j + 1; continue;
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
            all_res(data_idx, :) = writeChaserResult(nowT, carID, X_mean, Y_mean, ...
                RadarHeight, sp_mean, cosTheta2, sinTheta2, carDisLat, RCS_mean, ...
                carClass, theta0, latitudeMean, ori_longitude, ori_latitude, ...
                kAti, bAti);            
            break;
        end
        if coupleFlag == 0
            tracer_buffer(i, 1) = tracer_buffer(tracer_pointer, 1);
            tracer_buffer(i, 2) = tracer_buffer(tracer_pointer, 2);
            tracer_pointer = tracer_pointer - 1;
        else
            i = i + 1;
        end
    end
    
    %%%%% 识别算法
    j = 1;
    while j <= OKIndexPointer_len - 1  % 最后一个点无须检验，因为在当前假设下，最后一点如果不与其它点，成为组合，那么单独一个雷达点不被判断为车辆
        if BlockIndex(j)
            j = j + 1; continue;
        end
        if curFrameData(j + 1, 5) - curFrameData(j, 5) > 0.1
            BlockIndex(j) = 1; % 如果这个点能与前面的点连接，那么它早该被block，同时，由于与下一个点的速度差已经超过了0.1，那么接下来的所有点都不能与之组合
            j = j + 1; continue;
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
            if abs(sp_mean - curFrameData(j, 5)) > 0.1
                j = j + 1; continue;
            end
            if curFrameData(j, 3) < Xmin
                Xmin = curFrameData(j, 3);
            end
            if curFrameData(j, 3) > Xmax
                Xmax = curFrameData(j, 3) < Xmin;
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
            carLen = (Xmax - Xmin) / cosTheta2 + 0.4;
            if carLen < 2.5
                carClass = 2;
            elseif carLen > 7.5
                carClass = 1;
            else
                carClass = 0;
            end
            all_res(data_idx, :) = writeChaserResult(nowT, carUniqueId, X_mean, Y_mean, ...
                RadarHeight, sp_mean, cosTheta2, sinTheta2, Y_mean, RCS_mean, ...
                carClass, theta0, latitudeMean, ori_longitude, ori_latitude, ...
                kAti, bAti);
            tracer_pointer = tracer_pointer + 1;
            tracer_buffer(tracer_pointer, 1) = data_idx;
            tracer_buffer(tracer_pointer, 2) = frameStart + OKIndex(jStart) - 1;    
        end
        j = jStart + 1;
    end
end
final_data = all_res(1 : data_idx, :);
writematrix(final_data, 'result.csv')