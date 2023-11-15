function [LaneRadarTrack] = mapLane2Radar(RadarData, Lane, dirLaneFlag)
    global A Q R H P P_posterior
    % 卡尔曼滤波器
    A = [1 1; 0 1];    % 状态转移矩阵，上一时刻的状态转移到当前时刻
    Q = [0.5 0; 0 0.01];   % 过程噪声协方差矩阵Q，p(w)~N(0,Q)，噪声来自真实世界中的不确定性
    R = [4 0; 0 0.04];  % 观测噪声协方差矩阵R，p(v)~N(0,R)
    H = [1 0; 0 1]; % 状态观测矩阵
    P = R;
    P_posterior = P;

    n_Gap = length(unique(RadarData(:,1)));  % 预先算出帧数，以预先进行内存分配
    radarFrameTime = zeros(n_Gap, 1); % 雷达数据中不同帧的时间
    radarFrameTimeIdx = zeros(n_Gap, 1);    % 雷达数据中不同帧的时间对应的第一个下标
    radarFrameCnt = zeros(n_Gap, 1);    % 雷达数据中不同帧中点云的数量
    radarFrameTime(1) = RadarData(1, 1);
    radarFrameTimeIdx(1) = 1;
    cnt = 2;
    
    n_RadarData = length(RadarData(:, 1));
    for i = 2 : n_RadarData
        if RadarData(i, 1) == radarFrameTime(cnt - 1)
            continue;
        end
        radarFrameTime(cnt) = RadarData(i, 1);
        radarFrameTimeIdx(cnt) = i;
        cnt = cnt + 1;
        if cnt > n_Gap
            break;
        end
    end
    radarFrameCnt(1:n_Gap-1) = diff(radarFrameTimeIdx);
    radarFrameCnt(n_Gap) = n_RadarData - radarFrameTimeIdx(n_Gap) + 1;
    
    n_Lane = size(Lane, 1);
    Lane2RadarFrame = zeros(n_Lane, 1);   % 标定数据的时刻到雷达数据的帧的第一个下标的映射
    Lane2FrameIdx = zeros(n_Lane, 1); % 标定数据的时刻到雷达数据的帧的下标的映射
    Lane2FrameErr = zeros(n_Lane, 1); % 标定数据的时刻到雷达数据的帧的下标的映射的误差
    weeks = Lane(1, 1);   % 数据中统一的GPST周数
    startGapIdx = 1;

    for i = 1 : n_Lane
        second_in_weeks = Lane(i, 2);
        UNIX_time = GPST2UNIX(weeks, second_in_weeks);
        while startGapIdx < n_Gap
            if UNIX_time >= radarFrameTime(startGapIdx) && UNIX_time <= radarFrameTime(startGapIdx + 1) % 闭区间，以覆盖整个区域
                if abs(radarFrameTime(startGapIdx) - UNIX_time) < abs(radarFrameTime(startGapIdx + 1) - UNIX_time)
                    Lane2RadarFrame(i) = radarFrameTimeIdx(startGapIdx);
                    Lane2FrameIdx(i) = startGapIdx;
                    Lane2FrameErr(i) = abs(radarFrameTime(startGapIdx) - UNIX_time);
                else
                    Lane2RadarFrame(i) = radarFrameTimeIdx(startGapIdx + 1);
                    Lane2FrameIdx(i) = startGapIdx + 1;
                    Lane2FrameErr(i) = abs(radarFrameTime(startGapIdx + 1) - UNIX_time);
                end
                break;
            end
            % 当前闭区间没能对应上标定数据的时间戳，则转向下一闭区间
            startGapIdx  = startGapIdx + 1;
        end
    end
    n_map = length(unique(Lane2FrameIdx));  % 最终生成映射的数量
    LaneIdx2Map = zeros(1, n_map);      % 存储将要进行映射的Lane数据的下标
    lastFrameIdx = Lane2FrameIdx(1);
    i = 1;
    cnt = 1;
    while 1 % 如果有多个标定数据的时间对应于同一个雷达数据的时间戳，那么取时间差最小的
        min_error = 10000;
        min_err_idx = i;
        while i <= n_Lane && Lane2FrameIdx(i) == lastFrameIdx
            if Lane2FrameErr(i) < min_error
                min_error = Lane2FrameErr(i);
                min_err_idx = i;
            end
            i = i + 1;
        end
        LaneIdx2Map(cnt) = min_err_idx;
        cnt = cnt + 1;
        if i <= n_Lane
            lastFrameIdx = Lane2FrameIdx(i);
        else
            break;
        end

    end
    
    Lane_sp = dirLaneFlag * sqrt(Lane(:, 6).^2 + Lane(:, 7).^2);
    LaneRadarTrack = nan(n_map, 15);
    tmp_predict_x = zeros(n_map, 1);
    tmp_predict_y = zeros(n_map, 1);
    all_radar_x = zeros(n_map, 7);  % 记录针对同一物体的所有雷达数据（纵向距离x）
    all_radar_y = zeros(n_map, 7);  % 记录针对同一物体的所有雷达数据（横向距离y）
    LastOKIDX = 1;
    lastLastOKIdx = 1;
    cnt = 1;

    for i = LaneIdx2Map

        n_Frame = radarFrameCnt(Lane2FrameIdx(i));
        sp_gap = Lane_sp(i) - RadarData(radarFrameTimeIdx(Lane2FrameIdx(i)) : radarFrameTimeIdx(Lane2FrameIdx(i)) + n_Frame - 1, 5);
        [sort_sp_gap, idx] = sort(abs(sp_gap), 'ascend');

        Frame = RadarData(radarFrameTimeIdx(Lane2FrameIdx(i)) : radarFrameTimeIdx(Lane2FrameIdx(i)) + n_Frame - 1, :);
        [curFrameData, sortedIdx] = sortrows(Frame, [5 3 4 6 1]); % 挑出合理的雷达数据点，同时先后对速度，纵向距离，横向距离，RCS升序，时间排序
        
        OKFLAG = 0;

        %%%%%%%%%%% 第一检验条件：如果能找到速度相近，且距离上一个OK的点的距离小于5m的两个点，则视作成功找到对应点
        for j = 1 : n_Frame
            if sort_sp_gap(j) > 1   % 速度差距过大，提前结束这一检验
                break;
            end
            if j < n_Frame
                if abs(sort_sp_gap(j) - sort_sp_gap(j+1)) < 0.01
                    tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(j) - 1;
                    [predict_x, predict_y] = predict_lot(LastOKIDX, tmpDotIdx, RadarData, LaneRadarTrack, lastLastOKIdx);

                    % if cnt == 1 || (RadarData(tmpDotIdx, 3) - LaneRadarTrack(LastOKIDX, 2))^2 + (RadarData(tmpDotIdx, 4) - LaneRadarTrack(LastOKIDX, 3))^2 < 25
                    if cnt == 1 || (RadarData(tmpDotIdx, 3) - predict_x)^2 + (RadarData(tmpDotIdx, 4) - predict_y)^2 < 25 && RadarData(tmpDotIdx, 6) > 0
                        OKFLAG = 1;
                        DotIdx = tmpDotIdx;
                        % 该部分用于检测径向速度相近且位置接近的雷达数据
                        [all_radar_x(cnt, :), all_radar_y(cnt, :), sp_mean] = find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, LaneRadarTrack, LastOKIDX);
                        lastLastOKIdx = LastOKIDX;
                        LastOKIDX = cnt;
                        break;
                    end
                end
            end
        end

        %%%%%%%%%% 第二检验条件：如果能找距离上一个OK的点的距离小于5m或7m的点，则视作成功找到对应点
        if cnt == 1 && ~OKFLAG
            DotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(1) - 1;
            [predict_x, predict_y] = predict_lot(LastOKIDX, DotIdx, RadarData, LaneRadarTrack, lastLastOKIdx);
            % 该部分用于检测径向速度相近且位置接近的雷达数据
            [all_radar_x(cnt, :), all_radar_y(cnt, :), sp_mean] = find_relate_data(i, 1, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, LaneRadarTrack, LastOKIDX);
            LastOKIDX = cnt;
            OKFLAG = 1;
        elseif ~OKFLAG        % 如果下一个点与上一个点的距离小于5m，则接受这个点
            for limit = [25]
                if OKFLAG
                    break;
                else
                    for j = 1 : n_Frame
                        tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(j) - 1;
                        [predict_x, predict_y] = predict_lot(LastOKIDX, tmpDotIdx, RadarData, LaneRadarTrack, lastLastOKIdx);

                        % if (RadarData(tmpDotIdx, 3) - LaneRadarTrack(LastOKIDX, 2))^2 + (RadarData(tmpDotIdx, 4) - LaneRadarTrack(LastOKIDX, 3))^2 < limit
                        if (RadarData(tmpDotIdx, 3) - predict_x)^2 + (RadarData(tmpDotIdx, 4) - predict_y)^2 < limit && RadarData(tmpDotIdx, 6) > 0
                            OKFLAG = 1;
                            DotIdx = tmpDotIdx;
                            % 该部分用于检测径向速度相近且位置接近的雷达数据
                            [all_radar_x(cnt, :), all_radar_y(cnt, :), sp_mean] = find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, LaneRadarTrack, LastOKIDX);
                            break;
                        end
                    end
                end
            end
        end

        %%%%%%%%%% 第三检验条件：如果一直找不到下一个点，则直接取路径前进方向上的，距离上一个OK的点距离最小的点
        if OKFLAG == 0
            min_dist = 1000000;
            min_idx = 1;
            for j = 1 : n_Frame
                tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + j - 1;
                [predict_x, predict_y] = predict_lot(LastOKIDX, tmpDotIdx, RadarData, LaneRadarTrack, lastLastOKIdx);

                % dist = (LaneRadarTrack(LastOKIDX, 2) - RadarData(tmpDotIdx, 3)) ^ 2 + (LaneRadarTrack(LastOKIDX, 3) - RadarData(tmpDotIdx, 4)) ^ 2;
                dist = (predict_x - RadarData(tmpDotIdx, 3)) ^ 2 + (predict_y - RadarData(tmpDotIdx, 4)) ^ 2;
                if dist < min_dist
                    if RadarData(tmpDotIdx, 3) - LaneRadarTrack(LastOKIDX, 2) > 0
                        min_dist = dist;
                        min_idx = j;
                    end
                end
            end
            DotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + min_idx - 1;
            % 该部分用于检测径向速度相近且位置接近的雷达数据
            [all_radar_x(cnt, :), all_radar_y(cnt, :), sp_mean] = find_relate_data(i, min_idx, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, LaneRadarTrack, LastOKIDX);
            OKFLAG = 1;
        end
        if OKFLAG == 0
            cnt = cnt + 1;
            continue;
        end

        
        tmp_predict_x(cnt) = predict_x;
        tmp_predict_y(cnt) = predict_y;
        
        LaneRadarTrack(cnt, 1) = DotIdx;       % 写入点云序号
        LaneRadarTrack(cnt, 2) = all_radar_x(cnt, 7); % 写入距离雷达的纵向距离
        LaneRadarTrack(cnt, 3) = all_radar_y(cnt, 7); % 写入距离雷达的横向距离
        LaneRadarTrack(cnt, 4) = Lane(i, 3);    % 写入纬度
        LaneRadarTrack(cnt, 5) = Lane(i, 4);    % 写入经度
        LaneRadarTrack(cnt, 6) = Lane(i, 6);    % 写入北向速度
        LaneRadarTrack(cnt, 7) = Lane(i, 7);    % 写入东向速度
        LaneRadarTrack(cnt, 8) = Lane_sp(i);    % 写入速度
        LaneRadarTrack(cnt, 13) = v_true_cal(all_radar_x(cnt, 7), all_radar_y(cnt, 7), 7, RadarData(DotIdx, 5), 1); % 写入真实速度;
        LaneRadarTrack(cnt, 9) = sp_mean; % 写入径向速度;
        LaneRadarTrack(cnt, 10) = RadarData(radarFrameTimeIdx(Lane2FrameIdx(i)), 1);    % 写入时间
        LaneRadarTrack(cnt, 11) = RadarData(DotIdx, 6);    % 写入RCS
        LaneRadarTrack(cnt, 12) = LaneRadarTrack(cnt, 9) - LaneRadarTrack(cnt, 8);    % 写入速度差1
        LaneRadarTrack(cnt, 14) = LaneRadarTrack(cnt, 13) - LaneRadarTrack(cnt, 8);    % 写入速度差2
        LaneRadarTrack(cnt, 15) = Lane(i, 5);   % 写入海拔
        cnt = cnt + 1;
    end
end

function [radar_x, radar_y, sp_mean] = find_relate_data(i, j, RadarData, radarFrameTimeIdx, Lane2FrameIdx, DotIdx, n_Frame, idx, LaneRadarTrack, LastOKIDX)
    global A Q R H P P_posterior
    % 该部分用于检测径向速度相近且位置接近的雷达数据
    radar_x = zeros(1, 7);
    radar_y = zeros(1, 7);
    sp_mean = RadarData(DotIdx, 5);
    speedSum = RadarData(DotIdx, 5);
    radar_x(1) = RadarData(DotIdx, 3);
    radar_y(1) = RadarData(DotIdx, 4);
    tmp_cnt = 1;
    radar_x(7) = sum(radar_x(1 : tmp_cnt)) / tmp_cnt;
    radar_y(7) = sum(radar_y(1 : tmp_cnt)) / tmp_cnt;
    j = j + 1;
    sp_last = RadarData(DotIdx, 5);
    if j < n_Frame
        tmpDotIdx1 = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(j) - 1;
    end
    tmp_cnt = tmp_cnt + 1;
    while (j < n_Frame && abs(sp_last - RadarData(tmpDotIdx1, 5)) < 0.001)

        if (RadarData(tmpDotIdx1, 3) - radar_x(7))^2 + (RadarData(tmpDotIdx1, 4) - radar_y(7))^2 < 25
            
            speedSum = speedSum + RadarData(tmpDotIdx1, 5);
            radar_x(tmp_cnt) = RadarData(tmpDotIdx1, 3);
            radar_y(tmp_cnt) = RadarData(tmpDotIdx1, 4);
            radar_x(7) = sum(radar_x(1 : tmp_cnt)) / tmp_cnt;
            radar_y(7) = sum(radar_y(1 : tmp_cnt)) / tmp_cnt;
            tmp_cnt = tmp_cnt + 1;
            if tmp_cnt > 5
                break;
            end
        end
        j = j + 1;
        tmpDotIdx1 = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(j) - 1;
    end

    if LastOKIDX > 1
        sp_mean = speedSum / (tmp_cnt - 1);
        % ----------------------进行先验估计---------------------
        deltaT = RadarData(DotIdx, 1) - LaneRadarTrack(LastOKIDX, 10);
        A = [1 deltaT; 0 1];
        carDisLog = LaneRadarTrack(LastOKIDX, 2);
        carSpeed = LaneRadarTrack(LastOKIDX, 9);
        X_prior = A * [carDisLog; carSpeed];
        % 计算状态估计协方差矩阵P
        P_prior = A * P_posterior * A' + Q;
        % ----------------------计算卡尔曼增益
        R = [9 0; 0 0];  % 观测噪声协方差矩阵R，p(v)~N(0,R)
        K = P_prior * H' * inv(H * P_prior * H' + R);
        % ---------------------后验估计------------
        Z_measure = zeros(2, 1);
        Z_measure(1) = radar_x(7);
        Z_measure(2) = sp_mean;
        X_posterior = X_prior + K * (Z_measure - H * X_prior);
        X_mean = X_posterior(1);
        sp_mean = X_posterior(2);
        % 更新状态估计协方差矩阵P
        P_posterior = (eye(2, 2) - K * H) * P_prior;

        radar_x(7) = X_mean;
    end
    
    radar_x(6) = tmp_cnt - 1;
    radar_y(6) = tmp_cnt - 1;
end

function [predict_x, predict_y] = predict_lot(LastOKIDX, tmpDotIdx, RadarData, LaneRadarTrack, lastLastOKIdx)
    % 预测车辆在下一帧出现的位置，将历史速度与当前速度做一个权衡，用权衡后的速度乘以经过的时间，加上原本的位置，即得到预测位置
    if lastLastOKIdx == LastOKIDX
        predict_x = RadarData(tmpDotIdx, 3);
        predict_y = RadarData(tmpDotIdx, 4);
    else
        delta_t1 = RadarData(tmpDotIdx, 1) - LaneRadarTrack(LastOKIDX, 10);     % 当前时间与上一可靠轨迹点的时间之差
        delta_t2 = LaneRadarTrack(LastOKIDX, 10) - LaneRadarTrack(1, 10);   % 上一可靠轨迹点与第一个可靠轨迹点时间之差
        delta_t3 = LaneRadarTrack(LastOKIDX, 10) - LaneRadarTrack(lastLastOKIdx, 10);   % 上一可靠轨迹点与前前可靠轨迹点时间之差

        delta_x_his = (LaneRadarTrack(LastOKIDX, 2) - LaneRadarTrack(1, 2)) / delta_t2 * delta_t1;
        delta_y_his = (LaneRadarTrack(LastOKIDX, 3) - LaneRadarTrack(1, 3)) / delta_t2 * delta_t1;
        delta_x_cur = (LaneRadarTrack(LastOKIDX, 2) - LaneRadarTrack(lastLastOKIdx, 2)) / delta_t3 * delta_t1;
        delta_y_cur = (LaneRadarTrack(LastOKIDX, 3) - LaneRadarTrack(lastLastOKIdx, 3)) / delta_t3 * delta_t1;

        % delta_x_his = (tmp_predict_x(LastOKIDX) - tmp_predict_x(1)) / delta_t2 * delta_t1;
        % delta_y_his = (tmp_predict_y(LastOKIDX) - tmp_predict_y(1)) / delta_t2 * delta_t1;
        % delta_x_cur = (tmp_predict_x(LastOKIDX) - tmp_predict_x(lastLastOKIdx)) / delta_t3 * delta_t1;
        % delta_y_cur = (tmp_predict_y(LastOKIDX) - tmp_predict_y(lastLastOKIdx)) / delta_t3 * delta_t1;

        p = 1;
        delta_x = delta_x_his * p + delta_x_cur * (1 - p);
        delta_y = delta_y_his * p + delta_y_cur * (1 - p);
        v = v_true_cal(LaneRadarTrack(LastOKIDX, 2), LaneRadarTrack(LastOKIDX, 3), 7, LaneRadarTrack(LastOKIDX, 9), 1);
        delta_x = delta_t1 * v;

        predict_x = delta_x + LaneRadarTrack(LastOKIDX, 2);
        predict_y = delta_y + LaneRadarTrack(LastOKIDX, 3);

        % predict_x = delta_x + tmp_predict_x(LastOKIDX);
        % predict_y = delta_y + tmp_predict_y(LastOKIDX);

    end
end

function [UNIX_time] = GPST2UNIX(weeks, second_in_weeks)
    UNIX_TO_GPST = 315964800;
    UNIX_time = UNIX_TO_GPST + 24 * 60 * 60 * 7 * weeks + second_in_weeks - 18;
end
