function [LaneRadarTrack] = mapLane2Radar(RadarData, Lane)
    n_Gap = 15538;  % 15538是预先用matlab算出来的帧数，以预先进行内存分配
    radarFrameTime = zeros(n_Gap, 1); % 雷达数据中不同帧的时间
    radarFrameTimeIdx = zeros(n_Gap, 1);    % 雷达数据中不同帧的时间对应的第一个下标
    radarFrameCnt = zeros(n_Gap, 1);    % 雷达数据中不同帧中点云的数量
    radarFrameTime(1) = RadarData(1, 1);
    radarFrameTimeIdx(1) = 1;
    cnt = 2;
    
    n_RadarData = length(RadarData(:, 1));
    for i = 2 : n_RadarData
        if RadarData(i, 2) > radarFrameTimeIdx(cnt)
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
    Lane2RadarFrame = zeros(n_Lane, 1);   % 标定数据1的时刻到雷达数据的帧的第一个下标的映射
    Lane2FrameIdx = zeros(n_Lane, 1); % 标定数据1的时刻到雷达数据的帧的下标的映射
    weeks = 2269;   % 数据中统一的GPST周数
    startGapIdx = 1;
    for i = 1 : n_Lane
        second_in_weeks = Lane(i, 2);
        UNIX_time = GPST2UNIX(weeks, second_in_weeks);
        while startGapIdx < n_Gap
            if UNIX_time >= radarFrameTime(startGapIdx) && UNIX_time <= radarFrameTime(startGapIdx + 1) % 闭区间，以覆盖整个区域
                if abs(radarFrameTime(startGapIdx) - UNIX_time) < abs(radarFrameTime(startGapIdx + 1) - UNIX_time)
                    Lane2RadarFrame(i) = radarFrameTimeIdx(startGapIdx);
                    Lane2FrameIdx(i) = startGapIdx;
                else
                    Lane2RadarFrame(i) = radarFrameTimeIdx(startGapIdx + 1);
                    Lane2FrameIdx(i) = startGapIdx + 1;
                end
                break;
            end
            startGapIdx  = startGapIdx + 1;
        end
    end
    
    Lane_sp = sqrt(Lane(:, 6).^2 + Lane(:, 7).^2);
    LaneRadarTrack = nan(n_Lane, 12);
    LastOKIDX = 1;
    for i = 1 : n_Lane
        n_Frame = radarFrameCnt(Lane2FrameIdx(i));
        sp_gap = abs(Lane_sp(i) - RadarData(radarFrameTimeIdx(Lane2FrameIdx(i)) : radarFrameTimeIdx(Lane2FrameIdx(i)) + n_Frame - 1, 5));
        [~, idx] = sort(sp_gap, 'ascend');
        OKFLAG = 0;
        if i == 1   % 第一个点暂时不核验，这涉及到车辆标识的算法
            tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(1) - 1;
            OKFLAG = 1;
        else        % 如果下一个点与上一个点的距离小于5m，则接受这个点
            for j = 1 : n_Frame
                tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(j) - 1;
                if (RadarData(tmpDotIdx, 3) - LaneRadarTrack(LastOKIDX, 2))^2 + (RadarData(tmpDotIdx, 4) - LaneRadarTrack(LastOKIDX, 3))^2 < 25
                    OKFLAG = 1;
                    LastOKIDX = i;
                    break;
                end
            end
        end
        for limit = [50 100 150 200]     % 如果下一个点与上一个点的距离依次小于[50 100 150 200]m，则依次接受这个点
            if OKFLAG
                DotIdx = tmpDotIdx;
                break;
            else
                for j = 1 : n_Frame
                    tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + idx(j) - 1;
                    if (RadarData(tmpDotIdx, 3) - LaneRadarTrack(LastOKIDX, 2))^2 + (RadarData(tmpDotIdx, 4) - LaneRadarTrack(LastOKIDX, 3))^2 < limit
                        OKFLAG = 1;
                        if limit <= 50
                            LastOKIDX = i;
                        end
                        break;
                    end
                end
            end
        end
        if OKFLAG == 0      % 如果一直找不到下一个点，则直接再前面的，距离上一个点距离最小的点
            min_dist = 1000000;
            min_idx = 1;
            for j = 1 : n_Frame
                tmpDotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + j - 1;
                dist = (LaneRadarTrack(LastOKIDX, 2) - RadarData(tmpDotIdx, 3)) ^ 2 + (LaneRadarTrack(LastOKIDX, 3) - RadarData(tmpDotIdx, 4)) ^ 2;
                if dist < min_dist
                    if RadarData(tmpDotIdx, 3) - LaneRadarTrack(LastOKIDX, 2) > 0
                        min_dist = dist;
                        min_idx = j;
                    end
                end
            end
            DotIdx = radarFrameTimeIdx(Lane2FrameIdx(i)) + min_idx - 1;
        end
        
        LaneRadarTrack(i, 1) = DotIdx;       % 写入点云序号
        LaneRadarTrack(i, 2) = RadarData(DotIdx, 3); % 写入距离雷达的纵向距离
        LaneRadarTrack(i, 3) = RadarData(DotIdx, 4); % 写入距离雷达的横向距离
        LaneRadarTrack(i, 4) = Lane(i, 3);    % 写入纬度
        LaneRadarTrack(i, 5) = Lane(i, 4);    % 写入经度
        LaneRadarTrack(i, 6) = Lane(i, 6);    % 写入横向速度
        LaneRadarTrack(i, 7) = Lane(i, 7);    % 写入纵向速度
        LaneRadarTrack(i, 8) = Lane_sp(i);    % 写入速度
        LaneRadarTrack(i, 9) = RadarData(DotIdx, 5); % 写入径向速度;
        LaneRadarTrack(i, 10) = RadarData(radarFrameTimeIdx(Lane2FrameIdx(i)), 1);    % 写入时间
        LaneRadarTrack(i, 11) = RadarData(DotIdx, 6);    % 写入RCS
        LaneRadarTrack(i, 12) = LaneRadarTrack(i, 9) - LaneRadarTrack(i, 8);    % 写入速度差
    end
end