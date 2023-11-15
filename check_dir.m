% 通过计算雷达数据中的合理数据的速度为正的点数和为负的点数，确定车道的来向和去向，同时返回横向位置的均值
function [dirFlag, meanY] = check_dir(RadarData)
    n_radar_data = size(RadarData, 1);
    posCnt = 0;
    negCnt = 0;
    totY = 0;
    for i = 1 : n_radar_data
        y = RadarData(i, 4);
        speed = RadarData(i, 5);
        RCS = RadarData(i, 6);
        if (check_in_zone(0, -12.8, 12.8, 0, y) && RCS > 10)    % 合理数据点
            if (speed < 0)
                negCnt = negCnt + 1;
                totY = totY + y;
            elseif (speed > 0)
                posCnt = posCnt + 1;
                totY = totY + y;
            end
        end
    end
    if posCnt > negCnt
        dirFlag = 1;
    else
        dirFlag = -1;
    end
    meanY = totY / (posCnt + negCnt);
end