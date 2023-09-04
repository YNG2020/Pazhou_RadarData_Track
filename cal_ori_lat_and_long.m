% 通过标定数据，确定雷达坐标系的原点的经纬度
function [ori_longitude, ori_latitude] = cal_ori_lat_and_long(theta0, LaneRadarTrack1, LaneRadarTrack2, LaneRadarTrack3)
    R = 6371393;   % 地球平均半径
    latitudeMean = mean([LaneRadarTrack1(:, 4); LaneRadarTrack2(:, 4); LaneRadarTrack3(:, 4)]); % 平均纬度
    longitude_gap_per_meter = 360 / (2 * pi * R * cos(latitudeMean / 180 * pi));    % 东西方向的一米在经度上跨越的度数
    latitude_gap_per_meter = 360 / (2 * pi * R);    % 南北方向的一米在纬度上跨域的度数
    
    n_Lane1 = size(LaneRadarTrack1, 1);
    n_Lane2 = size(LaneRadarTrack2, 1);
    n_Lane3 = size(LaneRadarTrack3, 1);
    ori_coordinate1 = zeros(n_Lane1, 2);
    ori_coordinate2 = zeros(n_Lane2, 2);
    ori_coordinate3 = zeros(n_Lane3, 2);
    for i = 1 : n_Lane1
        westDeg = longitude_gap_per_meter * (LaneRadarTrack1(i, 2) * cos(theta0) - LaneRadarTrack1(i, 3) * sin(theta0));
        southDeg = latitude_gap_per_meter * (LaneRadarTrack1(i, 2) * sin(theta0) + LaneRadarTrack1(i, 3) * cos(theta0));
        ori_coordinate1(i, 1) = LaneRadarTrack1(i, 5) + westDeg;
        ori_coordinate1(i, 2) = LaneRadarTrack1(i, 4) + southDeg;
    end
    for i = 1 : n_Lane2
        westDeg = longitude_gap_per_meter * (LaneRadarTrack2(i, 2) * cos(theta0) - LaneRadarTrack2(i, 3) * sin(theta0));
        southDeg = latitude_gap_per_meter * (LaneRadarTrack2(i, 2) * sin(theta0) + LaneRadarTrack2(i, 3) * cos(theta0));
        ori_coordinate2(i, 1) = LaneRadarTrack2(i, 5) + westDeg;
        ori_coordinate2(i, 2) = LaneRadarTrack2(i, 4) + southDeg;
    end
    for i = 1 : n_Lane3
        westDeg = longitude_gap_per_meter * (LaneRadarTrack3(i, 2) * cos(theta0) - LaneRadarTrack3(i, 3) * sin(theta0));
        southDeg = latitude_gap_per_meter * (LaneRadarTrack3(i, 2) * sin(theta0) + LaneRadarTrack3(i, 3) * cos(theta0));
        ori_coordinate3(i, 1) = LaneRadarTrack3(i, 5) + westDeg;
        ori_coordinate3(i, 2) = LaneRadarTrack3(i, 4) + southDeg;
    end
    ori_longitude = mean([ori_coordinate1(:, 1); ori_coordinate2(:, 1); ori_coordinate3(:, 1)]);
    ori_latitude = mean([ori_coordinate1(:, 2); ori_coordinate2(:, 2); ori_coordinate3(:, 2)]);
end
