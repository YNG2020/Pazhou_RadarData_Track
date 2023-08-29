function [longitude, latitude] = getCoordinate(distLong, distLati, theta0, latitudeMean, ori_longitude, ori_latitude)
    R = 6371393;   % 地球平均半径
    longitude_gap_per_meter = 360 / (2 * pi * R * cos(latitudeMean / 180 * pi));    % 东西方向的一米在经度上跨越的度数
    latitude_gap_per_meter = 360 / (2 * pi * R);    % 南北方向的一米在纬度上跨域的度数
    westDeg = longitude_gap_per_meter * (distLong * cos(theta0) - distLati * sin(theta0));
    southDeg = latitude_gap_per_meter * (distLong * sin(theta0) + distLati * cos(theta0));
    longitude = ori_longitude - westDeg;
    latitude = ori_latitude - southDeg;
end
