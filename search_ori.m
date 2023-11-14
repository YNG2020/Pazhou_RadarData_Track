searchNum = 1000;
disErrTracer = zeros(searchNum, 1);
longitude_gap_per_meter = 360 / (2 * pi * R * cos(latitudeMean / 180 * pi));    % 东西方向的一米在经度上跨越的度数
latitude_gap_per_meter = 360 / (2 * pi * R);    % 南北方向的一米在纬度上跨域的度数
granularity = 0.05;
long_step = granularity * longitude_gap_per_meter;
lat_step = granularity * latitude_gap_per_meter;

oriLong = 113.525249970317049;
oriLat = 23.263544188509108;
curLong = oriLong;
curLat = oriLat;


for i = 1 : searchNum
    delta = 0 - i / 1000000;
    theta0 = 0.505648618725381 + delta;
    mean_disErr = get_ori_err(LaneRadarTrack1_choose, LaneRadarTrack3_choose, theta0, latitudeMean, ori_longitude, ori_latitude);
    disErrTracer(i) = mean_disErr;
end

oriA_longitude = 113.525249970317049;
oriA_latitude = 23.263544188509108;
mean_disErr_backup2 = disErrTracer;

i = 103;
j = 45;

ori_longitude = (i - 1) * long_step + oriLong;
ori_latitude = (j - 1) * lat_step + oriLat;