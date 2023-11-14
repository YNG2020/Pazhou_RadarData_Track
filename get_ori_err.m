function [mean_disErr] = get_ori_err(LaneRadarTrack1_choose, LaneRadarTrack3_choose, theta0, latitudeMean, ori_longitude, ori_latitude)

    [longitude1, latitude1] = getCoordinate(LaneRadarTrack1_choose(:, 2), LaneRadarTrack1_choose(:, 3), theta0, latitudeMean, ori_longitude, ori_latitude);
    [longitude3, latitude3] = getCoordinate(LaneRadarTrack3_choose(:, 2), LaneRadarTrack3_choose(:, 3), theta0, latitudeMean, ori_longitude, ori_latitude);
    
    R = 6371393;
    n_lane1 = length(longitude1);
    n_lane3 = length(longitude3);
    disErr1 = zeros(n_lane1, 1);
    disErr3 = zeros(n_lane3, 1);
    for i = 1 : n_lane1
        disErr1(i) = distance(LaneRadarTrack1_choose(i, 4), LaneRadarTrack1_choose(i, 5), latitude1(i), longitude1(i), R);
    end
    for i = 1 : n_lane3
        disErr3(i) = distance(LaneRadarTrack3_choose(i, 4), LaneRadarTrack3_choose(i, 5), latitude3(i), longitude3(i), R);
    end
    
    mean_disErr = mean([disErr1; disErr3]);

end
