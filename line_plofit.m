% 利用最小二乘法，拟合数据LaneRadarTrack_x, LaneRadarTrack1_y为y = kx + b的形式
function [k, b] = line_plofit(LaneRadarTrack_x, LaneRadarTrack_y)
    
    num_point = length(LaneRadarTrack_x);
    sum_x = 0; sum_y = 0; sum_x2 = 0; sum_xy = 0;
    for i = 1 : num_point
        sum_x = sum_x + LaneRadarTrack_x(i);
        sum_y = sum_y + LaneRadarTrack_y(i);
        sum_x2 = sum_x2 + LaneRadarTrack_x(i) * LaneRadarTrack_x(i);
        sum_xy = sum_xy + LaneRadarTrack_x(i) * LaneRadarTrack_y(i);
    end
    tmp = num_point * sum_x2 - sum_x * sum_x;
    if (abs(tmp) > 0.000000001)
        k = (num_point * sum_xy - sum_x * sum_y) / tmp;
        b = (sum_x2 * sum_y - sum_x * sum_xy) / tmp;
    else
        k = 0;
        b = 0;
    end
    
end
