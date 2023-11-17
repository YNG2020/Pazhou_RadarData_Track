% 写入正在追踪的目标的数据
function res = writeResult(nowT, carID, X_mean, Y_mean, ...
    RadarHeight, sp_mean, cosTheta2, sinTheta2, carLat, RCS_mean, ...
    theta0, latitudeMean, ori_longitude, ori_latitude, maxCarlen,...
    kAti, bAti, RadarDataID, dirLaneFlag)
    res = zeros(1, 16);
    res(1) = nowT;
    res(2) = carID;
    res(3) = X_mean;
    res(4) = Y_mean;
    sp_true = v_true_cal(X_mean, Y_mean, RadarHeight, sp_mean, cosTheta2);
    res(5) = sp_true * cosTheta2;
    if (abs(Y_mean - carLat) < 0.0001)
        res(6) = 0;
    elseif Y_mean > carLat
        res(6) = abs(sp_true * sinTheta2);
    else
        res(6) = -abs(sp_true * sinTheta2);
    end
    res(7) = RCS_mean;
    [longitude, latitude] = getCoordinate(X_mean, Y_mean, theta0, ...
        latitudeMean, ori_longitude, ori_latitude, dirLaneFlag);
    res(9) = longitude;
    res(10) = latitude;
    res(11) = kAti * X_mean + bAti;
    if abs(sp_true) == 0
        res(12) = 1;
    else
        res(12) = 0;
    end
    if dirLaneFlag * sp_true < 0
        res(13) = 1;
    else
        res(13) = 0;
    end
    if abs(sp_true) > 16.6667
        res(14) = 1;
    else
        res(14) = 0;
    end
    if maxCarlen < 7.5
        carClass = 0;
    else
        carClass = 1;
    end
    res(8) = carClass;
    res(15) = RadarDataID;
    res(16) = maxCarlen;
end