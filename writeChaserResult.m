% 写入正在追踪的目标的数据
function res = writeChaserResult(nowT, carID, X_mean, Y_mean, ...
    RadarHeight, sp_mean, cosTheta2, sinTheta2, carLat, RCS_mean, ...
    carClass, theta0, latitudeMean, ori_longitude, ori_latitude, ...
    kAti, bAti)
    res = zeros(1, 14);
    res(1) = nowT;
    res(2) = carID;
    res(3) = X_mean;
    res(4) = Y_mean;
    sp_true = v_true_cal(X_mean, Y_mean, RadarHeight, sp_mean, cosTheta2);
    res(5) = sp_true * cosTheta2;
    if Y_mean > carLat
        res(6) = abs(sp_true * sinTheta2);
    else
        res(6) = -abs(sp_true * sinTheta2);
    end
    res(7) = RCS_mean;
    res(8) = carClass;
    [longitude, latitude] = getCoordinate(X_mean, Y_mean, theta0, ...
        latitudeMean, ori_longitude, ori_latitude);
    res(9) = longitude;
    res(10) = latitude;
    res(11) = kAti * X_mean + bAti;
    if sp_true == 0
        res(12) = 1;
    else
        res(12) = 0;
    end
    if sp_true < 0
        res(13) = 1;
    else
        res(13) = 0;
    end
    if sp_true > 16.6667
        res(14) = 1;
    else
        res(14) = 0;
    end
end
