% 通过车辆当前的x，y，z坐标，以及径向速度，计算其实际速度（统一默认车辆沿着车道行驶，在误差可接受的范围内，同时默认车道方向就是x轴正向方向，此时alpha=1）
function [v] = v_true_cal(x, y, z, v_r, alpha)
    cosTheta = abs(x) / sqrt(x.^2 + y.^2 + z.^2);  % 在雷达数据里，x必然大于0
    v = v_r / cosTheta;
    v = v / alpha;
end