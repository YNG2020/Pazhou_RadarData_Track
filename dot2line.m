function [dist] = dot2line(k, b, x, y)
    dist = abs(k*x - y + b) / sqrt(1+k^2);
end