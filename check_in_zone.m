% 检查指定点是否位于区域内
function [INFLAG] = check_in_zone(k, b_left, b_right, x, y)
    if (b_left < b_right)
        b_tmp = b_left;
        b_left = b_right;
        b_right = b_tmp;
    end
    if k * x + b_left > y && k * x + b_right < y        % 在界内
        INFLAG = 1;
    else                % 在界外
        INFLAG = 0;
    end
end