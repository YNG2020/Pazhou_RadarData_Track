% 求出两条界限直线的截距
function [b_left, b_right] = get_intercept(k, b1, b3)
    % b = b3;
    % lineGap1 = 1.8 + 0;     % 假设第一车道的中央与左侧路基距离
    % lineGap2 = 12.6 + 0;     % 假设第一车道的中央与右侧路基距离
    
    % b_left_tmp1 = b + lineGap1 * sqrt(1 + k^2);
    % b_left_tmp2 = b - lineGap1 * sqrt(1 + k^2);
    % if abs(b_left_tmp1 - b3) > abs(b_left_tmp2 - b3)
    %     b_left = b_left_tmp1;
    % else
    %     b_left = b_left_tmp2;
    % end
    % 
    % b_right_tmp1 = b + lineGap2 * sqrt(1 + k^2);
    % b_right_tmp2 = b - lineGap2 * sqrt(1 + k^2);
    % if abs(b_right_tmp1 - b1) > abs(b_right_tmp2 - b1)
    %     b_right = b_right_tmp1;
    % else
    %     b_right = b_right_tmp2;
    % end
    if b1 < 0
        b_left = b1 - 1.8;
        b_right = b3 + 1.8;
    else
        b_left = b1 + 1.8;
        b_right = b3 - 1.8;
    end

end