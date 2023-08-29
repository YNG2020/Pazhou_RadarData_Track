% 获取1~n范围内的num个随机数
function [random_vec] = GetRansacRandomNum(n, num)

    random_vec = zeros(num, 1);
    random_vec(1) = randi(n);
    i = 2;
    while 1
        status = 1;
        r = randi(n);
           
        j = 1;
        while (j < i)
            if random_vec(j) == r
                status = 0;
                break;
            end
            j = j + 1;
        end

        if status == 1
            random_vec(i) = r;
            i = i + 1;
        end

        if i == num + 1
            break;
        end

    end

end
