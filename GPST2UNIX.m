function [UNIX_time] = GPST2UNIX(weeks, second_in_weeks)
    UNIX_TO_GPST = 315964800;
    UNIX_time = UNIX_TO_GPST + 24 * 60 * 60 * 7 * weeks + second_in_weeks - 18;
end
