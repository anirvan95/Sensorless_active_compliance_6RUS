function [speed] = speed_to_dxl(value)
     
    if value < 0 
        direction = 1024;
    else 
        direction = 0;
    end
    speed_factor = 0.114;
    max_value = 1023*speed_factor*6
    value = min(max(value, -max_value), max_value)
    speed = (round(direction + abs(value)/(6*speed_factor), 0))
end

