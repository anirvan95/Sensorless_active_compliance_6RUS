function [pos] = dxl_to_degree(value)
    max_pos = 4096;
    max_deg = 360;
    pos = round(((max_deg * double(value))/(max_pos - 1)) - (max_deg/2), 2);
end

