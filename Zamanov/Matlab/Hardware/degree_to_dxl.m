function [pos] = degree_to_dxl(value)
    max_pos = 4096;
    max_deg = 360;
    pos = (round((max_pos - 1) * ((max_deg/2 + double(value))/max_deg), 0));
    pos = min(max(pos, 0), max_pos - 1);
end

