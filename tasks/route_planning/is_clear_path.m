function is_clear = is_clear_path(driveability_map, start_pos, end_pos)

% default value of is_clear
is_clear = false
% unpack values into variables
[x1, y1] = start_pos{:}
[x2, y2] = end_pos{:}
% calculate dy and dx
dif_x = x2 - x1
dif_y = y2 - y1

if dif_x == 0
    % step = sign of dif_y
    % ys = range of values from 0 to dif_y + step with step steps
    % xs = array of x1 repeated n times where n is the number of rows in ys
else
    % m = dy/dx
    % b = y1 - m*x1
end