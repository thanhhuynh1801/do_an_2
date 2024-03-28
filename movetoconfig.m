function [pos]= movetoconfig(vel,accel,jerk,pos1,pos2) 
% vel = 120;
% accel = 40;
% jerk = 80;
maxVel = [vel*pi/180, vel*pi/180, vel*pi/180, vel*pi/180, vel*pi/180, vel*pi/180];
maxAccel = [accel*pi/180, accel*pi/180, accel*pi/180, accel*pi/180, accel*pi/180, accel*pi/180];
maxJerk = [jerk*pi/180, jerk*pi/180, jerk*pi/180, jerk*pi/180, jerk*pi/180, jerk*pi/180];

% Convert numeric array to cell array of character vectors
str_maxvel = num2cell(maxVel);
str_maxaccel = num2cell(maxAccel);
str_maxjerk = num2cell(maxJerk);
str_pos1 = num2cell(pos1);
str_pos2 = num2cell(pos2);
% Convert cell array of character vectors to string array
str_maxvel = string(str_maxvel);
str_maxaccel = string(str_maxaccel);
str_maxjerk = string(str_maxjerk);
str_pos1 = string(str_pos1);
str_pos2 = string(str_pos2);
% Join the elements with a space delimiter
maxvel_inv = strjoin(str_maxvel, ' ');
maxaccel_inv = strjoin(str_maxaccel, ' ');
maxjerk_inv = strjoin(str_maxjerk, ' ');
pos1_inv = strjoin(str_pos1, ' ');
pos2_inv = strjoin(str_pos2, ' ');

% Đầu vào từ MATLAB
c_pos_m = pos1_inv;
t_pos_m = pos2_inv;
max_vel_m = maxvel_inv;
max_acc_m = maxaccel_inv;
max_jerk_m = maxjerk_inv;
step_time_m = '0.05';


% Tạo lệnh gọi Python
% python_cmd = ['python E:\BTL_ROBOTIC\ruckig_pos.py "', c_pos_m, '" "', t_pos_m, '" "', max_vel_m, '" "', max_acc_m, '" "', max_jerk_m, '" "', step_time_m, '"'];
python_cmd = sprintf('python ruckig_pos.py "%s" "%s" "%s" "%s" "%s" "%s" "%s"', c_pos_m, t_pos_m, max_vel_m, max_acc_m, max_jerk_m, step_time_m);
% Gọi chương trình Python từ MATLAB
[status,position] = system(python_cmd);
% Xóa ký tự '[' và ']' để tạo thành một chuỗi con
data_str = strrep(position, '[', '');
data_str = strrep(data_str, ']', '');
% Tách chuỗi thành từng hàng
rows = strsplit(data_str, ', ');
% Đánh giá từng hàng thành mảng
data = cellfun(@eval, rows, 'UniformOutput', false);
% Chuyển đổi cell array thành ma trận
data = cell2mat(data);
% Reshape ma trận thành ma trận 6 cột
pos = reshape(data, 6, []).';

% % In kết quả
% disp(pos(1,1));
