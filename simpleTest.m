%xử lý các t�?a độ test
% xla;
%t�?a độ các điểm:
% data= [1,0,0;
%     1,0,0.1;
%     1,0,0.2;
%     1,0,0.3;
%     1,0,0.4;
%     1,0,0.5;
%     1,0,0.6;
%     1,0,0.7;
%     1,0,0.8;
%     1,0,0.9;
%     1,0,1
%     ];

% Hiển thị mảng chứa các điểm
disp('Mảng chứa các điểm:');
disp(point_convert);
num_points = size(point_convert, 1);

ur10 = createUR10Robot();% khởi tạo robot
data_inv = zeros(num_points,6);
str_array = cell(size(data_inv, 1), 1);
for i = 1:(num_points)
    px = point_convert(i, 1);
    py = point_convert(i, 2);
    pz = point_convert(i, 3);
    K = invert(px,py,pz,ur10);% tính động h�?c nghịch
    data_inv(i, :) = K;
    row1 = K;
    str_row1 = sprintf('1  %f', row1);
    str_row1_with_1 = strcat(str_row1, '  1');
    str_array{i} = str_row1_with_1;
end
disp('Mảng chuỗi:');
disp(str_array);

% joint= [0.7217927733079286, -2.668194978368974, -0.4917656144910349, 2.176429363877939, -0.7217927733079287, 0];
% TT= forward(joint(1),joint(2),joint(3),joint(4),joint(5),joint(6),ur10);
% disp(TT);


disp('Program started');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    vrep.simxSetStringSignal(clientID, 'connect_status', 'ok', vrep.simx_opmode_oneshot);
    vrep.simxSetStringSignal(clientID, 'in4_ip', '127.0.0.1', vrep.simx_opmode_oneshot);
    disp('Connected to remote API server');
    pause(1);

    for i = 1:num_points
        % Chuỗi số cần gửi từ MATLAB
        numberString = str_array{i};
        % Gửi chuỗi số từ MATLAB tới V-REP
        vrep.simxSetStringSignal(clientID, 'numberString', numberString, vrep.simx_opmode_oneshot);
        while true
            [errorCode, stringData] = vrep.simxGetStringSignal(clientID, 'phanhoi', vrep.simx_opmode_blocking);
            if strcmp(stringData, '111')
                disp(['Chuỗi kí tự từ V-REP:', stringData]);
                vrep.simxSetStringSignal(clientID, 'phanhoi', '', vrep.simx_opmode_oneshot);
                break;
            end
        end
    end


    %gửi tín hiệu kết thúc
    vrep.simxSetStringSignal(clientID, 'ketthuc', '11', vrep.simx_opmode_oneshot);
    
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);

    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');

%%
function ur10= createUR10Robot()
    %bang DH
%     a = [0, 0, -0.612, -0.5723, 0, 0];
%     alpha = [0, pi/2, 0, 0, pi/2, -pi/2];
%     d = [0.1273, 0, 0, 0.1639, 0.1157, 0.0922];
    
    a = [0 , 0.647, 0.6005, 0, 0, 0 ];
    alpha = [pi/2, 0, 0, -pi/2, pi/2,0];
    d = [0.1632, 0.197, -0.1235, 0.1278, 0.1025, 0.094+0.105];
    theta = [0, pi/2, 0, -pi/2, 0, 0];
  
    % Tạo đối tượng robot UR10
    ur10 = SerialLink([
    Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', theta(1)), ...
    Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', theta(2)), ...
    Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', theta(3)), ...
    Revolute('d', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', theta(4)), ...
    Revolute('d', d(5), 'a', a(5), 'alpha', alpha(5), 'offset', theta(5)), ...
    Revolute('d', d(6), 'a', a(6), 'alpha', alpha(6), 'offset', theta(6))
    ]);
    ur10.name = 'abb';
end
%%
function J= invert(px,py,pz,ur10)
roll = deg2rad(0);
pitch = deg2rad(90);
yaw = deg2rad(0);

R11 = cos(pitch) * cos(yaw);
R12 = cos(yaw) * sin(roll) * sin(pitch) - cos(roll) * sin(yaw);
R13 = cos(roll) * cos(yaw) * sin(pitch) + sin(roll) * sin(yaw);
R21 = cos(pitch) * sin(yaw);
R22 = cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw);
R23 = -cos(yaw) * sin(roll) + cos(roll) * sin(pitch) * sin(yaw);
R31 = -sin(pitch);
R32 = cos(pitch) * sin(roll);
R33 = cos(roll) * cos(pitch);
 T = [R11, R12, R13, px;
     R21, R22, R23, py;
     R31, R32, R33, pz;
     0, 0, 0, 1];
 J = ur10.ikine(T, [0, 0, 0], 'mask', [1, 1, 1, 1, 1, 1]) ;
end
%%

function T= forward(theta1, theta2, theta3, theta4, theta5, theta6,ur10)
 theta = [theta1, theta2, theta3, theta4, theta5, theta6];
 T = ur10.fkine(theta);
end

