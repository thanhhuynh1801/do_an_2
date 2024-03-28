% modelpath= 'C:\Users\huynh\Downloads\do_an_2_matlab (1)\remakeI10.slx';
% open_system(modelname);
modelname= 'remakeI10';
open_system(modelname);
set_param(modelname,'BlockReduction','off');
set_param(modelname,'StopTime','inf');
set_param(modelname,'simulationMode','normal');

set_param(modelname,'StartFcn','1');
set_param(modelname,'simulationCommand','start');

% a1=[1,0.5,0.5];
% ur10 = createUR10Robot();% khởi tạo robot
% K = invert(a1(1,1),a1(1,2),a1(1,3),ur10);% tính động học nghịch
% disp(K);
%%
disp('Máº£ng chá»©a cÃ¡c Ä‘iá»ƒm:');
disp(point_convert);
num_points = size(point_convert, 1);

ur10 = createUR10Robot();
data_inv = zeros(num_points,6);
str_array = cell(size(data_inv, 1), 1);
for i = 1:(num_points)
    px = point_convert(i, 1);
    py = point_convert(i, 2);
    pz = point_convert(i, 3);
    K = invert(px,py,pz,ur10);
    data_inv(i, :) = K;
    row1 = K;
    str_row1 = sprintf('1  %f', row1);
    str_row1_with_1 = strcat(str_row1, '  1');
    str_array{i} = str_row1_with_1;
end
disp('mang cau hinh:');
disp(str_array);


%%
% T= forward(K(1,1),K(1,2), K(1,3),K(1,4),K(1,5),K(1,6),ur10);
% disp(T);
%%

vel = 120;
accel = 40;
jerk = 80;
global pre_pos;
pre_pos=[0,0,0,0,0,0];

%%
for j = 1:(num_points)
    targetPos = data_inv(j, :);
    [position]= movetoconfig(vel,accel,jerk,pre_pos,targetPos);
    %disp(position(1,6));
    
    len_pos = size(position, 1);
    for i = 1:(len_pos)
        %Rotate robot in Simulink
        set_param([modelname '/Slider Gain'], 'Gain',num2str(position(i,1)*180/pi) );
        set_param([modelname '/Slider Gain1'], 'Gain',num2str(position(i,2)*180/pi));
        set_param([modelname '/Slider Gain2'], 'Gain',num2str(position(i,3)*180/pi));
        set_param([modelname '/Slider Gain3'], 'Gain',num2str(-position(i,4)*180/pi));
        set_param([modelname '/Slider Gain4'], 'Gain',num2str(-position(i,5)*180/pi));
        set_param([modelname '/Slider Gain5'], 'Gain',num2str(position(i,6)*180/pi));
        pause(0.01);
    end
    pre_pos = targetPos;   
end
move_home();
set_param(modelname, 'simulationCommand', 'stop');
disp('ketthuc');

%% move home
function move_home()
    modelname= 'remakeI10';
    vel = 120;
    accel = 40;
    jerk = 80;
    global pre_pos;
    home=[0,0,0,0,0,0];
    [position]= movetoconfig(vel,accel,jerk,pre_pos,home);

    len_pos = size(position, 1);
    for i = 1:(len_pos)
        %Rotate robot in Simulink
        set_param([modelname '/Slider Gain'], 'Gain',num2str(position(i,1)*180/pi) );
        set_param([modelname '/Slider Gain1'], 'Gain',num2str(position(i,2)*180/pi));
        set_param([modelname '/Slider Gain2'], 'Gain',num2str(position(i,3)*180/pi));
        set_param([modelname '/Slider Gain3'], 'Gain',num2str(-position(i,4)*180/pi));
        set_param([modelname '/Slider Gain4'], 'Gain',num2str(-position(i,5)*180/pi));
        set_param([modelname '/Slider Gain5'], 'Gain',num2str(position(i,6)*180/pi));
        pause(0.01);
    end
    pre_pos = home;
    
end
%%
function ur10= createUR10Robot()
    %bang DH
%     a = [0, 0, -0.612, -0.5723, 0, 0];
%     alpha = [0, pi/2, 0, 0, pi/2, -pi/2];
%     d = [0.1273, 0, 0, 0.1639, 0.1157, 0.0922];
    
    a = [0 , 0.647, 0.6005, 0, 0, 0 ];
    alpha = [pi/2, 0, 0, -pi/2, pi/2,0];
    d = [0.1632, 0.197, -0.1235, 0.1278, 0.1025, 0.094];
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
 T = [0, 0, 1, px;
     0, 1, 0, py;
     -1, 0, 0, pz;
     0, 0, 0, 1];
 J = ur10.ikine(T, [0, 0, 0], 'mask', [1, 1, 1, 1, 1, 1]) ;
end
%%
function T= forward(theta1, theta2, theta3, theta4, theta5, theta6,ur10)
 theta = [theta1, theta2, theta3, theta4, theta5, theta6];
 T = ur10.fkine(theta);
end

