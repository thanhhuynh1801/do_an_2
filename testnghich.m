%%%%
a1=[1,0.5,0.5];
ur10 = createUR10Robot();% khởi tạo robot
K = invert(a1(1,1),a1(1,2),a1(1,3),ur10);% tính động học nghịch
disp(K);



function ur10= createUR10Robot()
    %bang DH   
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
% function J= invert(px,py,pz,ur10)
%  T = [0, 0, 1, px;
%      0, 1, 0, py;
%      -1, 0, 0, pz;
%      0, 0, 0, 1];
%  J = ur10.ikine(T, [0, 0, 0], 'mask', [1, 1, 1, 1, 1, 1]) ;
% end
%%
function T= forward(theta1, theta2, theta3, theta4, theta5, theta6,ur10)
 theta = [theta1, theta2, theta3, theta4, theta5, theta6];
 T = ur10.fkine(theta);
end

function J= invert(px,py,pz,r,p,y,ur10)
roll = deg2rad(r);
pitch = deg2rad(p);
yaw = deg2rad(y);

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

