
theta2= 3.823535*pi/180;
ur10= createUR10Robot();
t= forward(0,theta2,0,0,0,0,ur10);
disp(t);




function ur10= createUR10Robot()
    %bang DH
%     a = [0, 0, -0.612, -0.5723, 0, 0];
%     alpha = [0, pi/2, 0, 0, pi/2, -pi/2];
%     d = [0.1273, 0, 0, 0.1639, 0.1157, 0.0922];
    
    a = [0 , 0.647, 0.6005, 0, 0, 0 ];
    alpha = [pi/2, 0, 0, -pi/2, pi/2,0];
    d = [0.1632, 0.197, -0.1235, 0.1278, 0.1025, 0.094+0.105];
    theta = [0, pi/2, 0, -pi/2, 0, 0];
  
    % Táº¡o Ä‘á»‘i tÆ°á»£ng robot UR10
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

function T= forward(theta1, theta2, theta3, theta4, theta5, theta6,ur10)
 theta = [theta1, theta2, theta3, theta4, theta5, theta6];
 T = ur10.fkine(theta);
end