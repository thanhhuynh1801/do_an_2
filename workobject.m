
grid on
T00 = [  1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
base = [ 0 ; 0 ; 0 ; 1]; 
x0 = base(1,1); y0 = base(2,1); z0 =base(3,1);

% Điểm A
A = [0.5, -0.79, 1.451];
B = [0.5, -0.79, 1.4];
C= [0.45,-0.79,1.451];

t= findIntersectionPoint(A,B,C);
disp(t)
draw_coordinates(A(1),A(2),A(3),t);
draw_coordinates(x0,y0,z0,T00);

D =[0.3480;0.2240;0;1];
TT=t*D
scatter3(TT(1), TT(2), TT(3), 'filled');
axis([-5 5 -5 5 0 5]);
xlabel('x');
ylabel('y');
zlabel('z');
rotate3d on


function draw_coordinates(x, y, z, mt)  
%x_axis
plot3([x (100*mt(1,1)+x)],[y (100*mt(2,1)+y)],[z (100*mt(3,1)+z)],'r','linewidth',2);
%text((130*mt(1,1)+x),(130*mt(2,1)+y),(130*mt(3,1)+z),'x','HorizontalAlignment','right','FontSize',11);
hold on;
%y_axis
plot3([x (100*mt(1,2)+x)],[y (100*mt(2,2)+y)],[z (100*mt(3,2)+z)],'g','linewidth',2);
%text((130*mt(1,2)+x),(130*mt(2,2)+y),(130*mt(3,2)+z),'y','HorizontalAlignment','right','FontSize',11);
%z_axis
plot3([x (100*mt(1,3)+x)],[y (100*mt(2,3)+y)],[z (100*mt(3,3)+z)],'b','linewidth',2);
%text((130*mt(1,3)+x),(130*mt(2,3)+y),(130*mt(3,3)+z),'z','HorizontalAlignment','right','FontSize',11);
end


function intersection_point = findIntersectionPoint(point1, point2, point3)
    x1 = point1(1);
    y1 = point1(2);
    z1 = point1(3);
    
    x2 = point2(1);
    y2 = point2(2);
    z2 = point2(3);
    
    x3 = point3(1);
    y3 = point3(2);
    z3 = point3(3);
    
    % Tính vector hướng của X
    direction_x = [x2 - x1, y2 - y1, z2 - z1];
    direction_x = direction_x / norm(direction_x);
    % Tính vector hướng của Z
    direction_z = cross(direction_x, [x3 - x1, y3 - y1, z3 - z1]);
    direction_z = direction_z / norm(direction_z);
    % tính vecto hướng của y
    direction_y = cross(direction_z,direction_x);
    direction_y = direction_y / norm(direction_y);
    intersection_point=[direction_x(1) direction_y(1) direction_z(1) x1;
        direction_x(2) direction_y(2) direction_z(2) y1;
        direction_x(3) direction_y(3) direction_z(3) z1;
        0 0 0 1];
end