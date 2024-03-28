%% xử lý ảnh
path='image1.png';
Point=image_processing(path);

% Lấy số lượng điểm trên đư�?ng biên
numPoints = size(Point, 1);
% Mảng để lưu các điểm
pointArray = zeros(numPoints, 4);
% thông số calib camera
a=0.002; % m/pixel
% Lặp qua danh sách t�?a độ và lưu từng điểm vào mảng
for i = 1:numPoints
    pointArray(i, 1) = Point(i, 1)*a;
    pointArray(i, 2) = Point(i, 2)*a;
    pointArray(i, 3) = 0;
    pointArray(i, 4) = 1;
end
disp(pointArray)
% % Hiển thị mảng chứa các điểm
% disp('Mảng chứa các điểm:');
% disp(pointArray)

%% chuyển t�?a độ ảnh sang t�?a độ base robot
A = [0.725, 0.5, 1]; %x1
B = [0.725, 0.5, 0.9];   %x2
C= [0.725,0.4,1];    %y1
T= findIntersectionPoint(A,B,C);
disp(T);
point_convert = zeros(numPoints, 4);
for i = 1:numPoints
    %toado= T^-1*(pointArray(i,:).')
    point_convert(i,:) = T*(pointArray(i,:).');
end
% Hiển thị mảng chứa các điểm
disp('Mảng chứa các điểm:');
disp(point_convert);
disp(1);
writematrix(point_convert(:, 1:3), 'point_xla.txt');

%%
grid on
T00 = [  1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
base = [ 0 ; 0 ; 0 ; 1]; 
x0 = base(1,1); y0 = base(2,1); z0 =base(3,1);

draw_coordinates(A(1),A(2),A(3),T);
draw_coordinates(x0,y0,z0,T00);

for i = 1:numPoints
    %pointArray(i, 1) = Point(i, 1)*a;
    scatter3(point_convert(i, 1),point_convert(i, 2),point_convert(i, 3), 'filled');
end
% Kích thước của ảnh: 343x442 pixels
p1 = [A(1) A(2) A(3)];
p2 = [A(1) A(2) (A(3)-343*a)];
p3 = [A(1) (A(2)-442*a) (A(3)-343*a)];
p4 = [A(1) (A(2)-442*a) A(3)]; 
x = [p1(1) p2(1) p3(1) p4(1)];
y = [p1(2) p2(2) p3(2) p4(2)];
z = [p1(3) p2(3) p3(3) p4(3)];
fill3(x, y, z, 1);

axis([-3 3 -3 3 0 3]);
xlabel('x');
ylabel('y');
zlabel('z');
rotate3d on

function draw_coordinates(x, y, z, mt)  
%x_axis
plot3([x (1*mt(1,1)+x)],[y (1*mt(2,1)+y)],[z (1*mt(3,1)+z)],'r','linewidth',2);
text((1*mt(1,1)+x),(1*mt(2,1)+y),(1*mt(3,1)+z),'x','HorizontalAlignment','right','FontSize',11);
hold on;
%y_axis
plot3([x (1*mt(1,2)+x)],[y (1*mt(2,2)+y)],[z (1*mt(3,2)+z)],'g','linewidth',2);
text((1*mt(1,2)+x),(1*mt(2,2)+y),(1*mt(3,2)+z),'y','HorizontalAlignment','right','FontSize',11);
%z_axis
plot3([x (1*mt(1,3)+x)],[y (1*mt(2,3)+y)],[z (1*mt(3,3)+z)],'b','linewidth',2);
text((1*mt(1,3)+x),(1*mt(2,3)+y),(1*mt(3,3)+z),'z','HorizontalAlignment','right','FontSize',11);
end

%% Convert coordinates to Baes
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