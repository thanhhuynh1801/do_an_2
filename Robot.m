% function ur10= robot();
%     a = [0, 0, -0.612, -0.5723, 0, 0];
%         alpha = [0, pi/2, 0, 0, pi/2, -pi/2];
%         d = [0.1273, 0, 0, 0.1639, 0.1157, 0.0922];
% 
%         % Tạo đối tượng robot UR10
%         ur10 = SerialLink([
%             Revolute('d', d(1), 'a', a(1), 'alpha', alpha(1)), ...
%             Revolute('d', d(2), 'a', a(2), 'alpha', alpha(2)), ...
%             Revolute('d', d(3), 'a', a(3), 'alpha', alpha(3)), ...
%             Revolute('d', d(4), 'a', a(4), 'alpha', alpha(4)), ...
%             Revolute('d', d(5), 'a', a(5), 'alpha', alpha(5)), ...
%             Revolute('d', d(6), 'a', a(6), 'alpha', alpha(6))
%         ]);
% end

robot = loadrobot("kukaIiwa14","DataFormat","row");
env = {collisionBox(0.5,0.5,0.05) collisionSphere(0.3)};
env{1}.Pose(3,end) = -0.05;
env{2}.Pose(1:3,end) = [0.1 0.2 0.8];

show(robot);
hold on
show(env{1})
show(env{2})
rrt = manipulatorRRT(robot,env);
rrt.ValidationDistance = 0.2;
% % rrt.SkippedSelfCollisions = "parent";

startConfig = [0.1 0 0 0 0 0 0];
goalConfig =  [1 1 1 1 1 1 1];
rng(0)
path = plan(rrt,startConfig,goalConfig);
disp(path);

