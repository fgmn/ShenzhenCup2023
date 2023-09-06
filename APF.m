% -------------------------------------------------------------------------
% 人工势场算法 解决无人机协同避障问题
% -------------------------------------------------------------------------

% 流程梳理 - 人工势场法
% 1）初始化起点、终点、障碍物、迭代次数、取点半径等参数的设定   其中A无人机和B无人机的起点终点是不同的
% 2）以起点为中心，作半径为r的圆，从圆上取八个均布的点
% 3）分别计算八个点的前进“代价”—— 终点对其的引力+所有障碍物对其的斥力 
% 4）取“代价”最小的点的坐标，结合现有起点，计算得到新的起点，然后重复上述内容

% 5）需要判断一下新的起点是否违背了问题的约束   约束1.和另一架无人机连接的线段与障碍圆必须相交  约束2. 两架无人机必须在障碍圆外部
% 约束3. 保证转弯半径必须大于30m

% 6）当发现 一个点距离终点很近 or 迭代的次数计算完 程序停止。

%%
clc
clear
close all

figure(1);
axis([-2500 4000 -2000 2000]);   % 地图 15x15
% 障碍物(0,0)  A站(-1000,0) B站(3500,0)

begin1=[-1000;0];         % 起点   相对于A无人机而言
over1=[3500;0];        % 终点

begin2=[3500;0];         % 起点   相对于B无人机而言
over2=[-1000;0];        % 终点

obstacle=[0;0];  % 障碍物x;y坐标

% 绘制起点、终点、障碍物
hold on;
plot(begin1(1),begin1(2),'*b','MarkerSize',10);
plot(over1(1),over1(2),'*b','MarkerSize',10);
plot(obstacle(1,:),obstacle(2,:),'ob');

 for i=1:size(obstacle,2)      % 在个障碍物点处，绘制椭圆。  'Curvature' 矩形的曲率
    rectangle('Position',[obstacle(1,i)-0.5,obstacle(2,i)-0.5,1,1],'Curvature',[1,1],'FaceColor','r');
 end
 
 % 定义圆的参数
center_x = 0;  % 圆心 x 坐标
center_y = 0;  % 圆心 y 坐标
radius = 500;   % 半径

% 画圆
rectangle('Position', [center_x - radius, center_y - radius, 2*radius, 2*radius], ...
          'Curvature', [1, 1], 'EdgeColor', 'b', 'LineWidth', 2);

% 设置坐标轴范围
axis equal;
grid on;
 
[point1,point2] = path_plan2(begin1,over1,begin2,over2,obstacle);  % 计算并绘制出路径