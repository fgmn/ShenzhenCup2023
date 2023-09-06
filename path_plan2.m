% 算法梳理
% 1）起点、终点、障碍物、迭代次数、取点半径等的设定
% 2）以起点为中心，作半径为r的圆，均匀从圆上取八个点
% 3）分别计算八个点的前进“代价” 即 终点对其的引力+所有障碍物对其的斥力 
% 4）取“代价”最小的点的坐标，结合现有起点，计算得到新的起点，然后重复上述内容
% 5）当发现 一个点距离终点很近 or 迭代的次数计算完了 程序停止。

% 该程序中，computP 负责代价计算，为核心计算函数。 可对其进行修改，以实现其他优化功能。


% 参数：起点 终点 障碍物 的坐标
% 返回值： point1储存的一系列起点信息
function [ point1,point2 ] = path_plan2(begin1,over1,begin2,over2,obstacle)

iters=1;      % 迭代次数
curr1=begin1;   % 无人机A的起点坐标
curr2=begin2;   % 无人机B的起点坐标
testR=20;    % 测试8点的圆的半径为0.5

 while ((norm(curr1-over1)>20) || (norm(curr2-over2)>20)) &&  (iters<=20000)    % 未到终点&迭代次数不足
%for kk =1:10
    %  这里设定无人机B先走   无人机A再走
    
    % 无人机B移动
    if (norm(curr2-over2)>1)   %  无人机B还未到达目的地
        point2(:,iters)=curr2;  % point2为函数返回值  是记录无人机B飞行轨迹的
        %先求这八个点的坐标
        for i=1:9   % 求 以curr2为起点，testR为半径的圆上的八个均匀分布的点
            sin((i-5)*pi/36)
            cos((i-5)*pi/36)
            curr2
            testpoint(:,i)=[testR*cos((i-5)*pi/36)+curr2(1);testR*sin((i-5)*pi/36)+curr2(2)]
            testOut(:,i)=computP2(testpoint(:,i),over2,obstacle,2,curr1);   % 计算上述各个点的所受合力
        end
        for i=1:9
            %  计算每个点是否满足条件  以及选出代价最小的
            xb = testpoint(1,i);  % 有可能到达的新坐标
            yb = testpoint(2,i);
            xa = curr1(1);
            ya = curr1(2);
            dt = abs(xa*yb-xb*ya)/(sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)));
            db = sqrt(xb*xb+yb*yb);
            if dt>=500 || db<500
                testOut(:,i) = 100000;    % 设置一个很大的值  保证不会被选到
            end
        end
        testOut
        
        [temp num]=min(testOut); % 找出这八个点中，代价最小的点
        %迭代的距离为0.1
        curr2=(curr2+testpoint(:,num))/2;  % 将上述求得点，迭代到curr2上。 （求取的 curr2与testpoint1 的中点）
    end
    
    
    
    % 无人机A移动
    if (norm(curr1-over1)>1)   %  无人机A还未到达目的地
        %计算当前点附近半径为1的8个点的势能，然后让当前点的势能减去8个点的势能取差值最大的，确定这个
        %方向，就是下一步迭代的点
        point1(:,iters)=curr1;    % point1为函数返回值，储存每次遍历得到的新起点 curr1
    
        %先求这八个点的坐标
        for i=1:9   % 求 以curr1为起点，testR为半径的圆上的八个均匀分布的点
            testpoint(:,i)=[testR*cos((i-5)*pi/36)+curr1(1);testR*sin((i-5)*pi/36)+curr1(2)]
            testOut(:,i)=computP(testpoint(:,i),over1,obstacle,1,curr2);   % 计算上述各个点的所受合力
        end
        for i=1:9
            %  计算每个点是否满足条件  以及选出代价最小的
            xa = testpoint(1,i);  % 有可能到达的新坐标
            ya = testpoint(2,i);
            xb = curr2(1);
            yb = curr2(2);
            dt = abs(xa*yb-xb*ya)/(sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)));
            da = sqrt(xa*xa+ya*ya);
            if dt>=500 || da<500
                testOut(:,i) = 100000;    % 设置一个很大的值  保证不会被选到
            end
        end
        testOut
        
        [temp num]=min(testOut); % 找出这八个点中，代价最小的点
        %迭代的距离为0.1
        curr1=(curr1+testpoint(:,num))/2;  % 将上述求得点，迭代到curr上。 （求取的 curr与testpoint1 的中点）
    end
    
    
    
    
    plot(curr1(1),curr1(2),'og');      % 绘制得到的 新的起点curr1
    plot(curr2(1),curr2(2),'og');      % 绘制得到的 新的起点curr2
    pause(0.01);            % 程序暂停一会再继续运行 -- 体现出路径搜索的过程
    iters=iters+1;          % 迭代次数+1
end


end

%  无人机A

 % 计算周围几个点的势能（代价）
 % 参数：当前起点  终点  障碍物   的坐标
 function [ output ] = computP( curr,over,obstacle,flag,p)  % flag记录的是当前计算的是无人机A还是无人机B  1代表A  2代表B

 
% 几个用于计算的相关参数 
k_att=0.000001;
repu=0;
k_rep=1;
Q_star=10000;     %。障碍物的斥力作用半径

% 计算终点对当前点的引力  
% tips：这个数值越小，说明当前点离终点越近
attr=1/2*k_att*(norm(curr-over))^2;     % 引力计算公式
if flag==1
    obstacle = [obstacle p];   % 添加上无人机B作为一个障碍点
elseif flag==2
    obstacle = [obstacle p];   % 添加上无人机A作为一个障碍点
end

% 计算所有障碍物对当前点斥力合 
% tips：斥力合越小，说明当前点遇到障碍物的概率越小
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star    % 障碍物到当前点距离在阈值内，考虑其斥力影响
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2*(norm(curr-over))^2;    % 斥力计算公式
        % ps： 当 curr和obstacle 坐标重合时，是否会出现repu计算卡死？ 是否需要对该条件进行设置
    else       % 障碍物到当前点距离在阈值外，忽略斥力影响
        repu=repu+0;
    end
end
attr
repu
output=-attr+repu;   % 引力+斥力  这个数值越小，越适合当作下一个起点

 end
 
 
 
 
 
 %无人机B
 
 % 计算周围几个点的势能（代价）
 % 参数：当前起点  终点  障碍物   的坐标
 function [ output ] = computP2( curr,over,obstacle,flag,p)  % flag记录的是当前计算的是无人机A还是无人机B  1代表A  2代表B

 
% 几个用于计算的相关参数 
k_att=0.000001;
repu=0;
k_rep=1;
Q_star=1000;     %。障碍物的斥力作用半径

% 计算终点对当前点的引力  
% tips：这个数值越小，说明当前点离终点越近
attr=1/2*k_att*(norm(curr-over))^2;     % 引力计算公式
if flag==1
    obstacle = [obstacle p];   % 添加上无人机B作为一个障碍点
elseif flag==2
    obstacle = [obstacle p];   % 添加上无人机A作为一个障碍点
end

% 计算所有障碍物对当前点斥力合 
% tips：斥力合越小，说明当前点遇到障碍物的概率越小
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star    % 障碍物到当前点距离在阈值内，考虑其斥力影响
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2*(norm(curr-over))^2;    % 斥力计算公式
        % ps： 当 curr和obstacle 坐标重合时，是否会出现repu计算卡死？ 是否需要对该条件进行设置
    else       % 障碍物到当前点距离在阈值外，忽略斥力影响
        repu=repu+0;
    end
end
output=-attr+repu;   % 引力+斥力  这个数值越小，越适合当作下一个起点

 end  