% 算法梳理
% 1）起点、终点、障碍物、迭代次数、取点半径等的设定
% 2）以起点为中心，作半径为r的圆，均匀从圆上取八个点
% 3）分别计算八个点的前进“代价” 即 终点对其的引力+所有障碍物对其的斥力 
% 4）取“代价”最小的点的坐标，结合现有起点，计算得到新的起点，然后重复上述内容
% 5）当发现 一个点距离终点很近 or 迭代的次数计算完了 程序停止。

% 该程序中，computP 负责代价计算，为核心计算函数。 可对其进行修改，以实现其他优化功能。


% 参数：起点 终点 障碍物 的坐标
% 返回值： point储存的一系列起点信息
function [ point ] = path_plan(begin,over,obstacle)

iters=1;      % 迭代次数
curr=begin;   % 起点坐标
testR=0.2;    % 测试8点的圆的半径为0.5

while (norm(curr-over)>0.2) &&  (iters<=2000)    % 未到终点&迭代次数不足
   
%     attr=attractive(curr,over);
%     repu=repulsion(curr,obstacle);
    %curoutput=computP(curr,over,obstacle);
    %计算当前点附近半径为0.2的8个点的势能，然后让当前点的势能减去8个点的势能取差值最大的，确定这个
    %方向，就是下一步迭代的点
    
    point(:,iters)=curr;    % point为函数返回值，储存每次遍历得到的新起点 curr
    
    %先求这八个点的坐标
    for i=1:8   % 求 以curr为起点，testR为半径的圆上的八个均匀分布的点
        testPoint(:,i)=[testR*sin((i-1)*pi/4)+curr(1);testR*cos((i-1)*pi/4)+curr(2)];
        testOut(:,i)=computP(testPoint(:,i),over,obstacle);   % 计算上述各个点的所受合力
    end
    [temp num]=min(testOut); % 找出这八个点中，代价最小的点 
    
    %迭代的距离为0.1
    curr=(curr+testPoint(:,num))/2;  % 将上述求得点，迭代到curr上。 （求取的 curr与testPoint 的中点）
    plot(curr(1),curr(2),'og');      % 绘制得到的 新的起点curr
    pause(0.01);            % 程序暂停一会再继续运行 -- 体现出路径搜索的过程
    iters=iters+1;          % 迭代次数+1
end
end



 % 计算周围几个点的势能（代价）
 % 参数：当前起点  终点  障碍物   的坐标
 function [ output ] = computP( curr,over,obstacle )

% 几个用于计算的相关参数 
k_att=1;
repu=0;
k_rep=100;
Q_star=2;     %。障碍物的斥力作用半径

% 计算终点对当前点的引力  
% tips：这个数值越小，说明当前点离终点越近
attr=1/2*k_att*(norm(curr-over))^2;     % 引力计算公式

% 计算所有障碍物对当前点斥力合 
% tips：斥力合越小，说明当前点遇到障碍物的概率越小
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star    % 障碍物到当前点距离在阈值内，考虑其斥力影响
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2;    % 斥力计算公式
        % ps： 当 curr和obstacle 坐标重合时，是否会出现repu计算卡死？ 是否需要对该条件进行设置
    else       % 障碍物到当前点距离在阈值外，忽略斥力影响
        repu=repu+0;
    end
end

output=attr+repu;   % 引力+斥力  这个数值越小，越适合当作下一个起点

 end  