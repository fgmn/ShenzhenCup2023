% �㷨����
% 1����㡢�յ㡢�ϰ������������ȡ��뾶�ȵ��趨
% 2�������Ϊ���ģ����뾶Ϊr��Բ�����ȴ�Բ��ȡ�˸���
% 3���ֱ����˸����ǰ�������ۡ� �� �յ���������+�����ϰ������ĳ��� 
% 4��ȡ�����ۡ���С�ĵ�����꣬���������㣬����õ��µ���㣬Ȼ���ظ���������
% 5�������� һ��������յ�ܽ� or �����Ĵ����������� ����ֹͣ��

% �ó����У�computP ������ۼ��㣬Ϊ���ļ��㺯���� �ɶ�������޸ģ���ʵ�������Ż����ܡ�


% ��������� �յ� �ϰ��� ������
% ����ֵ�� point�����һϵ�������Ϣ
function [ point ] = path_plan(begin,over,obstacle)

iters=1;      % ��������
curr=begin;   % �������
testR=0.2;    % ����8���Բ�İ뾶Ϊ0.5

while (norm(curr-over)>0.2) &&  (iters<=2000)    % δ���յ�&������������
   
%     attr=attractive(curr,over);
%     repu=repulsion(curr,obstacle);
    %curoutput=computP(curr,over,obstacle);
    %���㵱ǰ�㸽���뾶Ϊ0.2��8��������ܣ�Ȼ���õ�ǰ������ܼ�ȥ8���������ȡ��ֵ���ģ�ȷ�����
    %���򣬾�����һ�������ĵ�
    
    point(:,iters)=curr;    % pointΪ��������ֵ������ÿ�α����õ�������� curr
    
    %������˸��������
    for i=1:8   % �� ��currΪ��㣬testRΪ�뾶��Բ�ϵİ˸����ȷֲ��ĵ�
        testPoint(:,i)=[testR*sin((i-1)*pi/4)+curr(1);testR*cos((i-1)*pi/4)+curr(2)];
        testOut(:,i)=computP(testPoint(:,i),over,obstacle);   % ������������������ܺ���
    end
    [temp num]=min(testOut); % �ҳ���˸����У�������С�ĵ� 
    
    %�����ľ���Ϊ0.1
    curr=(curr+testPoint(:,num))/2;  % ��������õ㣬������curr�ϡ� ����ȡ�� curr��testPoint ���е㣩
    plot(curr(1),curr(2),'og');      % ���Ƶõ��� �µ����curr
    pause(0.01);            % ������ͣһ���ټ������� -- ���ֳ�·�������Ĺ���
    iters=iters+1;          % ��������+1
end
end



 % ������Χ����������ܣ����ۣ�
 % ��������ǰ���  �յ�  �ϰ���   ������
 function [ output ] = computP( curr,over,obstacle )

% �������ڼ������ز��� 
k_att=1;
repu=0;
k_rep=100;
Q_star=2;     %���ϰ���ĳ������ð뾶

% �����յ�Ե�ǰ�������  
% tips�������ֵԽС��˵����ǰ�����յ�Խ��
attr=1/2*k_att*(norm(curr-over))^2;     % �������㹫ʽ

% ���������ϰ���Ե�ǰ������� 
% tips��������ԽС��˵����ǰ�������ϰ���ĸ���ԽС
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star    % �ϰ��ﵽ��ǰ���������ֵ�ڣ����������Ӱ��
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2;    % �������㹫ʽ
        % ps�� �� curr��obstacle �����غ�ʱ���Ƿ�����repu���㿨���� �Ƿ���Ҫ�Ը�������������
    else       % �ϰ��ﵽ��ǰ���������ֵ�⣬���Գ���Ӱ��
        repu=repu+0;
    end
end

output=attr+repu;   % ����+����  �����ֵԽС��Խ�ʺϵ�����һ�����

 end  