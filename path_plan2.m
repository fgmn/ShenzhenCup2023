% �㷨����
% 1����㡢�յ㡢�ϰ������������ȡ��뾶�ȵ��趨
% 2�������Ϊ���ģ����뾶Ϊr��Բ�����ȴ�Բ��ȡ�˸���
% 3���ֱ����˸����ǰ�������ۡ� �� �յ���������+�����ϰ������ĳ��� 
% 4��ȡ�����ۡ���С�ĵ�����꣬���������㣬����õ��µ���㣬Ȼ���ظ���������
% 5�������� һ��������յ�ܽ� or �����Ĵ����������� ����ֹͣ��

% �ó����У�computP ������ۼ��㣬Ϊ���ļ��㺯���� �ɶ�������޸ģ���ʵ�������Ż����ܡ�


% ��������� �յ� �ϰ��� ������
% ����ֵ�� point1�����һϵ�������Ϣ
function [ point1,point2 ] = path_plan2(begin1,over1,begin2,over2,obstacle)

iters=1;      % ��������
curr1=begin1;   % ���˻�A���������
curr2=begin2;   % ���˻�B���������
testR=20;    % ����8���Բ�İ뾶Ϊ0.5

 while ((norm(curr1-over1)>20) || (norm(curr2-over2)>20)) &&  (iters<=20000)    % δ���յ�&������������
%for kk =1:10
    %  �����趨���˻�B����   ���˻�A����
    
    % ���˻�B�ƶ�
    if (norm(curr2-over2)>1)   %  ���˻�B��δ����Ŀ�ĵ�
        point2(:,iters)=curr2;  % point2Ϊ��������ֵ  �Ǽ�¼���˻�B���й켣��
        %������˸��������
        for i=1:9   % �� ��curr2Ϊ��㣬testRΪ�뾶��Բ�ϵİ˸����ȷֲ��ĵ�
            sin((i-5)*pi/36)
            cos((i-5)*pi/36)
            curr2
            testpoint(:,i)=[testR*cos((i-5)*pi/36)+curr2(1);testR*sin((i-5)*pi/36)+curr2(2)]
            testOut(:,i)=computP2(testpoint(:,i),over2,obstacle,2,curr1);   % ������������������ܺ���
        end
        for i=1:9
            %  ����ÿ�����Ƿ���������  �Լ�ѡ��������С��
            xb = testpoint(1,i);  % �п��ܵ����������
            yb = testpoint(2,i);
            xa = curr1(1);
            ya = curr1(2);
            dt = abs(xa*yb-xb*ya)/(sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)));
            db = sqrt(xb*xb+yb*yb);
            if dt>=500 || db<500
                testOut(:,i) = 100000;    % ����һ���ܴ��ֵ  ��֤���ᱻѡ��
            end
        end
        testOut
        
        [temp num]=min(testOut); % �ҳ���˸����У�������С�ĵ�
        %�����ľ���Ϊ0.1
        curr2=(curr2+testpoint(:,num))/2;  % ��������õ㣬������curr2�ϡ� ����ȡ�� curr2��testpoint1 ���е㣩
    end
    
    
    
    % ���˻�A�ƶ�
    if (norm(curr1-over1)>1)   %  ���˻�A��δ����Ŀ�ĵ�
        %���㵱ǰ�㸽���뾶Ϊ1��8��������ܣ�Ȼ���õ�ǰ������ܼ�ȥ8���������ȡ��ֵ���ģ�ȷ�����
        %���򣬾�����һ�������ĵ�
        point1(:,iters)=curr1;    % point1Ϊ��������ֵ������ÿ�α����õ�������� curr1
    
        %������˸��������
        for i=1:9   % �� ��curr1Ϊ��㣬testRΪ�뾶��Բ�ϵİ˸����ȷֲ��ĵ�
            testpoint(:,i)=[testR*cos((i-5)*pi/36)+curr1(1);testR*sin((i-5)*pi/36)+curr1(2)]
            testOut(:,i)=computP(testpoint(:,i),over1,obstacle,1,curr2);   % ������������������ܺ���
        end
        for i=1:9
            %  ����ÿ�����Ƿ���������  �Լ�ѡ��������С��
            xa = testpoint(1,i);  % �п��ܵ����������
            ya = testpoint(2,i);
            xb = curr2(1);
            yb = curr2(2);
            dt = abs(xa*yb-xb*ya)/(sqrt((xa-xb)*(xa-xb)+(ya-yb)*(ya-yb)));
            da = sqrt(xa*xa+ya*ya);
            if dt>=500 || da<500
                testOut(:,i) = 100000;    % ����һ���ܴ��ֵ  ��֤���ᱻѡ��
            end
        end
        testOut
        
        [temp num]=min(testOut); % �ҳ���˸����У�������С�ĵ�
        %�����ľ���Ϊ0.1
        curr1=(curr1+testpoint(:,num))/2;  % ��������õ㣬������curr�ϡ� ����ȡ�� curr��testpoint1 ���е㣩
    end
    
    
    
    
    plot(curr1(1),curr1(2),'og');      % ���Ƶõ��� �µ����curr1
    plot(curr2(1),curr2(2),'og');      % ���Ƶõ��� �µ����curr2
    pause(0.01);            % ������ͣһ���ټ������� -- ���ֳ�·�������Ĺ���
    iters=iters+1;          % ��������+1
end


end

%  ���˻�A

 % ������Χ����������ܣ����ۣ�
 % ��������ǰ���  �յ�  �ϰ���   ������
 function [ output ] = computP( curr,over,obstacle,flag,p)  % flag��¼���ǵ�ǰ����������˻�A�������˻�B  1����A  2����B

 
% �������ڼ������ز��� 
k_att=0.000001;
repu=0;
k_rep=1;
Q_star=10000;     %���ϰ���ĳ������ð뾶

% �����յ�Ե�ǰ�������  
% tips�������ֵԽС��˵����ǰ�����յ�Խ��
attr=1/2*k_att*(norm(curr-over))^2;     % �������㹫ʽ
if flag==1
    obstacle = [obstacle p];   % ��������˻�B��Ϊһ���ϰ���
elseif flag==2
    obstacle = [obstacle p];   % ��������˻�A��Ϊһ���ϰ���
end

% ���������ϰ���Ե�ǰ������� 
% tips��������ԽС��˵����ǰ�������ϰ���ĸ���ԽС
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star    % �ϰ��ﵽ��ǰ���������ֵ�ڣ����������Ӱ��
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2*(norm(curr-over))^2;    % �������㹫ʽ
        % ps�� �� curr��obstacle �����غ�ʱ���Ƿ�����repu���㿨���� �Ƿ���Ҫ�Ը�������������
    else       % �ϰ��ﵽ��ǰ���������ֵ�⣬���Գ���Ӱ��
        repu=repu+0;
    end
end
attr
repu
output=-attr+repu;   % ����+����  �����ֵԽС��Խ�ʺϵ�����һ�����

 end
 
 
 
 
 
 %���˻�B
 
 % ������Χ����������ܣ����ۣ�
 % ��������ǰ���  �յ�  �ϰ���   ������
 function [ output ] = computP2( curr,over,obstacle,flag,p)  % flag��¼���ǵ�ǰ����������˻�A�������˻�B  1����A  2����B

 
% �������ڼ������ز��� 
k_att=0.000001;
repu=0;
k_rep=1;
Q_star=1000;     %���ϰ���ĳ������ð뾶

% �����յ�Ե�ǰ�������  
% tips�������ֵԽС��˵����ǰ�����յ�Խ��
attr=1/2*k_att*(norm(curr-over))^2;     % �������㹫ʽ
if flag==1
    obstacle = [obstacle p];   % ��������˻�B��Ϊһ���ϰ���
elseif flag==2
    obstacle = [obstacle p];   % ��������˻�A��Ϊһ���ϰ���
end

% ���������ϰ���Ե�ǰ������� 
% tips��������ԽС��˵����ǰ�������ϰ���ĸ���ԽС
for i=1:size(obstacle,2)
    if norm(curr-obstacle(:,i))<=Q_star    % �ϰ��ﵽ��ǰ���������ֵ�ڣ����������Ӱ��
        repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/Q_star)^2*(norm(curr-over))^2;    % �������㹫ʽ
        % ps�� �� curr��obstacle �����غ�ʱ���Ƿ�����repu���㿨���� �Ƿ���Ҫ�Ը�������������
    else       % �ϰ��ﵽ��ǰ���������ֵ�⣬���Գ���Ӱ��
        repu=repu+0;
    end
end
output=-attr+repu;   % ����+����  �����ֵԽС��Խ�ʺϵ�����һ�����

 end  