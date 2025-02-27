% 人工势场法
% 作者：Ally
% 日期：2021/1/24
clc
clear
close all

%% 初始化车的参数
d = 3.5;               % 道路标准宽度
W = 1.8;               % 汽车宽度
L = 4.7;               % 车长

P0 = [0,-d/2,1,1];     % 车辆起点信息，1-2列位置，3-4列速度
Pg = [99,d/2,0,0];     % 目标位置
Pobs = [15,7/4,0,0;
    30,-3/2,0,0;
    45,3/2,0,0;
    60,-3/4,0,0;
    80,7/4,0,0];       % 障碍物位置
P = [Pobs;Pg];         % 将目标位置和障碍物位置合放在一起

Eta_att = 5;           % 计算引力的增益系数
Eta_rep_ob = 15;       % 计算斥力的增益系数
Eta_rep_edge = 50;     % 计算边界斥力的增益系数

d0 = 20;               % 障碍影响距离
n = size(P,1);         % 障碍与目标总计个数
len_step = 0.5;          % 步长
Num_iter = 200;        % 最大循环迭代次数

%% ***************初始化结束，开始主体循环******************
Pi = P0;               %将车的起始坐标赋给Xi
i = 0;
while sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) > 1
    i = i + 1;
    Path(i,:) = Pi;    % 保存车走过的每个点的坐标
    
    %计算车辆当前位置与障碍物的单位方向向量、速度向量
    for j = 1:n-1    
        delta(j,:) = Pi(1,1:2) - P(j,1:2);                              % 用车辆点-障碍点表达斥力
        dist(j,1) = norm(delta(j,:));                                   % 车辆当前位置与障碍物的距离
        unitVector(j,:) = [delta(j,1)/dist(j,1), delta(j,2)/dist(j,1)]; % 斥力的单位方向向量
    end
    
    %计算车辆当前位置与目标的单位方向向量、速度向量
    delta(n,:) = P(n,1:2)-Pi(1,1:2);                                    %用目标点-车辆点表达引力   
    dist(n,1) = norm(delta(n,:)); 
    unitVector(n,:)=[delta(n,1)/dist(n,1),delta(n,2)/dist(n,1)];

   %% 计算斥力 
    % 在原斥力势场函数增加目标调节因子（即车辆至目标距离），以使车辆到达目标点后斥力也为0
    for j = 1:n-1
        if dist(j,1) >= d0
            F_rep_ob(j,:) = [0,0];
        else
            % 障碍物的斥力1，方向由障碍物指向车辆
            F_rep_ob1_abs = Eta_rep_ob * (1/dist(j,1)-1/d0) * dist(n,1) / dist(j,1)^2;         
            F_rep_ob1 = [F_rep_ob1_abs*unitVector(j,1), F_rep_ob1_abs*unitVector(j,2)];   
           
            % 障碍物的斥力2，方向由车辆指向目标点
            F_rep_ob2_abs = 0.5 * Eta_rep_ob * (1/dist(j,1) - 1/d0)^2;                
            F_rep_ob2 = [F_rep_ob2_abs * unitVector(n,1), F_rep_ob2_abs * unitVector(n,2)];  
            
            % 改进后的障碍物合斥力计算
            F_rep_ob(j,:) = F_rep_ob1+F_rep_ob2;                                   
        end
    end
    
    
    % 增加边界斥力势场，根据车辆当前位置，选择对应的斥力函数
    if Pi(1,2) > -d+W/2 && Pi(1,2) <= -d/2             %下道路边界区域力场，方向指向y轴正向
        F_rep_edge = [0,Eta_rep_edge * norm(Pi(:,3:4))*(exp(-d/2-Pi(1,2)))];
    elseif Pi(1,2) > -d/2 && Pi(1,2) <= -W/2           %下道路分界线区域力场，方向指向y轴负向
        F_rep_edge = [0,1/3 * Eta_rep_edge * Pi(1,2).^2];
    elseif Pi(1,2) > W/2  && Pi(1,2) < d/2             %上道路分界线区域力场，方向指向y轴正向 
        F_rep_edge = [0, -1/3 * Eta_rep_edge * Pi(1,2).^2];
    elseif Pi(1,2) > d/2 && Pi(1,2)<=d-W/2             %上道路边界区域力场，方向指向y轴负向
        F_rep_edge = [0, Eta_rep_edge * norm(Pi(:,3:4)) * (exp(Pi(1,2)-d/2))];
    end
    
    %% 计算合力和方向
    F_rep = [sum(F_rep_ob(:,1))  + F_rep_edge(1,1),...
           sum(F_rep_ob(:,2)) + F_rep_edge(1,2)];                                      % 所有障碍物的合斥力矢量
    F_att = [Eta_att*dist(n,1)*unitVector(n,1), Eta_att*dist(n,1)*unitVector(n,2)];    % 引力矢量
    F_sum = [F_rep(1,1)+F_att(1,1),F_rep(1,2)+F_att(1,2)];                             % 总合力矢量
    UnitVec_Fsum(i,:) = 1/norm(F_sum) * F_sum;                                         % 总合力的单位向量
    
    %计算车的下一步位置
    Pi(1,1:2)=Pi(1,1:2)+len_step*UnitVec_Fsum(i,:);                     

%     %判断是否到达终点
%     if sqrt((Pi(1)-P(n,1))^2+(Pi(2)-P(n,2))^2) < 0.2 
%         break
%     end
end
Path(i,:)=P(n,:);            %把路径向量的最后一个点赋值为目标

%% 画图
figure
len_line = 100;



% 画灰色路面图
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %2号车

% 画分界线
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);     %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 


% 绘制路径
plot(P(1:n-1,1),P(1:n-1,2),'ro');   %障碍物位置
plot(P(n,1),P(n,2),'gv');       %目标位置
plot(P0(1,1),P0(1,2),'bs');    %起点位置
plot(Path(:,1),Path(:,2),'.b');%路径点

