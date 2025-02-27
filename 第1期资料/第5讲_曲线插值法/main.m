% 曲线插值法
% 作者：Ally
% 日期：2021/1/16
clc
clear
close all

%% 场景定义
% 换道场景路段与车辆相关参数的定义
d = 3.5;          % 道路标准宽度
len_line = 30;    % 直线段长度
W = 1.75;         % 车宽
L = 4.7;          % 车长
x1 = 20;          %1号车x坐标

% 车辆换道初始状态与终点期望状态
t0 = 0;
t1 = 3;
state_t0 = [0, -d/2; 5, 0; 0, 0];  % x,y; vx,vy; ax,ay
state_t1 = [20, d/2; 5, 0; 0, 0];
x2 = state_t0(1);

%% 画场景示意图
figure(1)
% 画灰色路面图
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on

% 画小车
fill([x1,x1,x1+L,x1+L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  %1号车
fill([x2,x2,x2-L,x2-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')  %2号车

% 画分界线
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);  %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

%% 五次多项式轨迹生成

% 计算A和B两个系数矩阵
X = [state_t0(:,1); state_t1(:,1)];
Y = [state_t0(:,2); state_t1(:,2)];
T = [ t0^5      t0^4      t0^3     t0^2    t0   1;
      5*t0^4    4*t0^3    3*t0^2   2*t0    1    0;
      20*t0^3   12*t0^2   6*t0     1       0    0;
      t1^5      t1^4      t1^3     t1^2    t1   1;
      5*t1^4    4*t1^3    3*t1^2   2*t1    1    0;
      20*t1^3   12*t1^2   6*t1     1       0    0];
A = T \ X;
B = T \ Y;

% 将时间从t0到t1离散化，获得离散时刻的轨迹坐标
t=(t0:0.05:t1)';
path=zeros(length(t),4);%1-4列分别存放x,y,vx,vy 
for i = 1:length(t)
    % 纵向位置坐标
    path(i,1) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * A;
    
    % 横向位置坐标
    path(i,2) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * B;
    
    % 纵向速度
    path(i,3) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * A;
    
    % 横向速度
    path(i,4) = [5*t(i)^4,  4*t(i)^3,  3*t(i)^2,  2*t(i), 1, 0] * B;
end

% 画换道轨迹
plot(path(:,1),path(:,2),'r--','linewidth',1.5); 

%% 分析速度

% 横向速度
figure 
plot(t, path(:,4), 'k'); 
xlabel('时间 / s ');
ylabel('横向速度 / m/s ');

% 纵向速度
figure 
plot(t, path(:,3), 'k'); 
xlabel('时间 / s ');
ylabel('纵向速度 / m/s ');

