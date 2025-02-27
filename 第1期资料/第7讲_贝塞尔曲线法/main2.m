% 贝塞尔曲线法:基于贝塞尔曲线规划换道轨迹
% 作者：Ally
% 日期：2021/1/30
clc
clear
close all

%% 
% 定义控制点
d = 3.5;
P0 = [0, -d/2];
P1 = [25, -d/2];
P2 = [25, d/2];
P3 = [50, d/2];
P = [P0; P1; P2; P3];

% 直接根据贝塞尔曲线定义式得到路径点
n = length(P)-1;
Path = [];
for t = 0:0.01:1
    if n == 1
        p_t = P;
    elseif n >= 2
        p_t = [0, 0];
        for i = 0:n
            k_C = factorial(n) / (factorial(i) * factorial(n-i));
            k_t = (1-t)^(n-i) * t^i;
            p_t = p_t + k_C * k_t * P(i+1,:);
        end
        Path(end+1,:) = p_t;
    
    else
        disp('控制点输入有误，请重新输入')
    end
end


%% 画图
d = 3.5;               % 道路标准宽度
W = 1.8;               % 汽车宽度
L = 4.7;               % 车长
figure
len_line = 50;

% 画灰色路面图
GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
hold on
fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  

% 画分界线
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);     %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal
set(gca, 'XLim',[-5 len_line]); 
set(gca, 'YLim',[-4 4]); 

% 绘制路径
scatter(P(:,1),P(:,2),'g')
plot(P(:,1),P(:,2),'r');%路径点
scatter(Path(:,1),Path(:,2),200, '.b');%路径点

