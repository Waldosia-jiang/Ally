% 基于三次样条曲线的换道路径规划
clc
clear
close all

%% 数据定义
d = 3.5;
k = 4;                                   
P = [0,-d/2; 10,-d/2; 20,-d/2; 30,d/2; 40,d/2; 50,d/2]';

%% 调用pchip函数，生成三次样条曲线
x_seq = P(1,:);
y_seq = P(2,:);
cs = pchip(x_seq,y_seq);
X_seq = linspace(0,50,100);
Y_seq = ppval(cs,X_seq);
path = [X_seq', Y_seq'];

%% 计算长度和曲率
x = path(:,1)';
y = path(:,2)';
diffX = diff(path(:,1));
diffY = diff(path(:,2));
cumLength = cumsum(sqrt(diffX.^2 + diffY.^2));   %长度
heading = atan2(diffY, diffX);
for i = 1:length(x)-2
    cur(i) = getCur(x(i:i+2)',y(i:i+2)');
end
cur(end+1) = cur(end);



%% 画曲率图
figure
hold on
grid on
% 主体图形绘制
plot(cumLength,cur,'LineWidth', 3,  'Color', 'k');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('路径长度/m');
hYLabel = ylabel('曲率/m^-^1');
% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
    set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)


% 画航向角图
figure
hold on
grid on
% 主体图形绘制
plot(cumLength, heading,'LineWidth', 3,  'Color', 'b');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('路径长度/m');
hYLabel = ylabel('航向角/rad');
% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)


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
fill([P(1,1),P(1,1),P(1,1)-L,P(1,1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'y')
fill([35,35,35-L,35-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')

% 画分界线
plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %分界线
plot([-5,len_line],[d,d],'w','linewidth',2);     %左边界线
plot([-5,len_line],[-d,-d],'w','linewidth',2);  %左边界线

% 设置坐标轴显示范围
axis equal
set(gca, 'XLim',[-5 len_line]);
set(gca, 'YLim',[-4 4]);

% 绘制路径
scatter(P(1,:),P(2,:),100,'r.')
plot(P(1,:),P(2,:),'r');%路径点
plot(path(:,1),path(:,2), 'y','linewidth',2);%路径点

