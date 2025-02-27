% 举例展示龙格库塔现象
clc
clear
close all

%% 举例展示龙格库塔现象
x = -1:0.1:1;
X = -1:0.01:1;
y = 1./(1+25*x.^2);
pp = polyfit(x,y,5);
Y_fit5 = polyval(pp,X);
pp = polyfit(x,y,9);
Y_fit9 = polyval(pp,X);
pp = polyfit(x,y,14);
Y_fit14 = polyval(pp,X);
pp = polyfit(x,y,16);
Y_fit16 = polyval(pp,X);

%%
figure
hold on
grid on
% 主体图形绘制

plot(X, Y_fit5,'LineWidth', 1.5,  'Color', 'B');
plot(X, Y_fit9,'LineWidth', 1.5,  'Color', 'G');
plot(X, Y_fit14,'LineWidth', 1.5,  'Color', 'C');
plot(X, Y_fit16,'LineWidth', 1.5,  'Color', 'R');
scatter(x, y,40,'MarkerEdgeColor','k',...
    'MarkerFaceColor','k');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('x坐标');
hYLabel = ylabel('y坐标');
hLegend = legend('5次多项式','9次多项式','14次多项式','16次多项式','龙格函数散点');

% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
set([hXLabel, hYLabel,hLegend], 'FontName',  'simsun')
set([hXLabel, hYLabel,hLegend], 'FontSize', 16)


