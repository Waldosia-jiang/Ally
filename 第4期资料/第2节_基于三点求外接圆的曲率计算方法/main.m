% 第2节_基于三点求外接圆的曲率计算方法
% 作者：Ally
% 日期：20220828
clc
clear
close all
load  refPath.mat
refPath = refPath_circle;

%% 基于三点求外接圆的曲率计算方法
for i = 1:size(refPath,1)-2
    A = refPath(i,:);
    B = refPath(i+1,:);
    C = refPath(i+2,:);
    a = norm(C-B);
    b = norm(C-A);
    c = norm(A-B);
    theta_B = acos((a^2 + c^2 - b^2) / (2*a*c));
    cur(i) = 2*sin(theta_B) / b;
end

%% 计算路径长度
diff_x = diff(refPath(:,1)) ;
diff_y = diff(refPath(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));

%% 画图
% 图1：参考曲线
figure
grid on
plot(refPath(:,1), refPath(:,2),'LineWidth', 3,  'Color', 'b');

% 图2：参考曲线的曲率
figure
hold on
grid on
plot(cumLength(1:end-1), cur,'LineWidth', 3,  'Color', 'b');
