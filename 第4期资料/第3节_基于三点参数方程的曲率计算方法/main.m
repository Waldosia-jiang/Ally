% 第3节_基于三点参数方程的曲率计算方法
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
    cur1(i) = 2*sin(theta_B) / b;
end

%% 基于三点参数方程的曲率计算方法
for i = 1:size(refPath,1)-2
    x = refPath(i:i+2,1);
    y = refPath(i:i+2,2);    
    ta = sqrt((x(2)-x(1))^2+(y(2)-y(1))^2);
    tb = sqrt((x(3)-x(2))^2+(y(3)-y(2))^2);
    M = [1,-ta,ta^2;
        1,0,0;
        1,tb,tb^2];
    A = M\x;
    B = M\y;
    cur2(i) = abs(2*(A(2)*B(3)-A(3)*B(2)))/((A(2)^2+B(2)^2)^1.5+1e-10);
end

%% 计算路径长度
diff_x = diff(refPath(:,1)) ;
diff_y = diff(refPath(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));

%% 画图比较
% 图1：参考曲线
figure
grid on
plot(refPath(:,1), refPath(:,2),'LineWidth', 3,  'Color', 'b');

% 图2：两种曲率比较
figure
hold on
grid on
plot(cumLength(1:end-1), cur1,'LineWidth', 3,  'Color', 'b');
plot(cumLength(1:end-1), cur2,'r--','LineWidth', 3 );

% 图3：两种曲率差值
figure
hold on
grid on
% 主体图形绘制
plot(cumLength(1:end-1), cur1-cur2,'LineWidth', 3,  'Color', 'b');
