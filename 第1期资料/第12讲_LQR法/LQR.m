% LQR法
% 作者：Ally
% 日期：20210429
clc
clear
close all
load  path.mat

%% 相关参数定义
dt = 0.1;
L = 2.9 ;
Q = 100*eye(3);
R = eye(2)* 2;

%% 轨迹处理
% 定义参考轨迹
refPos_x = path(:,1);
refPos_y = path(:,2);
refPos = [refPos_x, refPos_y];

% 计算航向角和曲率
diff_x = diff(refPos_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(refPos_y) ;
diff_y(end+1) = diff_y(end);
derivative1 = gradient(refPos_y) ./ abs(diff_x);              % 一阶导数
derivative2 = del2(refPos_y) ./ abs(diff_x);                  % 二阶导数
refHeading = atan2(diff_y , diff_x);                   % 航向角
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率

% 根据阿克曼转向原理，计算参考前轮转角
refPos_Delta = atan(L*refK);

% 参考速度
refSpeed = 40/3.6;

%% 主程序
% 赋初值
x = refPos_x(1)+0.5; 
y = refPos_y(1)+0.5; 
yaw = refHeading(1)+0.02;
v = 10;
Delta = 0;
idx = 1;

% 轨迹跟踪实际量
pos_actual = [x,y];
v_actual  = v;
Delta_actual = Delta;
idx_actual = 1;
latError_LQR = [];

% 循环
while idx < length(refPos_x)-1
    % 寻找参考轨迹最近目标点
    idx = calc_target_index(x,y,refPos_x,refPos_y);  
    
    % LQR控制器
    [v_delta,delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,refPos_x,refPos_y,refHeading,refPos_Delta,refK,L,Q,R,dt);    
    
    % 如果误差过大，退出循迹
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 更新状态
    [x,y,yaw,v,Delta] = update(x,y,yaw,v, v_delta,delta, dt,L, refSpeed,delta_r);
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    v_actual(end+1,:)  = v;
    Delta_actual(end+1)  = Delta;
    idx_actual(end+1) = idx;
    latError_LQR(end+1,:) =  [idx,latError];
end

% 画图
figure
plot(refPos_x,refPos_y,'r')
hold on
for i = 1:size(pos_actual,1)
    scatter(pos_actual(i,1), pos_actual(i,2),150,'b.')
    pause(0.01);
end

% 保存
path_LQR = pos_actual;
save path_LQR.mat path_LQR
save latError_LQR.mat latError_LQR

%% 寻找参考轨迹最近目标点
function target_idx = calc_target_index(pos_x,pos_y, refPos_x,refPos_y)
i = 1:length(refPos_x)-1;
dist = sqrt((refPos_x(i)-pos_x).^2 + (refPos_y(i)-pos_y).^2);
[~, target_idx] = min(dist);
end


%% LQR控制
function [v_delta,Delta_delta,delta_r,latError] =  LQR_control(idx,x,y,v,yaw,refPos_x,refPos_y,refPos_yaw,refPos_Delta,refPos_k,L,Q,R,dt)
% 求位置、航向角参考量
x_r = refPos_x(idx);
y_r = refPos_y(idx);
heading_r = refPos_yaw(idx);
delta_r = refPos_Delta(idx);

% 求位置、航向角的误差
x_error  = x - x_r;
y_error = y - y_r;
yaw_error =  yaw - heading_r;

% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% 将误差值赋值到状态量
X(1,1) = x_error; 
X(2,1) = y_error;  
X(3,1) = yaw_error;

% 由状态方程矩阵系数，计算K
A = [1,  0,  -v*dt*sin(heading_r);
     0,  1,  v * dt * cos(heading_r);
     0,  0,  1];
B = [dt * cos(heading_r),    0;
     dt * sin(heading_r),    0;
     dt * tan(heading_r)/L,  v*dt/(L * cos(delta_r)^2)];


K = calcu_K(A,B,Q,R);

% 获得速度误差量、前轮转角误差量两个控制量
u = -K * X;  % 2行1列
v_delta = u(1);      
Delta_delta = u(2);

end


%% 计算增益
function K = calcu_K (A,B,Q,R)

% 终止条件定义
iter_max = 500;
epsilon = 0.01;

% 循环
P_old = Q;
for i = 1:iter_max
    P_new = A' * P_old * A - (A' * P_old * B) / (R + B' * P_old * B) *( B' * P_old * A) +Q;
    if abs(P_new - P_old) <= epsilon
        break
    else
        P_old = P_new; 
    end
end

P = P_new;
K = (B' * P * B + R) \ (B' * P * A);  % 2行3列
end

%% 更新状态
function [x, y, yaw, v, Delta] = update(x, y, yaw, v, v_delta,Delta_delta,dt,L,refSpeed,refDelta)
Delta = refDelta + Delta_delta;
x = x + v * cos(yaw) * dt;
y = y + v * sin(yaw) * dt;
yaw = yaw + v / L * tan(Delta) * dt;
v = refSpeed + v_delta;
end
