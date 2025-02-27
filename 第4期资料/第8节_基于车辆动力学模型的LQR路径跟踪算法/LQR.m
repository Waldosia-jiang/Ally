clc
clear
close all
load  refPath.mat

%% 相关参数定义
dt = 0.1;
L = 2.9;
Q = 1*eye(4);
R = 1*eye(1);

%% 轨迹处理
% 定义参考轨迹
refPos = refPath_line;
refSpeed = 10;    % 参考速度

% 计算航向角
diff_x = diff(refPos(:,1)) ;
diff_y = diff(refPos(:,2)) ;
cumLength = cumsum(sqrt(diff_x.^2 + diff_y.^2));
refHeading = atan2(diff_y , diff_x);                   % 航向角
refHeading(end+1) = refHeading(end);

% 曲率
for i = 1:size(refPos,1)-2
    x = refPos(i:i+2,1);
    y = refPos(i:i+2,2);
    refCur(i) = abs(getCur(x,y));
end
refCur(i+1) = refCur(i);
refCur(i+2) = refCur(i);

% 根据阿克曼转向原理，计算参考前轮转角
refDelta = atan(L*refCur);

% 车辆初始状态定义
iniPos = refPos(1,:);
iniSpd = 1;
iniHeading = refHeading(1);
%% 主程序
% 当前车辆信息
currentPos = iniPos;
currentSpd = iniSpd;
currentHeading = iniHeading;

% 赋初值
Delta = 0;

% 轨迹跟踪实际量
posSet = currentPos;
headingSet = currentHeading;
spdSet = currentSpd;
latErrSet = [];

latError_last = 0;
yaw_error_last = 0;
omega = 0;
actualCur = 0.0001;
targetIdx = 1;
% 循环
while true
     % 寻找预瞄距离范围内最近路径点
     if size(refPos,1) - targetIdx> 20
         dist = sqrt((refPos(targetIdx:targetIdx+20,1)-currentPos(1)).^2 + ...
             (refPos(targetIdx:targetIdx+20,2)-currentPos(2)).^2);
     else
         dist = sqrt((refPos(targetIdx:end,1)-currentPos(1)).^2 + ...
             (refPos(targetIdx:end,2)-currentPos(2)).^2);
     end
     [~,targetIdx_temp] = min(dist);
     targetIdx = targetIdx + targetIdx_temp - 1;

    
    % 判断是否超出索引
    if targetIdx >= size(refPos,1)
        break
    end
    
    % 速度P控制器
    a = 0.8* (refSpeed-currentSpd)/dt;
    
    % LQR控制器
    [Delta,delta_r,latError,yaw_error] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refSpeed,refCur,L,Q,R,dt,latError_last,yaw_error_last,actualCur,omega);    
   
    % 如果误差过大，退出循迹
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 更新状态
    [currentPos,currentHeading,currentSpd,omega] = update(currentPos,currentHeading,currentSpd, a,Delta, dt,L);
    
    % 保存每一步的实际量
    posSet(end+1,:) = currentPos;
    headingSet(end+1,:) = currentHeading;
    spdSet(end+1,:) = currentSpd;
    latErrSet(end+1,:) = [cumLength(targetIdx),latError];
    
    %
    latError_last = latError;
    yaw_error_last = yaw_error;
    
    % 计算实际路径曲率
    if size(posSet,1) >= 3
        actualCur = getCur(posSet(end-2:end,1), posSet(end-2:end,2));
    else
        actualCur = 0.00001;
    end

end
latErr_mean = mean(abs(latErrSet(:,2)));

%%  画图
% 画路径跟踪图
figure
hold on
grid on
plot(refPos(:,1), refPos(:,2),  'Color', 'b');
scatter(posSet(:,1),posSet(:,2),10,'MarkerEdgeColor','r',...
    'MarkerFaceColor','r')   % 坐标离散点
plot(posSet(:,1), posSet(:,2),  'Color', 'r');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
legend('规划车辆轨迹', '实际行驶轨迹');


% 画误差图
figure
hold on
grid on
plot(latErrSet(1:end,1), latErrSet(1:end,2),'LineWidth', 3,  'Color', 'b');
plot([0,latErrSet(end,1)], [latErr_mean,latErr_mean], 'r:','LineWidth', 3);
xlabel('距离 / m');
ylabel('横向误差 / m');
legend('横向误差', '平均横向误差');

%% LQR控制
function [Delta,delta_r,latError,yaw_error] =  LQR_controller(targetIdx,currentPos,currentSpd,currentHeading,refPos,refHeading,refDelta,refSpd,refCur,L,Q,R,dt,latError_last,yaw_error_last,actualCur,omega)
% 车辆参数
m = 1723;          % 质量
lf = 1.232;        % 质心到前轴的距离
lr = 1.468;        % 质心到后轴的距离
C_af = -66900;     % 前轮侧偏刚度
C_ar = -62700;     % 后轮侧偏刚度
Iz = 4175;         % 饶Z轴的转动惯量

% 求位置、航向角参考量
x_r = refPos(targetIdx,1);
y_r = refPos(targetIdx,2);
heading_r = refHeading(targetIdx);
delta_r = refDelta(targetIdx);

% 求位置、航向角的误差
x_error  = currentPos(1) - x_r;
y_error = currentPos(2) - y_r;
yaw_error =  currentHeading - heading_r;

% 根据百度Apolo，计算横向误差
latError = y_error*cos(heading_r) - x_error*sin(heading_r);

% 计算变化率
latError_dot = currentSpd * sin(yaw_error); 
yawError_dot = omega - currentSpd*refCur(targetIdx)*cos(yaw_error)/(1+refCur(targetIdx)*latError);

% 将误差值赋值到状态量
X(1,1) = latError; 
X(2,1) = latError_dot;  
X(3,1) = yaw_error;
X(4,1) = yawError_dot;

% 由状态方程矩阵系数，计算K
A = [0,  1,  0, 0;
    0,  2*(C_af + C_ar)/(m*currentSpd), -2*(C_af + C_ar)/m, 2*(lf*C_af - lr*C_ar)/(m*currentSpd);
     0,  0,  0, 1;
     0,  2*(lf*C_af - lr*C_ar)/(Iz*currentSpd), -2*(lf*C_af - lr*C_ar)/Iz, 2*(lf^2*C_af + lr^2*C_ar)/(Iz*currentSpd)];
B = [0; -2*C_af/m; 0; -2*lf*C_af/Iz];

K = lqr(A,B,Q,R);

% 获得控制量
u = -K * X; 
Delta = max(u,-0.44);
Delta = min(Delta,0.44);

end

%% 更新状态
function [pos_new, currentHeading_new, currentSpd, omega] = update(currentPos,currentHeading,currentSpd, a,Delta, dt,L)
currentSpd = currentSpd + a*dt;
omega =  currentSpd / L * tan(Delta);
currentHeading_new = currentHeading + currentSpd / L * tan(Delta) * dt;
pos_new(1) = currentPos(1) + currentSpd*cos(currentHeading_new)*dt;
pos_new(2) =  currentPos(2) + currentSpd*sin(currentHeading_new)*dt;
end
