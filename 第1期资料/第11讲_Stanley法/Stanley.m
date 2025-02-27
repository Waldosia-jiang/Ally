% Stanley法
% 作者：Ally
% 日期：20210429
clc
clear
close all
load  path.mat

%% 相关参数定义
% 参考轨迹
RefPos = path;            

% 计算轨迹的参考航向角
diff_x = diff(RefPos(:,1)) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(RefPos(:,2)) ;
diff_y(end+1) = diff_y(end);
RefHeading = atan2(diff_y ,diff_x);

% 其他常量参数
targetSpeed = 10;           % 目标速度，单位： m /s
InitialState = [RefPos(1,:)-0.5,RefHeading(1)+0.02,0];  % 纵向位置、横向位置、航向角、速度
k = 5;                      % 增益参数
Kp = 1;                     % 速度P控制器系数
dt = 0.1;                   % 时间间隔，单位：s
L = 2;                      % 车辆轴距，单位：m

%% 主程序
% 车辆初始状态定义
state = InitialState;
state_actual = state;
idx = 1;
latError_Stanley = [];

% 循迹
while idx < size(RefPos,1)-1
    % 寻找预瞄距离范围内最近路径点
    idx = findTargetIdx(state,RefPos);
       
    % 计算前轮转角
    [delta,latError] = stanley_control(idx,state,RefPos,RefHeading,k);
    
    % 如果误差过大，退出循迹
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 计算加速度
    a = Kp* (targetSpeed-state(4));
    
    % 更新状态量
    state_new = UpdateState(a,state,delta,dt,L);
    state = state_new;
    
    % 保存每一步的实际量
    state_actual(end+1,:) = state_new;
    latError_Stanley(end+1,:) =  [idx,latError];
end

% 画图
figure
plot(RefPos(:,1), RefPos(:,2), 'r');
xlabel('纵向坐标 / m');
ylabel('横向坐标 / m');
hold on
for i = 1:size(state_actual,1)
    scatter(state_actual(i,1), state_actual(i,2),150, 'b.');
    pause(0.01)
end
legend('规划车辆轨迹', '实际行驶轨迹')

%  保存
path_stanley = state_actual(:,1:2);
save path_stanley.mat path_stanley;
save latError_Stanley.mat latError_Stanley
%% 首先在参考轨迹上搜索离当前位置最近的点
function target_idx = findTargetIdx(state,RefPos)
for i = 1:size(RefPos,1)
    d(i,1) = norm(RefPos(i,:) - state(1:2));
end
[~,target_idx] = min(d);  % 找到距离当前位置最近的一个参考轨迹点的序号
end

%% 获得控制量
function [delta,latError] = stanley_control(idx,state,RefPos,RefHeading,k)

% 根据百度Apolo，计算横向误差
dx = state(1) - RefPos(idx,1);
dy = state(2) - RefPos(idx,2);
phi_r = RefHeading(idx);
latError = dy*cos(phi_r) - dx*sin(phi_r);

% 分别计算只考虑航向误差的theta和只考虑横向误差的theta
theta_fai =  RefHeading(idx)- state(3);
theta_y = atan2(-k*latError,state(4));

% 将两个角度合并即为前轮转角
delta = theta_fai + theta_y;
end

%% 更新状态量
function state_new = UpdateState(a,state_old,delta,dt,L)
state_new(1) =  state_old(1) + state_old(4)*cos(state_old(3))*dt; %纵向坐标
state_new(2) =  state_old(2) + state_old(4)*sin(state_old(3))*dt; %横向坐标
state_new(3) =  state_old(3) + state_old(4)*dt*tan(delta)/L;      %航向角
state_new(4) =  state_old(4) + a*dt;                              %纵向速度
end
