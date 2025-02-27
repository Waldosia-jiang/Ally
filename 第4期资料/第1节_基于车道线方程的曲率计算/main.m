% 第1节_基于车道线方程的曲率计算
% 作者：Ally
% 日期：20220828
clc
clear
close all

%% 构建道路场景
% 场景初始化
scenario = drivingScenario;   % 初始化场景
scenario.SampleTime = 0.1;    % 采样时间

% 道路中心坐标    
roadCenters = [0 0 0; 2 0 0; 4  0 0 ;30 -15 0; 60 -40 0];  

% 建立道路
marking = [laneMarking('Solid') ...
    laneMarking('Dashed') laneMarking('Dashed') laneMarking('Solid')];             % 分界线线型
laneSpecification = lanespec(3, 'Marking', marking);         % 道路规范
road(scenario, roadCenters, 'Lanes', laneSpecification);     % 生成道路

% 获得道路边界,先把边界点位置重新整理次序
rdbdy = roadBoundaries(scenario);
rdbdy = rdbdy{1,1}(1:end-1,:);
rdbdy = [rdbdy(3:end,:); rdbdy(1:2,:)];
rdbdy(1,3) = 0;

%% 构建车辆信息
% 分别三条车道的中心线坐标，以生成车辆航迹点
ptNums = size(rdbdy,1);
for j = 1:ptNums/2
    rightBdyPt = rdbdy(j,:);
    leftBdyPt = rdbdy(ptNums-j+1,:); 
    leftWaypoints(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*1/6;
    centerWaypoints(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*3/6;
    rightWaypoints(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*5/6;
    leftLaneLine(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*1/3;
    rightLaneLine(j,:) = leftBdyPt + (rightBdyPt - leftBdyPt)*2/3;
end

% 本车信息
ego.vehicle = [];
ego.waypoints = centerWaypoints;
ego.pos = ego.waypoints(1,:);
ego.speed = 15;

% 1号交通车信息
obs = struct;
obs(1).vehicle = [];
obs(1).waypoints = leftWaypoints(30:end,:);
obs(1).pos = obs(1).waypoints(1,:);
obs(1).speed = 0.001;

% 2号交通车信息
obs(2).vehicle = [];
obs(2).waypoints = centerWaypoints(15:end,:);
obs(2).pos = obs(2).waypoints(1,:);
obs(2).speed = 10;

% 2号交通车信息
obs(3).vehicle = [];
obs(3).waypoints = rightWaypoints(10:end,:);
obs(3).pos = obs(3).waypoints(1,:);
obs(3).speed = 8;

%% 在场景增加车辆
% 根据自车信息增加车辆
ego.vehicle = vehicle(scenario,'Position',ego.pos,'PlotColor','b');
trajectory(ego.vehicle, ego.waypoints,ego.speed);

% 根据交通车信息增加车辆
obs(1).vehicle = vehicle(scenario,'Position',obs(1).pos,'PlotColor','k');
trajectory(obs(1).vehicle, obs(1).waypoints,obs(1).speed);
obs(2).vehicle = vehicle(scenario,'Position',obs(2).pos,'PlotColor','g');
trajectory(obs(2).vehicle, obs(2).waypoints,obs(2).speed);
obs(3).vehicle = vehicle(scenario,'Position',obs(3).pos,'PlotColor','g');
trajectory(obs(3).vehicle, obs(3).waypoints,obs(3).speed);

%% 启动仿真
plot(scenario)
tjty = [];
step = 1;
while advance(scenario)
    pause(0.05)
    tjty(step).time = scenario.SimulationTime;
    tjty(step).pos = scenario.Actors(1, 3).Position(1:2);
    step = step + 1;
end
grid off
axis off
xlim([-5,70])
ylim([-50,10])
hold on

%% 利用三次多项式对车道线进行拟合
% 拟合得到双侧车道线和车道中心线的三次多项式系数
poly_left = polyfit(leftLaneLine(:,1),leftLaneLine(:,2),3);
f_left = polyval(poly_left,leftLaneLine(:,1));
poly_right = polyfit(rightLaneLine(:,1),rightLaneLine(:,2),3);
f_right = polyval(poly_right,rightLaneLine(:,1));
poly_center = polyfit(centerWaypoints(:,1),centerWaypoints(:,2),3);
f_center = polyval(poly_center,centerWaypoints(:,1));

% 画拟合结果图
plot(leftLaneLine(:,1),f_left,'b--')
plot(rightLaneLine(:,1),f_right,'b--')
plot(centerWaypoints(:,1),f_center,'b','linewidth',2)