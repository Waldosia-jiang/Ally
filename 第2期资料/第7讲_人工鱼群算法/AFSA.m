% 第7讲：人工鱼群算法
% 作者： Ally
% 日期： 2021/10/16
clc
clear
close all

%% 三维路径规划模型定义
startPos = [1, 1, 1];
goalPos = [100, 100, 80];

% 随机定义山峰地图
mapRange = [100,100,100];              % 地图长、宽、高范围
[X,Y,Z] = defMap(mapRange);

%% 初始参数设置
N = 50;                % 鱼群数量
visual = 50;           % 人工鱼的感知距离
step = 3;              % 人工鱼的移动最大步长
delta = 10;            % 拥挤度因子
try_number = 50;       % 单只鱼的最大迭代次数
iterMax = 100;         % 迭代次数
pointNum = 3;          % 解的维度 

% 位置界限
posBound = [[0,0,0]',mapRange'];

%% 种群初始化
% 初始化一个空的鱼群结构体
fishes = struct;
fishes.pos= [];
fishes.fitness = [];
fishes.path = [];
fishes = repmat(fishes,N,1);

% 初始化最优人工鱼
GlobalBest.fitness = inf;

% 鱼群随机分布位置
for i = 1:N
    % 鱼群按照正态分布随机生成
    fishes(i).pos.x = unifrnd(posBound(1,1),posBound(1,2),1,pointNum);
    fishes(i).pos.y = unifrnd(posBound(2,1),posBound(2,2),1,pointNum);
    fishes(i).pos.z = unifrnd(posBound(3,1),posBound(3,2),1,pointNum);
    
    % 适应度
    [fitness,path] = calFitness(startPos, goalPos, fishes(i).pos);
    
    % 碰撞检测判断
    flag = judgeObs(path,X,Y,Z);
    if flag == 1
        % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
        fishes(i).fitness = 1000*fitness;
        fishes(i).path = path;
    else
        % 否则，表明可以选择此路径
        fishes(i).fitness = fitness;
        fishes(i).path = path;
    end

    % 更新个体人工鱼的最优
    fishes(i).Best.pos = fishes(i).pos;
    fishes(i).Best.fitness = fishes(i).fitness;
    fishes(i).Best.path = fishes(i).path;
    
    % 更新全局最优
    if fishes(i).Best.fitness < GlobalBest.fitness
        GlobalBest = fishes(i).Best;
    end
end

% 初始化每一代的最优适应度，用于画适应度迭代图
fitness_beat_iters = zeros(iterMax,1);


%% 循环
for iter = 1:iterMax
    for i = 1:N
        
        fish = fishes(i);
        % 群聚行为
        [pos_swarm,fitness_swarm,path_swarm] =  swarmBehavior(fish,fishes,...
            pointNum,visual,startPos,goalPos,try_number,delta,step,X,Y,Z);
        
        % 追尾行为
        [pos_follow,fitness_follow,path_follow] = followBehavior(fish,...
            fishes,pointNum,visual,startPos,goalPos,try_number,delta,step,X,Y,Z);

         % 两种行为找最优
        if fitness_swarm < fitness_follow
            fishes(i).pos = pos_swarm;
            fishes(i).path = path_swarm;
            fishes(i).fitness = fitness_swarm;
        else
            fishes(i).pos = pos_follow;
            fishes(i).path = path_follow;
            fishes(i).fitness = fitness_follow;
        end
        
        % 更新全局最优人工鱼
        if fishes(i).fitness < GlobalBest.fitness
            GlobalBest = fishes(i);
        end
        
    end
    
    % 把每一代的最优粒子赋值给fitness_beat_iters
    fitness_beat_iters(iter) = GlobalBest.fitness;
    
    % 在命令行窗口显示每一代的信息
    disp(['第' num2str(iter) '代:' '最优适应度 = ' num2str(fitness_beat_iters(iter))]);
    
    % 画图
    plotFigure(startPos,goalPos,X,Y,Z,GlobalBest);
    pause(0.001);
end
        
%% 结果展示
% 理论最小适应度：直线距离
fitness_best = norm(startPos - goalPos);
disp([ '理论最优适应度 = ' num2str(fitness_best)]);

% 画适应度迭代图
figure
plot(fitness_beat_iters,'LineWidth',2);
xlabel('迭代次数');
ylabel('最优适应度');