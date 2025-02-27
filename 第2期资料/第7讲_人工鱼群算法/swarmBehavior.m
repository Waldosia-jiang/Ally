function [pos_swarm,fitness_swarm,path_swarm] = swarmBehavior(fish,fishs,...
    pointNum,visual,startPos,goalPos,try_number,delta,step,X,Y,Z)

N = length(fishs);
% 确定视野范围内的伙伴数目与中心位置
for i = 1:pointNum
    % 初始化感知范围内的位置变量和鱼数量
    pos_sum = [0 0 0];
    n_swarm = 0;
    
    % 依次遍历
    for j = 1:N
        % 计算第j只鱼与本鱼的距离
        dx = fish.pos.x(i) - fishs(j).pos.x(i);
        dy = fish.pos.y(i) - fishs(j).pos.y(i);
        dz = fish.pos.z(i) - fishs(j).pos.z(i);
        dist = sqrt(dx^2 + dy^2 + dz^2);
        
        % 判断是否位于感知范围内
        if dist < visual
            n_swarm = n_swarm+1;  % 统计在感知范围内的鱼数量
            pos_sum = pos_sum + ...
                [fishs(j).pos.x(i), fishs(j).pos.y(i), fishs(j).pos.z(i)];
        end
    end
    
    % 计算其他伙伴的中心位置，并减掉自身的位置
    pos_sum = pos_sum -[fish.pos.x(i),fish.pos.y(i),fish.pos.z(i)];
    n_swarm = n_swarm - 1;
    centerPos = pos_sum / n_swarm;
    
    % 对于第i个维度的控制点，赋值
    pos_c.x(i) = centerPos(1);
    pos_c.y(i) = centerPos(2);
    pos_c.z(i) = centerPos(3);
end

% 判断中心位置是否拥挤
[fitness_c,~] = calFitness(startPos, goalPos, pos_c);
if  fitness_c/n_swarm < delta*fish.fitness  && fitness_c < fish.fitness
    % 群聚行为：伙伴中心位置状态较优，向此方向群聚
    for i = 1:pointNum
        direction = [pos_c.x(i)-fish.pos.x(i), pos_c.y(i)-fish.pos.y(i), ...
            pos_c.z(i)-fish.pos.z(i)];
        centerPos = [fish.pos.x(i),fish.pos.y(i),fish.pos.z(i)] + ...
            rand*step*direction/norm(direction);
        pos_swarm.x(i) = centerPos(1);
        pos_swarm.y(i) = centerPos(2);
        pos_swarm.z(i) = centerPos(3);
    end
    
    % 计算适应度
    [fitness_swarm,path_swarm] = calFitness(startPos, goalPos,pos_swarm);
    
    % 碰撞检测判断
    flag = judgeObs(path_swarm,X,Y,Z);
    if flag == 1
        % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
        fitness_swarm = 1000*fitness_swarm;
    end
else
    
    % 觅食行为
    label_prey = 0;             % 用于判断觅食行为是否找到优于当前的状态
    for i = 1:try_number
        % 随机搜索一个状态
        pos_rand.x = fish.pos.x + visual*(-1 + rand*2);
        pos_rand.y = fish.pos.y + visual*(-1 + rand*2);
        pos_rand.z = fish.pos.z + visual*(-1 + rand*2);
        [fitness_rand,~] = calFitness(startPos, goalPos, pos_rand);
        
        % 判断搜索到的状态是否比原来的好
        if fitness_rand < fish.fitness
            for j = 1:pointNum
                direction = [pos_rand.x(j) - fish.pos.x(j), pos_rand.y(j)-fish.pos.y(j), ...
                    pos_rand.z(j) - fish.pos.z(j)];
                centerPos = [fish.pos.x(j),fish.pos.y(j),fish.pos.z(j)] + ...
                    rand*step*direction/norm(direction);
                pos_swarm.x(j) = centerPos(1);
                pos_swarm.y(j) = centerPos(2);
                pos_swarm.z(j) = centerPos(3);
            end
            % 计算适应度
            [fitness_swarm,path_swarm] = calFitness(startPos, goalPos,pos_swarm);
            
            % 碰撞检测判断
            flag = judgeObs(path_swarm,X,Y,Z);
            if flag == 1
                % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
                fitness_swarm = 1000*fitness_swarm;
            end
            label_prey = 1;
            break
        end
    end
    
    % 随机行为
    if label_prey == 0
        % 随机搜索一个状态
        pos_swarm.x = fish.pos.x + step *(-1 + rand*2);
        pos_swarm.y = fish.pos.y + step *(-1 + rand*2);
        pos_swarm.z = fish.pos.z + step *(-1 + rand*2);
        [fitness_swarm,path_swarm] = calFitness(startPos, goalPos,pos_swarm);
        
        % 碰撞检测判断
        flag = judgeObs(path_swarm,X,Y,Z);
        if flag == 1
            % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
            fitness_swarm = 1000*fitness_swarm;
        end
    end
end

