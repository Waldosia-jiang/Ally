function [pos_follow,fitness_follow,path_follow] = followBehavior(fish,...
    fishs,pointNum,visual,startPos,goalPos,try_number,delta,step,X,Y,Z)

N = length(fishs);
fitness_follow = inf;

% 搜索人工鱼视野范围内的最高适应度个体
for i = 1:N
    dist = [];
    for j = 1:pointNum
        dx = fish.pos.x(j) - fishs(i).pos.x(j);
        dy = fish.pos.y(j) - fishs(i).pos.y(j);
        dz = fish.pos.z(j) - fishs(i).pos.z(j);
        dist(j) = sqrt(dx^2 + dy^2 + dz^2);
    end
    
    if max(dist) < visual && fishs(i).fitness < fitness_follow
        fitness_follow = fishs(i).fitness;
        pos_best = fishs(i).pos;
    end
end


% 搜索最高适应度人工鱼的视野范围内的伙伴数量
n_follow = 0;
for i = 1:N
    dist = [];
    for j = 1:pointNum
        dx = pos_best.x(j) - fishs(i).pos.x(j);
        dy = pos_best.y(j) - fishs(i).pos.y(j);
        dz = pos_best.z(j) - fishs(i).pos.z(j);
        dist(j) = sqrt(dx^2 + dy^2 + dz^2);
    end
    
    if max(dist) < visual
        n_follow = n_follow + 1;
    end
end
n_follow = n_follow - 1;   % 减掉最高适应度人工鱼自身

% 判断中心位置是否拥挤
if  fitness_follow/n_follow < delta*fish.fitness  && fitness_follow < fish.fitness
    % 群聚行为：伙伴中心位置状态较优，向此方向群聚
    for i = 1:pointNum
        direction = [pos_best.x(i)-fish.pos.x(i), pos_best.y(i)-fish.pos.y(i), ...
            pos_best.z(i)-fish.pos.z(i)];
        temp = [fish.pos.x(i),fish.pos.y(i),fish.pos.z(i)] +...
            rand*step*direction/norm(direction);
        pos_follow.x(i) = temp(1);
        pos_follow.y(i) = temp(2);
        pos_follow.z(i) = temp(3);
    end
    
    % 计算适应度
    [fitness_follow,path_follow] = calFitness(startPos, goalPos,pos_follow);
    
    % 碰撞检测判断
    flag = judgeObs(path_follow,X,Y,Z);
    if flag == 1
        % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
        fitness_follow = 1000*fitness_follow;
    end
else
    
    % 觅食行为
    label_prey = 0;             % 判断觅食行为是否找到优于当前的状态
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
                temp = [fish.pos.x(j),fish.pos.y(j),fish.pos.z(j)] + ...
                    rand*step*direction/norm(direction);
                pos_follow.x(j) = temp(1);
                pos_follow.y(j) = temp(2);
                pos_follow.z(j) = temp(3);
            end
            % 计算适应度
            [fitness_follow,path_follow] = calFitness(startPos, goalPos,pos_follow);
            
            % 碰撞检测判断
            flag = judgeObs(path_follow,X,Y,Z);
            if flag == 1
                % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
                fitness_follow = 1000*fitness_follow;
            end
            label_prey = 1;
            break;
        end
    end
    
    % 随机行为
    if label_prey==0
        % 随机搜索一个状态
        pos_follow.x = fish.pos.x + step *(-1 + rand*2);
        pos_follow.y = fish.pos.y + step *(-1 + rand*2);
        pos_follow.z = fish.pos.z + step *(-1 + rand*2);
        [fitness_follow,path_follow] = calFitness(startPos, goalPos,pos_follow);
        
        % 碰撞检测判断
        flag = judgeObs(path_follow,X,Y,Z);
        if flag == 1
            % 若flag=1，表明此路径将与障碍物相交，则增大适应度值
            fitness_follow = 1000*fitness_follow;
        end
    end
end



