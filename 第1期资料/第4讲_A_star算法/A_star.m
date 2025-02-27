% A*算法
% 作者：Ally
% 日期：2021/1/9
clc
clear
close all

%% 画地图

% 栅格地图的行数、列数定义
m = 5;
n = 7;
start_node = [2, 3];
target_node = [6, 3];
obs = [4,2; 4,3; 4,4];

for i = 1:m
    plot([0,n], [i, i], 'k');
    hold on
end
    
for j = 1:n
     plot([j, j], [0, m], 'k');
end

axis equal
xlim([0, n]);
ylim([0, m]);   

% 绘制障碍物、起止点颜色块
fill([start_node(1)-1, start_node(1), start_node(1), start_node(1)-1],...
    [start_node(2)-1, start_node(2)-1 , start_node(2), start_node(2)], 'g');

fill([target_node(1)-1, target_node(1), target_node(1), target_node(1)-1],...
    [target_node(2)-1, target_node(2)-1 , target_node(2), target_node(2)], 'r');

for i = 1:size(obs,1)
    temp = obs(i,:);
    fill([temp(1)-1, temp(1), temp(1), temp(1)-1],...
        [temp(2)-1, temp(2)-1 , temp(2), temp(2)], 'b');
end

%% 预处理

% 初始化closeList
closeList = start_node;
closeList_path = {start_node,start_node};
closeList_cost = 0;
child_nodes = child_nodes_cal(start_node,  m, n, obs, closeList);  

% 初始化openList
openList = child_nodes;
for i = 1:size(openList,1)
    openList_path{i,1} = openList(i,:);
    openList_path{i,2} = [start_node;openList(i,:)];
end

for i = 1:size(openList, 1)
    g = norm(start_node - openList(i,1:2));
    h = abs(target_node(1) - openList(i,1)) + abs(target_node(2) - openList(i,2));
    f = g + h;
    openList_cost(i,:) = [g, h, f];
end

%% 开始搜索
% 从openList开始搜索移动代价最小的节点
[~, min_idx] = min(openList_cost(:,3));
parent_node = openList(min_idx,:);


%% 进入循环
flag = 1;
while flag   
    
    % 找出父节点的忽略closeList的子节点
    child_nodes = child_nodes_cal(parent_node,  m, n, obs, closeList); 
    
    % 判断这些子节点是否在openList中，若在，则比较更新；没在则追加到openList中
    for i = 1:size(child_nodes,1)
        child_node = child_nodes(i,:);
        [in_flag,openList_idx] = ismember(child_node, openList, 'rows');
        g = openList_cost(min_idx, 1) + norm(parent_node - child_node);
        h = abs(child_node(1) - target_node(1)) + abs(child_node(2) - target_node(2));
        f = g+h;
        
        if in_flag   % 若在，比较更新g和f        
            if g < openList_cost(openList_idx,1)
                openList_cost(openList_idx, 1) = g;
                openList_cost(openList_idx, 3) = f;
                openList_path{openList_idx,2} = [openList_path{min_idx,2}; child_node];
            end
        else         % 若不在，追加到openList
            openList(end+1,:) = child_node;
            openList_cost(end+1, :) = [g, h, f];
            openList_path{end+1, 1} = child_node;
            openList_path{end, 2} = [openList_path{min_idx,2}; child_node];
        end
    end
   
    
    % 从openList移除移动代价最小的节点到 closeList
    closeList(end+1,: ) =  openList(min_idx,:);
    closeList_cost(end+1,1) =   openList_cost(min_idx,3);
    closeList_path(end+1,:) = openList_path(min_idx,:);
    openList(min_idx,:) = [];
    openList_cost(min_idx,:) = [];
    openList_path(min_idx,:) = [];
 
    % 重新搜索：从openList搜索移动代价最小的节点
    [~, min_idx] = min(openList_cost(:,3));
    parent_node = openList(min_idx,:);
    
    % 判断是否搜索到终点
    if parent_node == target_node
        closeList(end+1,: ) =  openList(min_idx,:);
        closeList_cost(end+1,1) =   openList_cost(min_idx,1);
        closeList_path(end+1,:) = openList_path(min_idx,:);
        flag = 0;
    end
end
    

%% 画路径
path_opt = closeList_path{end,2};
path_opt(:,1) = path_opt(:,1)-0.5;
path_opt(:,2) = path_opt(:,2)-0.5;
scatter(path_opt(:,1), path_opt(:,2), 'k');
plot(path_opt(:,1), path_opt(:,2), 'k');
  