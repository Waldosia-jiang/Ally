% Dijkstra算法
% 作者：Ally
% 日期：2020/12/18

clc
clear
close all

%% 图定义
% 根据节点的邻近节点表及字母节点-数字节点对应表，构造节点元胞数组
nodes_dist = cell(0);
nodes_dist(1,:) = {1, [2, 6, 7], [12, 16, 14]};
nodes_dist(2,:) = {2, [1, 3, 6], [12, 10, 7]};
nodes_dist(3,:) = {3, [2, 4, 5, 6], [10, 3, 5, 6]};
nodes_dist(4,:) = {4, [3, 5], [3, 4]};
nodes_dist(5,:) = {5, [3, 4, 6, 7], [5, 4, 2, 8]};
nodes_dist(6,:) = {6, [1, 2, 3, 5, 7], [16, 7, 6, 2, 9]};
nodes_dist(7,:) = {7, [1, 5, 6], [14, 8, 9]};

%% 算法初始化
% S/U的第一列表示节点编号
% 对于S，第二列表示从源节点到本节点已求得的最小距离，不再变更；
% 对于U，第二列表示从源节点到本节点暂时求得的最小距离，可能会变更
S = [4, 0];
U(:,1) = [1, 2, 3, 5, 6, 7];
U(:,2) = [inf, inf, 3, 4, inf, inf];

% 最优路径及暂时最优路径的初始化
path_opt = cell(7,2);
path_opt(4,:) = {4, 4};

path_temp = cell(7,2);
path_temp(3,:) = {3, [4, 3]};
path_temp(4,:) = {4, 4};
path_temp(5,:) = {5, [4, 5]};

%% 循环遍历所有节点
while ~isempty(U)
    
    % 在U集合找出当前最小距离值及对应节点,并移除该节点至S集合中
    [dist_min, idx] = min(U(:,2));
    node_min = U(idx, 1);
    S(end+1,:) = [node_min, dist_min];
    U(idx,:) = [];
    
    % 将最小距离值的节点添加到最优路径集合
    path_opt(node_min,:) = path_temp(node_min,:);
    
    %% 依次遍历最小距离节点的邻节点，判断是否在U集合中更新邻节点的距离值
    for i = 1:length(nodes_dist{node_min, 2})
        
        % 需要判断的节点
        node_temp = nodes_dist{node_min, 2}(i);
        
        % 找出U集合中节点node_temp的索引值
        idx_temp = find(node_temp == U(:,1));
        
        % 判断是否更新
        if ~isempty(idx_temp)
            if dist_min + nodes_dist{node_min, 3}(i) < U(idx_temp, 2)
                U(idx_temp, 2) = dist_min + nodes_dist{node_min, 3}(i);
                
                % 更新暂时最优路径
                path_temp{node_temp, 1} = node_temp;
                path_temp{node_temp, 2} = [path_opt{node_min, 2}, node_temp];                
            end
        end
    end
end
        
        
        
    
    





