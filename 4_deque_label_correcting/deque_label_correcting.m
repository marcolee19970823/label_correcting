%% clear
clc
clear

%% 设置变量
g_node_list = []; %网络节点集合
g_link_list = []; %网络弧集合
g_origin = [];   %网络源节点
g_number_of_nodes = 0;%网络节点个数
node_predecessor = [];%前向节点集合
node_label_cost = [];%距离标签集合
Max_label_cost = inf; %初始距离标签
g_adjacent_arc_list={}; %节点邻接弧集合（从节点i发出的弧集合）
g_node_status=[]; %网络节点状态
SE_LIST=[]; %可扫描节点集合

%% 导入网络数据文件，构建基础网络并初始化相关变量
%读取网络节点数据
df_node = csvread('node.csv',1,0);
g_number_of_nodes = size(df_node,1);
g_node_list = df_node(:,1);
g_node_status = zeros(1,g_number_of_nodes);
for i = 1 : g_number_of_nodes
    if df_node(i,4)==1
        g_origin=df_node(i,1);
        o_zone_id = df_node(i,5);
    end
end
Distance = ones(g_number_of_nodes,g_number_of_nodes)*Max_label_cost; %距离矩阵
node_predecessor = -1*ones(1,g_number_of_nodes);
node_label_cost = Max_label_cost*ones(1,g_number_of_nodes);
node_predecessor(1,g_origin) = 0;
node_label_cost(1,g_origin) = 0;
g_adjacent_arc_list = cell(1, g_number_of_nodes);
%读取网络弧数据
df_link = csvread('road_link.csv',1,0);
g_link_list = df_link(:,2:3);
for i = 1 : size(df_link, 1) 
    Distance(df_link(i,2),df_link(i,3)) = df_link(i,4);
    g_adjacent_arc_list{df_link(i,2)} = [g_adjacent_arc_list{df_link(i,2)}, df_link(i,3)];
end

%% 最短路径求解：扫描网络弧，依据检查最优性条件更新距离标签
SE_LIST=[g_origin];
g_node_status(g_origin)=1;
while ~isempty(SE_LIST)
    head=SE_LIST(1); %从可扫描列表中取出第一个元素
    SE_LIST(1)=[]; %从可扫描列表中删除第一个元素
    g_node_status(head) = 2;
    adjacent_arc_list = g_adjacent_arc_list(head); %获取当前节点的邻接弧
    adjacent_arc_list = cell2mat(adjacent_arc_list);
    for i = 1 : length(adjacent_arc_list)
        tail = adjacent_arc_list(i);
        if node_label_cost(tail)>node_label_cost(head)+Distance(head,tail)
            node_label_cost(tail)=node_label_cost(head)+Distance(head,tail);
            node_predecessor(tail)=head;
            if g_node_status(tail)==0
                SE_LIST = [SE_LIST, tail];
                g_node_status(tail) = 1;
            elseif g_node_status(tail) == 2
                SE_LIST = [tail, SE_LIST];
                g_node_status(tail) = 1;
            end
        end
    end   
end

%% 依据前向节点生成最短路径
destination_column = [];
distance_column = [];
path_column = {};
o_zone_id_column = o_zone_id * ones(g_number_of_nodes-1, 1);
d_zone_id_column = [];
agent_id_column = [1:(g_number_of_nodes-1)]';
for i = 1: size(g_node_list, 1)
    destination = g_node_list(i);
    if g_origin ~= destination
        destination_column = [destination_column; destination];
        d_zone_id_column = [d_zone_id_column; df_node(i,5)];
        if node_label_cost(destination)==Max_label_cost
            path="";
            distance = 99999;
            distance_column = [distance_column; 99999];
        else
            to_node=destination;
            path=num2str(to_node);
            while node_predecessor(to_node)~=g_origin
                path=strcat(';',path);
                path=strcat(num2str(node_predecessor(to_node)),path);
                g=node_predecessor(to_node);
                to_node=g;
            end
            path=strcat(';',path);
            path=strcat(num2str(g_origin),path);
            distance_column = [distance_column; node_label_cost(i)];
        end
        path_column=[path_column;path];
    end
end

title = {'agent_id','o_zone_id','d_zone_id','node_sequence','distance'};
result_table=table(agent_id_column,o_zone_id_column,d_zone_id_column,path_column,distance_column,'VariableNames',title);
writetable(result_table, 'agent.csv','Delimiter',',','QuoteStrings',true)


