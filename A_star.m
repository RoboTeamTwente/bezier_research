clear,clc;
% INPUT: 
%   -> list of nodes [(int)ID, (float)x, (float)y]
%   -> list of segments [(int)ID, (int)Node1, (int)Node2] 
%   -> start node (int)ID
%   -> end node (int)ID
% OUTPUT: List of nodes from start to end that together form the shortest path

%% Settings for random input
NN = 150; % number of nodes
NS = 200; % number of segments
startID = 1;
endID = 5;

%% Input
% nodesIN = [(1:6); [0 0 1 1 2 3]; [0 1 0 1 1 1]]';
% segments = [(1:8); [1 1 1 2 3 3 4 5]; [2 3 4 4 4 5 5 6]]';
% startID = 1;
% endID = 6;

nodesIN = [(1:NN); 100*rand(2,NN)]';
segments = [(1:NS); floor(linspace(1,NN-1,NS)); ceil(linspace(2,NN,NS))]';

%% Setup
numNodes = length(nodesIN(:,1));
numSegs = length(segments(:,1));
endInd = find(nodesIN(:,1)==endID,1); % index of end node in the nodes input
for i = 1:numNodes
    j = nodesIN(i,1); % ID of node
    dist_to_end = sqrt((nodesIN(i,2)-nodesIN(endInd,2))^2 + ((nodesIN(i,3)-nodesIN(endInd,3))^2)); % euclidean distance to end node
    nodes(j) = struct('ID',j,'x',nodesIN(j,2),'y',nodesIN(j,3),'via',NaN,'dist_start',inf,'dist_end',dist_to_end,'visited',false);

    if nodes(j).ID == startID
        % visit start node
        nodes(j).dist_start = 0;
        nodes(j).visited = true;
    end
end

queue = startID; % All relevant node ID's are stored here. The lower the cost, the higher up in the queue the node is.

%% Algorithm
while queue(1) ~= endID % while the end node is not on top of the queue
    % Expand the node on top of the queue (parent node)
    parentID = queue(1);
    queue(1) = []; % delete node from list, it is done now
    segID = segments(segments(:,2)==parentID, 1); % ID's of segments that are connected to parent node
    for i = 1:length(segID)
        childID = segments(segID(i),3); % ID of node on the other side of the segment
        dist_to_start = nodes(parentID).dist_start + sqrt((nodes(parentID).x - nodes(childID).x)^2 + (nodes(parentID).y - nodes(childID).y)^2);
        %cost = dist_to_start + nodes(childID).dist_end;
        
        % Update node info
        nodes(childID).dist_start = dist_to_start;
        nodes(childID).via = parentID;
        nodes(parentID).visited = true;
        
        % Update queue
        if isempty(queue)
            queue = childID;
        else 
            % place childID at the correct place in the queue
            for j = 1:length(queue)
                childCost = nodes(childID).dist_start + nodes(childID).dist_end;
                queueNodeCost = nodes(queue(j)).dist_start + nodes(queue(j)).dist_end;
                if childCost < queueNodeCost
                    if j == 1
                        queue = [childID, queue];
                    else
                        queue = [queue(1:j-1), childID, queue(j:end)];
                    end
                    break;
                elseif j == length(queue)
                    queue = [queue, childID];
                    break;
                end
            end
        end
    end
end

%% Backtrack path to make list of nodes
path = endID;
while path(1) ~= startID
    nextID = nodes(path(1)).via;
    path = [nextID, path];
end

%% Plot the whole thing
figure
hold on
for i = 1:length(segments(:,1))
    N1 = segments(i,2); % ID of first node
    N2 = segments(i,3); % ID of second node
    line([nodes(N1).x, nodes(N2).x],[nodes(N1).y, nodes(N2).y],'color','k','linestyle','--')
end
for i = 2:length(path)
    N1 = path(i-1); % ID of first node
    N2 = path(i); % ID of second node
    line([nodes(N1).x, nodes(N2).x],[nodes(N1).y, nodes(N2).y],'color','m','linestyle','-','linewidth',2)
end

grid on
plot(nodesIN(:,2),nodesIN(:,3),'.b','MarkerSize',40);
plot(nodes(startID).x,nodes(startID).y,'.g','MarkerSize',40);
plot(nodes(endID).x,nodes(endID).y,'.r','MarkerSize',40);


