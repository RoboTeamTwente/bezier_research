function [path] = findShortestPath(nodesIN, segments, startID, endID) 
% findShortestPath  Find the shortest path through a set of connected points.
%    > nodesIN:    list of nodes [(int)ID, (float)x, (float)y].
%    > segments:   list of segments [(int)ID, (float)x, (float)y].
%    > startID:    ID of the node where to start at (int).
%    > endID:      ID of the node where to end at (int).

%% Setup
numNodes = length(nodesIN(:,1));
numSegs = length(segments(:,1));
endInd = find(nodesIN(:,1)==endID,1); % index of end node in the nodes input
for i = 1:numNodes
    j = nodesIN(i,1); % ID of node
    ind = nodesIN(:,1) == j;
    dist_to_end = sqrt((nodesIN(i,2)-nodesIN(endInd,2))^2 + ((nodesIN(i,3)-nodesIN(endInd,3))^2)); % euclidean distance to end node
    nodes(j) = struct('ID',j,'x',nodesIN(ind,2),'y',nodesIN(ind,3),'via',NaN,'dist_start',inf,'dist_end',dist_to_end,'visited',false);

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
    nodes(parentID).visited = true;
    children = [segments(segments(:,2)==parentID, 3); segments(segments(:,3)==parentID, 2)]; % ID's of nodes that are connected to parent node
    for i = 1:length(children)
        childID = children(i); % ID of node on the other side of the segment
        if ~nodes(childID).visited
        dist_to_start = nodes(parentID).dist_start + sqrt((nodes(parentID).x - nodes(childID).x)^2 + (nodes(parentID).y - nodes(childID).y)^2);
        %cost = dist_to_start + nodes(childID).dist_end;
        
        % Update node info
        nodes(childID).dist_start = dist_to_start;
        nodes(childID).via = parentID;

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
    
    if isempty(queue)
        disp('Queue is empty! Something must be wrong...');
        path = [];
        return;
    end
end

%% Backtrack path to make list of nodes
path = [endID, nodes(endID).x, nodes(endID).y];
while path(1,1) ~= startID
    nextID = nodes(path(1,1)).via;
    path = [[nextID, nodes(nextID).x, nodes(nextID).y]; path];
end
end


