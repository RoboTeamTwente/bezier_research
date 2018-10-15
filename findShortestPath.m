function [path] = findShortestPath(nodesIN, segments, startID, endID) 
% INPUT: 
%   -> list of nodes [(int)ID, (float)x, (float)y]
%   -> list of segments [(int)ID, (int)Node1, (int)Node2] 
%   -> start node (int)ID
%   -> end node (int)ID
% OUTPUT: Path with node ID's and coordinates from start to end

%% Setup
numNodes = length(nodesIN(:,1));
numSegs = length(segments(:,1));
endInd = find(nodesIN(:,1)==endID,1); % index of end node in the nodes input
for i = 1:numNodes
    j = nodesIN(i,1); % ID of node
    dist_to_end = sqrt((nodesIN(i,2)-nodesIN(endInd,2))^2 + ((nodesIN(i,3)-nodesIN(endInd,3))^2)); % euclidean distance to end node
    nodes(j) = struct('ID',j,'x',nodesIN(j,2),'y',nodesIN(j,3),'via',NaN,'dist_start',inf,'dist_end',dist_to_end);

    if nodes(j).ID == startID
        % visit start node
        nodes(j).dist_start = 0;
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
path = [endID, nodes(endID).x, nodes(endID).y];
while path(1,1) ~= startID
    nextID = nodes(path(1,1)).via;
    path = [[nextID, nodes(nextID).x, nodes(nextID).y]; path];
end
end


