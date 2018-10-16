clear,clc;
% INPUT: 
%   -> list of nodes [(int)ID, (float)x, (float)y]
%   -> list of segments [(int)ID, (int)Node1, (int)Node2] 
%   -> start node (int)ID
%   -> end node (int)ID
% OUTPUT: List of nodes from start to end that together form the shortest path

% %% Settings for random input
% NN = 20; % number of nodes
% NS = 30; % number of segments (must be bigger than NN)
% startID = 1;
% endID = randi(NN,1);

%% Settings for n-by-n grid
n = 7;
startID = 1;
endID = randi(n^2,1);

%% Input
% nodesIN = [(1:6); [0 0 1 1 2 3]; [0 1 0 1 1 1]]';
% segments = [(1:8); [1 1 1 2 3 3 4 5]; [2 3 4 4 4 5 5 6]]';
% startID = 1;
% endID = 6;

% nodesIN = [(1:NN); 100*rand(2,NN)]';
% segments = [(1:NS); floor(linspace(1,NN-1,NS)); ceil(linspace(2,NN,NS))]';

nodesIN = zeros(n^2,3);
nodesIN(:,1) = (1:n^2)';
segments = zeros(2*n^2-2*n,3);
segments(:,1) = (1:(2*n^2-2*n))';
c = 1;
s = 1;
for i = 1:n
    for j = 1:n
        nodesIN(c,2:3) = [i+rand, j+rand];
        if i == n && j == n
            % do nothing
        elseif i == n
            segments(s,2:3) = [c, c+1]; % segment to the top
            s = s+1;
        elseif j == n
            segments(s,2:3) = [c, c+n]; % segment to the right
            s = s+1;
        else
            segments(s,2:3) = [c, c+1]; % segment to the top
            segments(s+1,2:3) = [c, c+n]; % segment to the right
            s = s+2;
        end
        c = c + 1;
    end
end

path = findShortestPath(nodesIN,segments,startID,endID);

%% Plot the whole thing
figure
hold on
for i = 1:length(segments(:,1))
    N1 = segments(i,2); % ID of first node
    N2 = segments(i,3); % ID of second node
    ind1 = find(nodesIN(:,1)==N1);
    ind2 = find(nodesIN(:,1)==N2);
    line([nodesIN(ind1,2), nodesIN(ind2,2)],[nodesIN(ind1,3), nodesIN(ind2,3)],'color','k','linestyle','--')
end
for i = 2:length(path(:,1))
    line([path(i,2), path(i-1,2)],[path(i,3), path(i-1,3)],'color','m','linestyle','-','linewidth',2)
end

grid on
plot(nodesIN(:,2),nodesIN(:,3),'.b','MarkerSize',20);
plot(path(1,2),path(1,3),'.g','MarkerSize',20);
plot(path(end,2),path(end,3),'.r','MarkerSize',20);
axis([0 n+1 0 n+1])


