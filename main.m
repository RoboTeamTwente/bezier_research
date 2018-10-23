clear,clc;
% MAIN CODE (pseudo style)

% Get information from world_state
% This is not possible now so everything is defined/generated
%  - Struct objects (.x,.y,.radius)
%  - Current robot position and velocity
%  - Field size [x, y]
fieldSize = [120, 90];
fieldCoordinates = [fieldSize(1) fieldSize(2); ...
    -fieldSize(1) fieldSize(2); -fieldSize(1) -fieldSize(2); ...
    fieldSize(1) -fieldSize(2)]/2;
v0 = struct('amp',10,'theta',0.5*pi);

% Generate objects and start/end point (is received from world in real code)
nObjects = 5; % used for testing stuff
obj = [rand(nObjects,1)*(fieldSize(1)/2) rand(nObjects,1)*(fieldSize(2)/2)];
ptObject = [fieldCoordinates; obj];
m = randi([-1 1], nObjects,2); % generate random -1 1 matrix
m(~m) = 1; % turn zeros into 1
ptObject(5:end,:) = ptObject(5:end,:).*m; % multiply so it's not only positive
ptStart = [rand(1,1)*(fieldSize(1)/2) rand(1,1)*(fieldSize(2)/2)]*m(1);
ptEnd = [rand(1,1)*(fieldSize(1)/2) rand(1,1)*(fieldSize(2)/2)]*m(1);

% Set
[nObjects,~]=size(ptObject); % this one should be used for real stuff

%% Get Voronoi diagram
%  - Matrix with connected points in the Voronoi diagram
%  - 6493 = start, 6494 = end
%  - Change name function from voronoiPlanning to makeVoronoi
[allComb, center] = voronoiPlanning(nObjects, ptObject, ptStart, ptEnd);

%%
% Get shortest path
%  - Matrix (numberPathNodes-by-3) path = [ID, x, y]
%   (this includes the start and end nodes)
if isempty(find(allComb(:,2)==6493,1)) || isempty(find(allComb(:,2)==6494,1))
    disp('No connections to start and/or end point...');
    path = [6493, ptStart; 6494, ptEnd];
else
    [path] = findShortestPath(center, allComb, center(end-1,1), center(end,1));
end

figure
plotter(allComb,center,path,ptObject)
axis([-fieldSize(1) fieldSize(1), -fieldSize(2) fieldSize(2)]*1.1/2)

% Take the first 3 nodes from path and create first path of Bezier Curve
%   (the end node of this first path will be somewhere on the edge between
%    the second and third node)
if length(path(:,1)) > 2
    obst = struct('x',ptObject(:,1),'y',ptObject(:,2),'radius',5*ones(nObjects,1));
    [curve] = createBezierCurve(path,v0,obst);
end

% start at the third node
% (set of control points now contains 2 points)
% while % not at the end of the path
%     if % convex including next node does not contain any obstacle
%         % add node to set of control points
%     else
%         % choose point on edge from last node to next node such that the
%         %   convex does not contain any obstacle
%         % add this point to the set of control points
%
%         % make bezier curve and add it to the total curve
%
%         % empty set of control points
%         % add last point of previous curve and next node
%     end
% end