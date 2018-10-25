clear all; close all; clc;
% MAIN CODE (pseudo style)
getMovementData = true; % if true, get the velocity, acceleration and rotation of the path

% Get information from world_state
% This is not possible now so everything is defined/generated
%  - Struct objects (.x,.y,.radius)
%  - Current robot position and velocity
%  - Field size [x, y]
%  - Size of robot
robotDiameter = 2*9 + 5; % robot radius in cm
fieldSize = [1200, 900]; % field dimensions in cm
fieldCoordinates = [fieldSize(1) fieldSize(2); ...
    -fieldSize(1) fieldSize(2); -fieldSize(1) -fieldSize(2); ...
    fieldSize(1) -fieldSize(2)]/2;
startOrientationAngle=rand(1,1)*2*pi; % 0-2pi, uses this angle atm
v0 = struct('amp',100,'theta',startOrientationAngle);

% Generate objects and start/end point (is received from world in real code)
nObjects = 15; % used for testing stuff
obj = [rand(nObjects,1)*(fieldSize(1)/2) rand(nObjects,1)*(fieldSize(2)/2)];
ptStart = fieldSize.*(rand(1,2)-0.5);
ptEnd = fieldSize.*(rand(1,2)-0.5);
ptObject = [ptStart; ptEnd; fieldCoordinates; obj];
m = randi([-1 1], nObjects,2); % generate random -1 1 matrix
m(~m) = 1; % turn zeros into 1
ptObject(7:end,:) = ptObject(7:end,:).*m; % multiply so it's not only positive

% Set
[nObjects,~]=size(ptObject); % this one should be used for real stuff

%% Get Voronoi diagram
%  - Matrix with connected points in the Voronoi diagram
%  - 6493 = start, 6494 = end
%  - Change name function from voronoiPlanning to makeVoronoi
[allComb, center, startCP] = voronoiPlanning(nObjects, ptObject, ptStart, ptEnd, robotDiameter, startOrientationAngle);

%% Get shortest path
%  - Matrix (numberPathNodes-by-3) path = [ID, x, y]
%   (this includes the start and end nodes)
if isempty(find(allComb(:,2)==6493,1)) || isempty(find(allComb(:,2)==6494,1))
    disp('No connections to start and/or end point...');
    path = [6493, ptStart; 6494, ptEnd];
else
    [path] = findShortestPath(center, allComb, center(end-1,1), center(end,1));
end

%% Create Bezier Curve
% Take the first 3 nodes from path and create first path of Bezier Curve
%   (the end node of this first path will be somewhere on the edge between
%    the second and third node)
if length(path(:,1)) < 3
    path = [path(1,:); [1 path(1,2)+v0.amp*cos(v0.theta) path(1,3)+v0.amp*sin(v0.theta)]; path(end,:)];
end

rowsToNotUse = [1 2]; % remove start & end point for calculation & plotting
ptObject(rowsToNotUse,:) = [];
nObjects = nObjects - 2;

obst = struct('x',ptObject(:,1),'y',ptObject(:,2),'radius',robotDiameter*ones(nObjects,1));
[Q] = createBezierCurve(path,v0,obst);
[curve,movementData] = finishBezierCurve(path,obst,Q,getMovementData);

%% Show result
figure(1)
%set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
plotter(allComb, center, path, ptObject, curve, v0, nObjects, robotDiameter, fieldSize, ptStart, startOrientationAngle)
axis([-fieldSize(1) fieldSize(1), -fieldSize(2) fieldSize(2)]*1.1/2)
plot(startCP(1), startCP(2), 'm*')

figure(2)
showMovementData(movementData)

% do finishBezierCurve