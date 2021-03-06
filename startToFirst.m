clear all; close all; clc;

fieldSize = [120 90]; % size of the field: x y
fieldCoordinates = [fieldSize(1) fieldSize(2); ...
    -fieldSize(1) fieldSize(2); -fieldSize(1) -fieldSize(2); ...
    fieldSize(1) -fieldSize(2)]/2;
nObjects = 15; % used for testing stuff
obj = [rand(nObjects,1)*(fieldSize(1)/2) rand(nObjects,1)*(fieldSize(2)/2)];
ptObject = [fieldCoordinates; obj];
m = randi([-1 1], nObjects,2); % generate random -1 1 matrix
m(~m) = 1; % turn zeros into 1
ptObject(5:end,:) = ptObject(5:end,:).*m; % multiply so it's not only positive
% ptObject = [0 0; 100 100; 0 100; 100 0; 30 40; 50 80; 35 70];
ptStart = [rand(1,1)*(fieldSize(1)/2) rand(1,1)*(fieldSize(2)/2)]*m(1);
ptEnd = [rand(1,1)*(fieldSize(1)/2) rand(1,1)*(fieldSize(2)/2)]*m(1);
% ptStart = [-40 -30];
startOrientationAngle=rand(1,1)*2*pi; % 0-2pi
orientationMargin = 10; % random value, for plotting of the orientation vector
scaleFactor = 1; % random value, to determine the partition of the radius
x = ptObject(:,1); y = ptObject(:,2);
[nObjects,~]=size(ptObject); % this one should be used for real stuff

% Orientation vector (for plotting)
h = orientationMargin * sin(startOrientationAngle);
l = orientationMargin * cos(startOrientationAngle);
ptStartOrientation = [ptStart(1)+l, ptStart(2)+h];
dp = ptStartOrientation - ptStart; 

% All triangles
triangleCombinations = possibleCombinations(1:nObjects,3); 

% Compute circumcircles
[nCombinations,~] = size(triangleCombinations);
[center, radius] = createCircumcircles(triangleCombinations, ptObject, nCombinations);

% Receive delaunay points + combinations
[validCenter, validCombinations] = makeDelaunay(ptObject, center, radius, nCombinations, nObjects, triangleCombinations);
% sortedCenter = sortCenter(validCenter);
[nCombinations,~] = size(validCenter);

% Find adjacent triangles
adjacentTriangles = findAdjacentTriangles(nCombinations, validCombinations);

% Create boundary polygon
k = [1;2;3;4;1];

% Make vx & vy
[vx, vy] = centerLines(nCombinations, adjacentTriangles, validCenter);

% Give end variables a normal name
center = [(1:nCombinations)' validCenter];
combs = validCombinations; 

% First line stuff
% Closest point
[index] = findClosestPoint(ptStart,center,nCombinations);
closestPointx = [ptStart(1); center(index,2)];
closestPointy = [ptStart(2); center(index,3)];

% All points in quadrant of end point
[quadEndx,quadEndy] = findQuadrantPoints(ptStart, ptEnd, nCombinations, center);

% All points in quadrant of vector
[quadVectorx,quadVectory] = findQuadrantAnglePoints(startOrientationAngle, nCombinations, ptStart, center);

% Points within radius between start and end
[radiusx,radiusy] = findPointInRadius(ptStart, ptEnd, nCombinations, center, scaleFactor);

% Plot
close all;
figure
set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop

subplot(2,2,1)
triplot(combs, ptObject(:,1), ptObject(:,2),'k-')
hold on
plot(vx,vy,'b-')
plot(closestPointx, closestPointy,'m-')
plot(ptStart(1),ptStart(2),'g*')
plot(ptEnd(1),ptEnd(2),'g*')
plot(ptObject(:,1), ptObject(:,2),'k*')
plot(center(:,2), center(:,3), 'r*')
quiver(ptStart(1),ptStart(2),dp(1),dp(2),0,'MaxHeadSize',0.5)
xlim([-fieldSize(1)/2-5 fieldSize(1)/2+5]); ylim([-fieldSize(2)/2-5 fieldSize(2)/2+5]);
title('Closest point')
grid on

subplot(2,2,2)
triplot(combs, ptObject(:,1), ptObject(:,2),'k-')
hold on
plot(vx,vy,'b-')
plot(quadEndx, quadEndy,'m-')
plot(ptStart(1),ptStart(2),'g*')
plot(ptEnd(1),ptEnd(2),'g*')
plot(ptObject(:,1), ptObject(:,2),'k*')
plot(center(:,2), center(:,3), 'r*')
quiver(ptStart(1),ptStart(2),dp(1),dp(2),0,'MaxHeadSize',0.5)
xlim([-fieldSize(1)/2-5 fieldSize(1)/2+5]); ylim([-fieldSize(2)/2-5 fieldSize(2)/2+5]);
title('Points in quadrant end point')
grid on

subplot(2,2,3)
triplot(combs, ptObject(:,1), ptObject(:,2),'k-')
hold on
plot(vx,vy,'b-')
plot(quadVectorx, quadVectory,'m-')
plot(ptStart(1),ptStart(2),'g*')
plot(ptEnd(1),ptEnd(2),'g*')
plot(ptObject(:,1), ptObject(:,2),'k*')
plot(center(:,2), center(:,3), 'r*')
quiver(ptStart(1),ptStart(2),dp(1),dp(2),0,'MaxHeadSize',0.5)
xlim([-fieldSize(1)/2-5 fieldSize(1)/2+5]); ylim([-fieldSize(2)/2-5 fieldSize(2)/2+5]);
title('Points in quadrant vector')
grid on

subplot(2,2,4)
triplot(combs, ptObject(:,1), ptObject(:,2),'k-');
hold on
plot(vx,vy,'b-')
plot(radiusx,radiusy,'m-')
plot(ptStart(1),ptStart(2),'g*');
plot(ptEnd(1),ptEnd(2),'g*');
plot(ptObject(:,1), ptObject(:,2),'k*');
plot(center(:,2), center(:,3), 'r*')
quiver(ptStart(1),ptStart(2),dp(1),dp(2),0,'MaxHeadSize',0.5)
xlim([-fieldSize(1)/2-5 fieldSize(1)/2+5]); ylim([-fieldSize(2)/2-5 fieldSize(2)/2+5]);
title('Points within radius')
grid on

%% Functions
function [center, radius] = createCircumcircles(combs, ptObject, nCombinations)
corners = ones(3,2);  
center = ones(nCombinations,2);
radius = ones(nCombinations,1);
    for i = 1:nCombinations
        for k = 1:3
            corners(k,:) = [ptObject(combs(i,k),1), ptObject(combs(i,k),2)];
        end
    A = [corners(1,1) corners(1,2)];
    B = [corners(2,1) corners(2,2)];
    C = [corners(3,1) corners(3,2)];
    
    % Line AB is represented as ax + by = c 
    [a, b] = lineFromPoints(A, B);

    % Line BC is represented as ex + fy = g 
    [e, f] = lineFromPoints(B, C);

    [a, b, c] = perpendicularBisectorFromLine(A, B, a, b); 
    [e, f, g] = perpendicularBisectorFromLine(B, C, e, f);

    [x, y] = lineLineIntersection(a, b, c, e, f, g);
    center(i,:) = [x, y];
    
    % Calculate radius
    radius(i) = sqrt((center(i,1)-corners(1,1))^2+(center(i,2)-corners(1,2))^2);
    
    end
end

function area = shoelace(corners) % shoelace formula
    area = 0.5*(corners(1,1) * corners(2,2) + corners(2,1) * corners(3,2) + ...
        corners(3,1) * corners(1,2) - corners(2,1) * corners(1,2) - ...
        corners(3,1) * corners(2,2) - corners(1,1) * corners(3,2));
end

function [a, b, c] = lineFromPoints(A, B)
    a = B(1,2) - A(1,2);
    b = A(1,1) - B(1,1);
    c = a*(A(1,1)) + b*(A(1,2));
end

function [a, b, c] = perpendicularBisectorFromLine(A, B, a, b)
    midPoint = [(A(1) + B(1))/2 (A(2) + B(2))/2];
    % c = -bx + ay 
    c = -b * (midPoint(1)) + a * (midPoint(2));
    temp = a;
    a = -b;
    b = temp;
end

function [x, y] = lineLineIntersection(a, b, c, e, f, g)
    determinant = a*f - e*b; 
    if determinant == 0 
        disp('Lines are parallel, cannot calculate center')
        x = 0; y = 0;
    else
        x = (f*c - b*g)/determinant; 
        y = (a*g - e*c)/determinant; 
    end
end

function combs = possibleCombinations(v,m)
v = v(:).'; % Make sure v is a row vector.
n = length(v);
if n == m
    combs = v;
elseif m == 1
    combs = v.';
else
    combs = [];
    if m < n && m > 1
        for k = 1:n-m+1
            Q = possibleCombinations(v(k+1:n),m-1);
            combs = [combs; [v(ones(size(Q,1),1),k) Q]]; 
        end
    end
end
end

function [temp, combs] = makeDelaunay(ptObject, center, radius, nCombinations, nObjects, combs)
temp = center;
for i = 1:nCombinations
    for k = 1:nObjects
        distance = sqrt((center(i,1)-ptObject(k,1))^2+(center(i,2)-ptObject(k,2))^2);
        if round(distance,4) < round(abs(radius(i)),4)
            temp(i,:) = [0 0];
            combs(i,:) = [0 0 0];
        end
    end
end
temp = temp(any(temp,2),:); % remove zero rows
combs = combs(any(combs,2),:);
end

function [cx, cy, nLines] = centerLines(nCombinations, adjacentTriangles, validCenter)
cx = [];
cy = [];
p = 1;
for i = 1:nCombinations
    for k = 2:4
        if adjacentTriangles(i,k) ~= 0
            cx(1:2,p) = [validCenter(adjacentTriangles(i,1),1); validCenter(adjacentTriangles(i,k),1)];
            cy(1:2,p) = [validCenter(adjacentTriangles(i,1),2); validCenter(adjacentTriangles(i,k),2)];
            p = p + 1;
        end
    end
end

[~,nLines] = size(cx);
for i = 1:nLines
    for k = 1:nLines
        if k ~= i 
            if (cx(1,k) == cx(1,i)) && (cx(2,k) == cx(2,i)) || (cx(1,k) ...
                    == cx(2,i) && cx(2,k) == cx(1,i))
                cx(:,k) = [0 0];
                cy(:,k) = [0 0];
            end
        end
    end
end
cx(:,~any(cx,1)) = [];
cy(:,~any(cy,1)) = [];
end

function adjacentTriangles = findAdjacentTriangles(nCombinations, validCombinations)
newCombinations = ones(nCombinations, 4);
for i = 1:nCombinations
    newCombinations(i,:) = [validCombinations(i,1) validCombinations(i,2) validCombinations(i,3) validCombinations(i,1)];
end

adjacentTriangles = zeros(nCombinations,4);
adjacentTriangles(:,1) = [1:nCombinations];
for i = 1:nCombinations
    p = 2;
    for k = 1:3
        comb = [newCombinations(i,k) newCombinations(i,k+1)];
        for n = 1:nCombinations
            for t = 1:3
                if i ~= n
                    if (newCombinations(n,t) == comb(1,1) && newCombinations(n,t+1) == comb(1,2)) || ...
                        (newCombinations(n,t+1) == comb(1,1) && newCombinations(n,t) == comb(1,2))
                        adjacentTriangles(i,p) = n;
                        p = p + 1;
                    end
                end
            end
        end
    end
end      
end

function [index] = findClosestPoint(pt,options,nCombinations)
    for i = 1:nCombinations
        dist(i) = sqrt((pt(1)-options(i,2))^2+(pt(2)-options(i,3))^2);
    end
[~,index] = min(dist);
end

function [x,y] = findQuadrantPoints(ptStart, ptEnd, nCombinations, center)
p = 1;
quadCenter = [];
    if (ptStart(1) < ptEnd(1)) && (ptStart(2) < ptEnd(2))
        % quad = 1;
        for i = 1:nCombinations
            if (ptStart(1) < center(i,2)) && (ptStart(2) < center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    elseif (ptStart(1) < ptEnd(1)) && (ptStart(2) > ptEnd(2))
        % quad = 2;
        for i = 1:nCombinations
            if (ptStart(1) < center(i,2)) && (ptStart(2) > center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    elseif (ptStart(1) > ptEnd(1)) && (ptStart(2) > ptEnd(2))
        % quad = 3;
        for i = 1:nCombinations
            if (ptStart(1) > center(i,2)) && (ptStart(2) > center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    else
        % quad = 4;
        for i = 1:nCombinations
            if (ptStart(1) > center(i,2)) && (ptStart(2) < center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    end

    [n,~] = size(quadCenter);
    if isempty(quadCenter)
        x = ptStart(1); y = ptStart(2);
    else
        for i = 1:n
            x(:,i) = [ptStart(1) quadCenter(i,1)];
            y(:,i) = [ptStart(2) quadCenter(i,2)];
        end
    end
end

function [x,y] = findQuadrantAnglePoints(startOrientationAngle, nCombinations, ptStart, center)
p = 1;
quadCenter = [];
    if startOrientationAngle >= 0 && startOrientationAngle <= 0.5*pi
        % quad = 1;
        for i = 1:nCombinations
            if (ptStart(1) < center(i,2)) && (ptStart(2) < center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    elseif startOrientationAngle > 1.5*pi && startOrientationAngle <= 2*pi
        % quad = 2;
        for i = 1:nCombinations
            if (ptStart(1) < center(i,2)) && (ptStart(2) > center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    elseif startOrientationAngle >= pi && startOrientationAngle <= 1.5*pi
        % quad = 3;
        for i = 1:nCombinations
            if (ptStart(1) > center(i,2)) && (ptStart(2) > center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    else
        % quad = 4;
        for i = 1:nCombinations
            if (ptStart(1) > center(i,2)) && (ptStart(2) < center(i,3))
                quadCenter(p,:) = [center(i,2) center(i,3)];
                p = p + 1;
            end
        end
    end

    [n,~] = size(quadCenter);
    if isempty(quadCenter)
        x = ptStart(1); y = ptStart(2);
    else
        for i = 1:n
           x(:,i) = [ptStart(1) quadCenter(i,1)];
           y(:,i) = [ptStart(2) quadCenter(i,2)];
        end
    end
end

function [x,y] = findPointInRadius(ptStart, ptEnd, nCombinations, center, scaleFactor)
p = 1;
centerInRadius = [];
dist = sqrt((ptStart(1)-ptEnd(1))^2+(ptStart(2)-ptEnd(2))^2);
    for i = 1:nCombinations
        distToPoint(i) = sqrt((ptStart(1)-center(i,2))^2+(ptStart(2)-center(i,3))^2);
        if distToPoint(i) < scaleFactor*dist
            centerInRadius(p,:) = [center(i,2) center(i,3)];
            p = p + 1;
        end
    end
    
    [n,~] = size(centerInRadius);
    if isempty(centerInRadius)
        x = ptStart(1); y = ptStart(2);
    else
        for i = 1:n
           x(:,i) = [ptStart(1) centerInRadius(i,1)];
           y(:,i) = [ptStart(2) centerInRadius(i,2)];
        end
    end
end