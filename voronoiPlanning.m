clear all; close all; clc;

% TODO LIST
% Not use MATLAB function 'boundary'
% Lines to infinity
% x coordinate sorting is NOT the best option
% Remove lines inside polygons created by cicumcenter polygons
% Remove lines to boundary points if points are outside of boundary

%% 
fieldSize = [1200 900]; % size of the field: x y
fieldCoordinates = [fieldSize(1) fieldSize(2); ...
    fieldSize(1) -fieldSize(2); -fieldSize(1) fieldSize(2); ...
    -fieldSize(1) -fieldSize(2)]/2;
nObjects = 3; % used for testing stuff
obj = [rand(nObjects,1)*(fieldSize(1)/2) rand(nObjects,1)*(fieldSize(2)/2)];
ptObject = [fieldCoordinates; obj];
% ptObject = [0 0; 100 100; 0 100; 100 0; 30 40; 50 80; 35 70];
ptStart = [rand(1,1)*(fieldSize(1)/2) rand(1,1)*(fieldSize(2)/2)];
startOrientationAngle=0.5*pi; % 0-2pi
x = ptObject(:,1); y = ptObject(:,2);
[nObjects,~]=size(ptObject); % this one should be used for real

% All triangles
triangleCombinations = possibleCombinations(1:nObjects,3); 

% Compute circumcircles
[nCombinations,~] = size(triangleCombinations);
[center, radius] = createCircumcircles(triangleCombinations, ptObject, nCombinations);

% Receive delaunay points + combinations
[validCenter, validCombinations] = makeDelaunay(ptObject, center, radius, nCombinations, nObjects, triangleCombinations);
sortedCenter = sortCenter(validCenter);
[nCombinations,~] = size(validCenter);

% Create boundary polygon
k = boundary(x,y,0); % not okay

% Calculate middle points on polygon
midPoint = calculateMid(x,y,k);
[nMidpoints,~] = size(midPoint);

% Make vx & vy
vx = ones(2, nCombinations - 1 + nMidpoints); vy = vx;
[vx, vy] = centerLines(nCombinations, sortedCenter, vx, vy);

% Find closest center points to mid points
closestPoint = findClosest(nMidpoints, nCombinations, validCenter, midPoint);

% Add midpoints and closest points to vx & vy
[vx, vy] = addRemain(vx, vy, midPoint, closestPoint, nMidpoints, nCombinations);

%% Plot
close all
figure
set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
plot(ptObject(:,1), ptObject(:,2),'r*');
hold on
triplot(validCombinations, ptObject(:,1), ptObject(:,2));
plot(validCenter(:,1), validCenter(:,2), 'k*')
% plot(vx,vy,'m-')
plot(ptStart(1),ptStart(2),'g*');
xlim([-fieldSize(1)/2-50 fieldSize(1)/2+50]); ylim([-fieldSize(2)/2-50 fieldSize(2)/2+50]);
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

function sorted = sortCenter(validCenter)
[n, ~] = size(validCenter);
    for i = 1:n
        [~, index] = max(validCenter(:,1));
        if i == 1
            sorted(n,:) = validCenter(index,:);
        else
            sorted(n+1-i,:) = validCenter(index,:);
        end
        validCenter(index,:) = [0 0];
    end
    
    for i = 1:n-1
        if sorted(i,1) == sorted(i+1,1)
            if sorted(i,2) > sorted(i+1,2)
                sorted([n+i n+i+1]) = sorted([n+i+1 n+i]);
            end
        end
    end
end

function midPoint = calculateMid(x,y,k)
    for i = 1:length(k)-1
        midPoint(i,:) =  [(x(k(i))+x(k(i+1)))/2 (y(k(i))+y(k(i+1)))/2];
    end
end

function [cx, cy] = centerLines(nCombinations, sortedCenter, cx, cy)
    for i = 1:nCombinations
            cx(1,i) = sortedCenter(i,1);
            cy(1,i) = sortedCenter(i,2);
        if i ~= 1 
            cx(2,i-1) = sortedCenter(i,1);
            cy(2,i-1) = sortedCenter(i,2);
        end
        if i == nCombinations
            cx(2,i) = sortedCenter(1,1);
            cy(2,i) = sortedCenter(1,2);
        end
    end
end

function closestPoint = findClosest(nMidpoints, nCombinations, validCenter, midPoint)
closestPoint = [];
    for i = 1:nMidpoints
        for k = 1:nCombinations
            distance(k) = sqrt((validCenter(k,1) - midPoint(i,1))^2 + (validCenter(k,2) - midPoint(i,2))^2);
        end
        [~,index] = min(distance);
        closestPoint(i,:) = [validCenter(index,1) validCenter(index,2)]; 
    end
end

function [vx, vy] = addRemain(vx, vy, midPoint, closestPoint, nMidpoints, nCombinations)
    for i = 1:nMidpoints
        vx(1,i + nCombinations) = midPoint(i,1);
        vx(2,i + nCombinations) = closestPoint(i,1);
        vy(1,i + nCombinations) = midPoint(i,2);
        vy(2,i + nCombinations) = closestPoint(i,2);
    end       
end