clear all; close all; clc;

% TODO LIST
% Make voronoi diagram

ptObject = [20 30; 60 90; 10 45; 65 10; 90 60]; 
[nObjects,~]=size(ptObject);

% All triangles
triangleCombinations = possibleCombinations(1:nObjects,3); 

% Compute circumcircles
[nCombinations,~] = size(triangleCombinations);
[center, radius] = createCircumcircles(triangleCombinations, ptObject, nCombinations);

% Receive delaunay points + combinations
[validCenter, validCombinations] = makeDelaunay(ptObject, center, radius, nCombinations, nObjects, triangleCombinations);
sortedCenter = sortCenter(validCenter);

% Make voronoi
% [vx, vy] = createVoronoi(validCenter, validCombinations, nObjects, ptObject);  

% Check
x = ptObject(:,1); y = ptObject(:,2);
tri = delaunay(x,y);
tr = triangulation(tri,x,y);
c = tr.circumcenter();
[vx, vy] = voronoi(x,y);

%% Plot
close all
figure
set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
subplot(1,2,1)
plot(ptObject(:,1), ptObject(:,2),'r*');
hold on
triplot(validCombinations, ptObject(:,1), ptObject(:,2));
% viscircles([center(i,1) center(i,2)], radius(i)); %
plot(validCenter(:,1), validCenter(:,2), 'g*')
plot(c(:,1), c(:,2), 'd')
% plot(temp(:,1),temp(:,2),'d','MarkerEdgeColor','red') %
xlim([0 100]); ylim([0 100]);
grid on

subplot(1,2,2)
plot(x,y,'r+',vx,vy,'b-')
%% Functions
function [vx,vy] = createVoronoi(validCenter, validCombinations, nObjects, ptObject)    
    x = ptObject(:,1);
    y = ptObject(:,2);
    % Create matrix T where i and j are endpoints of edge of triangle T(i,j)
    n = nObjects;
    t = repmat((1:size(validCombinations,1))',1,3);
    T = sparse(validCombinations,validCombinations(:,[3 1 2]),t,n,n); 

    % i and j are endpoints of internal edge in triangle E(i,j)
    E = (T & T').*T; 
    % i and j are endpoints of external edge in triangle F(i,j)
    F = xor(T, T').*T;

    % v and vv are triangles that share an edge
    [~,~,v] = find(triu(E));
    [~,~,vv] = find(triu(E'));

    % Internal edges
    vx = [validCenter(v,1) validCenter(vv,1)]';
    vy = [validCenter(v,2) validCenter(vv,2)]';

    % Compute lines-to-infinity
    % i and j are endpoints of the edges of triangles in z
    [i,j,z] = find(F);

    % Counter-clockwise components of lines between endpoints
    dx = x(j) - x(i);
    dy = y(j) - y(i);

    % Calculate scaling factor for length of line-to-infinity
    % Distance across range of data
    rx = max(x)-min(x); 
    ry = max(y)-min(y);
    % Distance from vertex to center of data
    cx = (max(x)+min(x))/2 - validCenter(z,1); 
    cy = (max(y)+min(y))/2 - validCenter(z,2);
    % Sum of these two distances
    nm = sqrt(rx.*rx + ry.*ry) + sqrt(cx.*cx + cy.*cy);
    % Compute scaling factor
    scale = nm./sqrt((dx.*dx+dy.*dy));

    % Lines from voronoi vertex to "infinite" endpoint
    % We know it's in correct direction because compononents are CCW
    ex = [c(z,1) c(z,1)-dy.*scale]';
    ey = [c(z,2) c(z,2)+dx.*scale]';
    % Combine with internal edges
    vx = [vx ex];
    vy = [vy ey];
end

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

