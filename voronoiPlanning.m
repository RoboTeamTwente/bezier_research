clear all; close all; clc;

% TODO LIST
% all possible combinations in combination matrix
% choose right points out of the center matrix 

%%
ptObject = [20 30; 60 90; 10 45; 65 10; 90 60];
[nObjects,~]=size(ptObject);

% All triangles
triangleCombinations = possibleCombinations(1:nObjects, 3); % 3 corners in triangle

% Compute circumcircles
[nCombinations,~] = size(triangleCombinations);
[radius, center] = createCircumcircles(triangleCombinations, ptObject, nCombinations);

% Check
x = ptObject(:,1); y = ptObject(:,2);
tri = delaunay(x,y);
tr = triangulation(tri,x,y);
c = tr.circumcenter();

% Plot
% close all
figure
set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
plot(ptObject(:,1), ptObject(:,2),'r*');
hold on
triplot(triangleCombinations, ptObject(:,1), ptObject(:,2));
% triplot(rightCombinations,x,y);
plot(c(:,1),c(:,2),'m*');
plot(center(:,1), center(:,2), 'g*')
xlim([0 100]); ylim([0 100]);

%% Functions
function [vx,vy] = createVoronoi(x,y)
    tri = delaunay(x,y);
    tr = triangulation(tri,x,y);
    c = tr.circumcenter();
    
    % Create matrix T where i and j are endpoints of edge of triangle T(i,j)
    n = numel(x);
    t = repmat((1:size(tri,1))',1,3);
    T = sparse(tri,tri(:,[3 1 2]),t,n,n); 

    % i and j are endpoints of internal edge in triangle E(i,j)
    E = (T & T').*T; 
    % i and j are endpoints of external edge in triangle F(i,j)
    F = xor(T, T').*T;

    % v and vv are triangles that share an edge
    [~,~,v] = find(triu(E));
    [~,~,vv] = find(triu(E'));

    % Internal edges
    vx = [c(v,1) c(vv,1)]';
    vy = [c(v,2) c(vv,2)]';

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
    cx = (max(x)+min(x))/2 - c(z,1); 
    cy = (max(y)+min(y))/2 - c(z,2);
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

function combinations = possibleCombinations(v,m)
v = v(:).'; % Make sure v is a row vector.
n = length(v);
if n == m
    combinations = v;
elseif m == 1
    combinations = v.';
else
    combinations = [];
    if m < n && m > 1
        for k = 1:n-m+1
            Q = possibleCombinations(v(k+1:n),m-1);
            combinations = [combinations; [v(ones(size(Q,1),1),k) Q]]; 
        end
    end
end
end

function [radius, center] = createCircumcircles(combinations, ptObject, nObjects)
corners = ones(3,2);  
radius = ones(nObjects,1);
center = ones(nObjects,2);
    for i = 1:nObjects
        for k = 1:3
            corners(k,:) = [ptObject(combinations(i,k),1), ptObject(combinations(i,k),2)];
        end
        a = sqrt((corners(2,1)-corners(1,1))^2+(corners(2,2)-corners(1,2))^2); % triangle sides
        b = sqrt((corners(3,1)-corners(2,1))^2+(corners(3,2)-corners(2,2))^2);
        c = sqrt((corners(1,1)-corners(3,1))^2+(corners(1,2)-corners(3,1))^2);
        area = shoelace(corners);
%         area = 0.5*(corners(1,1) * corners(2,2) + corners(2,1) * corners(3,2) + ...
%         corners(3,1) * corners(1,2) - corners(2,1) * corners(1,2) - ...
%         corners(3,1) * corners(2,2) - corners(1,1) * corners(3,2));
        radius(i,1) = a * b * c / (4 * area);
        barCenter = [a^2 * (b^2 + c^2 -a^2), b^2 * (c^2 + a^2 - b^2), c^2 * (a^2 + b^2 - c^2)];
        almostCenter = barCenter(1) * corners(1,:) + barCenter(2) * corners(2,:) + barCenter(3) * corners(3,:);
        % This is not the right name but ok
        center(i,:) = almostCenter/sum(barCenter);
    end
end

function area = shoelace(corners) % shoelace formula
    area = 0.5*(corners(1,1) * corners(2,2) + corners(2,1) * corners(3,2) + ...
        corners(3,1) * corners(1,2) - corners(2,1) * corners(1,2) - ...
        corners(3,1) * corners(2,2) - corners(1,1) * corners(3,2));
end



