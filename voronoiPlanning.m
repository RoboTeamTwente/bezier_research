clear all; close all; clc;

%%
ptObject = [20 30; 60 90; 10 45; 65 10; 90 60];
[nObjects,~]=size(ptObject);

% Sort objects with increasing x
sortedObjects = ones(nObjects,2);
sortedObjects = sortObjects(ptObject, nObjects);

% Divide objects
groupA = [];
groupB = [];
[groupA, groupB] = divideObjects(sortedObjects, nObjects);

% All triangles
triangleCombinations = possibleCombinations(nObjects);

% close all
figure
set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
plot(sortedObjects(:,1),sortedObjects(:,2),'r*');
hold on
% tri2=[1 2 3; 2 3 5; 2 4 5];
triplot(triangleCombinations,sortedObjects(:,1),sortedObjects(:,2));
% % triplot(tri,x,y,'g');
% xlim([0 100]); ylim([0 100]);

%%
% [vx, vy] = createVoronoi(ptObject(:,1), ptObject(:,2));
% plot(ptObject(:,1),ptObject(:,2), 'b*')
% hold on
% plot(c(:,1), c(:,2), 'r*');
% plot(vx, vy, 'g');
% triplot(tri,x,y);
% xlim([0 100]); ylim([0 100]);

%%
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

function sortedObjects = sortObjects(ptObject, nObjects)
    for i = 1:nObjects
        [~, index] = max(ptObject(:,1));
        if i==1
            sortedObjects(nObjects,:)=ptObject(index,:);
        else
            sortedObjects(nObjects+1-i,:)=ptObject(index,:);
        end
        ptObject(index,:)=[0 0];
    end
    
    for i = 1:nObjects-1
        if sortedObjects(i,1)==sortedObjects(i+1,1)
            if sortedObjects(i,2) > sortedObjects(i+1,2)
                sortedObjects([nObjects+i nObjects+i+1]) = sortedObjects([nObjects+i+1 nObjects+i]);
            end
        end
    end
end

function [groupA, groupB] = divideObjects(sortedObjects, nObjects)
    divisionNumber=nObjects/2;
    if rem(nObjects, 2)==0
        groupA=sortedObjects(1:divisionNumber,:);
        groupB=sortedObjects(divisionNumber+1:end,:);
    else
        groupA=sortedObjects(1:round(divisionNumber),:);
        groupB=sortedObjects(round(divisionNumber)+1:end,:);
    end
end

function combinations = possibleCombinations(nObjects)
    for i = 1:nObjects
        for k = 1:3
            value = i+k-1;
            if value > nObjects
                value = value - nObjects;
            end
            combinations(i,k) = value;
        end
    end
end
