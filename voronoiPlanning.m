clear all; close all; clc;

%%
ptObject = [20 30; 30 90; 50 45; 65 10; 90 60; ];
[vx, vy] = createVoronoi(ptObject(:,1), ptObject(:,2));
plot(vx, vy, 'b');
xlim([0 100]); ylim([0 100]);

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