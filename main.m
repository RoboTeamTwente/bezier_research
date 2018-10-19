clear,clc;
%% MAIN CODE (pseudo style)

% Get Voronoi diagram
%  - Matrix (numberNodes-by-3) nodes = [ID, x, y]
%  - Matrix (numberSegments-by-3) segments = [segmentID, Node1ID, Node2ID]

% Get shortest path
%  - Matrix (numberPathNodes-by-3) path = [ID, x, y]
%   (this includes the start and end nodes)

% Take the first 3 nodes from path and create first path of Bezier Curve
%   (the end node of this first path will be somewhere on the edge between
%    the second and third node)

% start at the third node
% set of control points now contains 2 points
while % not at the end of the path
    if % convex including next node does not contain any obstacle
        % add node to set of control points
    else
        % choose point on edge from last node to next node such that the
        %   convex does not contain any obstacle
        % add this point to the set of control points
        
        % make bezier curve and add it to the total curve
        
        % empty set of control points
        % add last point of previous curve and next node
    end
end