function [allComb, center] = voronoiPlanning(nObjects, ptObject, ptStart, ptEnd)
% Generates a Voronoi diagram
%
% INPUTS
% nObjects = the amount of objects (robots) that must be avoided
% ptObject = the coordinates of the objects
% ptStart = start position of the robot
% ptEnd = end position of the robot
%
% OUTPUT
% allComb = matrix in which all points that are connected in the Voronoi
% diagram are put. 1 row = 1 combination of 2 points.

% All triangles
triangleCombinations = possibleCombinations(1:nObjects,3); 

% Compute circumcircles
[nCombinations,~] = size(triangleCombinations);
[center, radius] = createCircumcircles(triangleCombinations, ptObject, nCombinations);

% Receive delaunay points + combinations
[validCenter, validCombinations] = makeDelaunay(ptObject, center, radius, nCombinations, nObjects, triangleCombinations);
[nCombinations,~] = size(validCenter);
center = [(1:nCombinations)' validCenter];
center = [center; [6493, ptStart]; [6494, ptEnd]];

% Find adjacent centers + triangles
adjacentCenter = findAdjacentCenter(nCombinations, validCombinations);

% Lines from start/end to center points
[startComb, endComb] = getStartEndLines(validCombinations, center, nCombinations);

if isempty(startComb) && isempty(endComb)
    allComb = adjacentCenter;
elseif isempty(startComb) && ~isempty(endComb)
    allComb = [adjacentCenter; endComb];
elseif ~isempty(startComb) && isempty(endComb)
    allComb = [adjacentCenter; startComb];
else
    allComb = [adjacentCenter; startComb; endComb];
end

allComb = [(1:length(allComb(:,1)))', allComb]; % enumerate allComb

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

    function [cx, cy, nLines] = makeLines(nCombinations, adjacentTriangles, validCenter, radiusx, radiusy)
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
    [~,n] = size(radiusx);
    for i = 1:n
        cx(1:2,p) = radiusx(1:2,i);
        cy(1:2,p) = radiusy(1:2,i);
        p = p + 1;
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

    function adjacentCenter = findAdjacentCenter(nCombinations, validCombinations)
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

    [nTriangles,~] = size(adjacentTriangles);
    p = 1;
    for i = 1:nTriangles
        for k = 2:4
            if adjacentTriangles(i,k) ~= 0 && adjacentTriangles(i,k) > adjacentTriangles(i,1)
                adjacentCenter(p,:) = [adjacentTriangles(i,1), adjacentTriangles(i,k)];
                p = p + 1;
            end
        end
    end
    end

    function [startComb, endComb] = getStartEndLines(validCombinations, center, nCombinations)
    startComb = [];
    endComb = [];
    p = 1;
    q = 1;
    for i = 1:nCombinations
        if validCombinations(i,1) == 1
            if validCombinations(i,2) == 2
                endComb(p,:) = [6494 center(i,1)];
                p = p + 1;
            end
            startComb(q,:) = [6493 center(i,1)];
            q = q + 1;
        elseif validCombinations(i,1) == 2
            endComb(p,:) = [6494 center(i,1)];
            p = p + 1;
        end
    end
    end                    
end
