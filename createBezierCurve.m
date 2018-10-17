function [curve,Q] = createBezierCurve(path,v0,obst)
% Takes in a path with points and spits out a smooth curve passed these
% points.
% -> v0:    initial velocity vector
% -> obst:  list containing obstacles
if isempty(path) || length(path(:,1)) < 3
    disp('Please insert a path of at least 3 points.');
end

% Case I:   p0, p1, p2 are collinear, v0 is co-directional to p1-p0
% Case II:  q1 is to the opposite side of p0p1 as p2
% Case III: q1 is to the same side of p0p1 as p2. p0p1 can not intersect
%   p1p2 without intersecting an obstacle (convex)
% Case IV:  p0q1 intersects p1p2 without intersecting an obstacle

% Ignore case I, this is a special case of case II or III.
Q = zeros(4,2); % control points
pts = path(:,2:3); % path points [X, Y]

if ~onSameSide(pts,v0)
    %% Case II
    % Determine maxes for parameters (only #3 atm)
    % max_q2 and max_q1 lay on the edges of the voronoi diagram
    max_q1 = pts(1,:) + 1.5 * v0/norm(v0); % TODO: incorporate Voronoi
    max_q2 = pts(2,:) - 1.5 * (pts(3,:)-pts(2,:))/norm(pts(3,:)-pts(2,:)); % TODO: incorporate Voronoi
    
    % max_q3 is computed such that p0p1max_q3 is the biggest triangle not
    %   intersecting an obstacle.
    max_q1ToObstacle = [obst.x, obst.y] - pts(1,:);
    rotDir = (atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)); % direction in which to rotate the vecToObst
    rotAng = rotDir*asin(obst.radius/norm(max_q1ToObstacle)); % angle to rotate p0ToObstacle by
    max_q1ToQ3 = max_q1ToObstacle * [cos(rotAng), sin(rotAng); -sin(rotAng), cos(rotAng)];
    alpha = [max_q1ToQ3(2)/max_q1ToQ3(1), -1]; % vector to make life easier
    s = (dot(pts(1,:),alpha) - dot(pts(2,:),alpha))/(dot(pts(3,:),alpha) - dot(pts(2,:),alpha));
    if s > 1 || s < 0 || isnan(s)
        s = 1;
    end
    max_q3 = pts(2,:) + s*(pts(3,:)-pts(2,:));
    
    % Determine parameters by minimizing curvature
    a = 0.5; % q1-parameter; interval [0,1]
    b = 0.8; % q2-parameter; interval [0,1]
    c = 0.5; % q3-parameter; interval [0,1]
    
    % Determine control points
    Q(1,:) = pts(1,:); % q0 = p0
    Q(2,:) = pts(1,:) + a*(max_q1-pts(1,:)); % q1 = p0 + a*(max_q1-p0)
    Q(3,:) = pts(2,:) + b*(max_q2-pts(2,:)); % q2 = p1 + b*(max_q2-p1)
    Q(4,:) = pts(2,:) + c*(max_q3-pts(2,:)); % q3 = p1 + c*(max_q3-p1)
    
    disp('opposite side!')
else
    %% Case III
    % Determine maxes for parameters (only #3 atm)
    % max_q1 lays on the edge of the voronoi diagram on the line p0 + t*v0
    max_q1 = pts(1,:) + v0/dot(v0,[obst.x, obst.y]-pts(1,:)); % assuming the voronoi edge is with the obstacle
    
    % max_q3 is computed such that max_q1p1max_q3 is the biggest triangle not
    %   intersecting an obstacle.
    max_q1ToObstacle = [obst.x, obst.y] - max_q1;
    rotDir = (atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)); % direction in which to rotate the vecToObst
    rotAng = rotDir*asin(obst.radius/norm(max_q1ToObstacle)); % angle to rotate p0ToObstacle by
    max_q1ToQ3 = max_q1ToObstacle * [cos(rotAng), sin(rotAng); -sin(rotAng), cos(rotAng)];
    alpha = [max_q1ToQ3(2)/max_q1ToQ3(1), -1]; % vector to make life easier
    s = (dot(max_q1,alpha) - dot(pts(2,:),alpha))/(dot(pts(3,:),alpha) - dot(pts(2,:),alpha));
    if s > 1 || s < 0 || isnan(s)
        s = 1;
    end
    max_q3 = pts(2,:) + s*(pts(3,:)-pts(2,:));
    max_q2 = max_q3;
    
    % Determine parameters by minimizing curvature
    a = 1; % q1-parameter; interval [0,1]
    b = 0.2; % q2-parameter; interval [0,1]
    c = 0.8; % q3-parameter; interval [0,1]
    
    % Determine control points
    Q(1,:) = pts(1,:); % q0 = p0
    Q(2,:) = pts(1,:) + a*(max_q1-pts(1,:)); % q1 = p0 + a*(max_q1-p0)
    Q(3,:) = pts(2,:) + b*(max_q2-pts(2,:)); % q2 = p1 + b*(max_q2-p1)
    Q(4,:) = pts(2,:) + c*(max_q3-pts(2,:)); % q3 = p1 + c*(max_q3-p1)
    
    disp('same side!')
end
curve = points2Curve(Q);


%% FUNCTIONS
    function [bool] = onSameSide(pts,v0)
        % determines whether q1 is on the same side of p0p1 as p2 or not
        p0 = pts(1,:);
        p1 = pts(2,:);
        p2 = pts(3,:);
        q1 = p0 + v0; % for this purpose, the magnitude does not matter
        
        % THEOREM: CD are on the same side of AB if and only if (AB x AC).(AB x AD) > 0
        AB = [p1-p0, 0]; % make 3D vector of p0p1
        AC = [q1-p0, 0]; % make 3D vector of p0q1
        AD = [p2-p0, 0]; % make 3D vector of p0p2
        
        if dot(cross(AB,AC), cross(AB,AD)) > 0
            bool = true;
        else
            bool = false;
        end
    end

    function [curve] = points2Curve(P)
        % P: n by 2 matrix with control points (x, y)
        dt = 0.01; % distance between two adjacent points
        T = 0:dt:1;
        X = zeros(1,length(T));
        Y = zeros(1,length(T));
        
        if isempty(P)
            disp('Please insert control points!');
            return;
        end
        
        n = length(P(:,1)) - 1; % degree of polynomial
        
        for i = 0:n
            X = X + nchoosek(n,i) * T.^i .* (1-T).^(n-i) * P(i+1,1);
            Y = Y + nchoosek(n,i) * T.^i .* (1-T).^(n-i) * P(i+1,2);
        end
        
        curve = [X;Y];
    end
end