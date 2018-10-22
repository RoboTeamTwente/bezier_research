function [curve] = createBezierCurve(path,v0,obst)
% Takes in a path with points and spits out a smooth curve passed these
% points.
% -> v0:    initial velocity struct (amp, theta)
% -> obst:  list containing obstacles

% TODO: What if the minimum curvature is still too high?

if isempty(path) || length(path(:,1)) < 3
    disp('Please insert a path of at least 3 points.');
end

% Case I:   p0, p1 and p2 are on 1 line
% Case II:  q1 is to the opposite side of p0p1 as p2
% Case III: q1 is to the same side of p0p1 as p2. p0p1 can not intersect
%   p1p2 without intersecting an obstacle (convex)
% Case IV:  p0q1 intersects p1p2 without intersecting an obstacle

% MULTIPLE OBJECTS
Q = zeros(4,2); % control points
pts = path(:,2:3); % path points [X, Y]

obstVec = [obst.x, obst.y];
v0UVec = [cos(v0.theta), sin(v0.theta)];
distObstToV0 = norm((pts(1,:)-obstVec) - dot(pts(1,:)-obstVec,v0UVec)*v0UVec);
angDiff = abs(angleOf(pts(2,:)-pts(1,:)) - angleOf(pts(3,:)-pts(1,:))); % angle between p0p1 and p0p2
if abs(v0.theta-angleOf(pts(2,:)-pts(1,:))) < angDiff && abs(v0.theta-angleOf(pts(3,:)-pts(1,:))) < angDiff && distObstToV0 > obst.radius
    % condition 1: p0+t*v0 must intersect with line segment p1p2
    % condition 2: p0+t*v0 cannot intersect with an obstacle
    
    %% Case IV
    % max_q2 is computed such that p0p1max_q2 is the biggest triangle not
    %   intersecting an obstacle.
    p0ToObstacle = obstVec - pts(1,:);
    rotDir = (atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)); % direction in which to rotate the vecToObst
    rotAng = rotDir*asin(obst.radius/norm(p0ToObstacle)); % angle to rotate p0ToObstacle by
    p0ToQ2 = p0ToObstacle * [cos(rotAng), sin(rotAng); -sin(rotAng), cos(rotAng)];
    alpha = [p0ToQ2(2)/p0ToQ2(1), -1]; % vector to make life easier
    s = (dot(pts(1,:),alpha) - dot(pts(2,:),alpha))/(dot(pts(3,:),alpha) - dot(pts(2,:),alpha));
    if s > 1 || s < 0 || isnan(s)
        s = 1;
    end
    max_q2 = pts(2,:) + s*(pts(3,:)-pts(2,:));
    
    s = (pts(2,2)-pts(1,2) - tan(v0.theta)*(pts(2,1)-pts(1,1))) / (tan(v0.theta)*(pts(3,1)-pts(2,1)) - (pts(3,2)-pts(2,2)));
    
    % only parameter to minimize curvature
    b = 0.5;
    
    Q(2,:) = pts(2,:) + s*(pts(3,:)-pts(2,:));
    Q(3,:) = Q(2,:) + b*(max_q2-Q(2,:));
    Q(4,:) = []; % only 3 control points needed
    disp('case IV')
elseif onSameSide(pts,v0UVec)
    %% Case III
    % Determine maxes for parameters
    % max_q1 lays on the edge of the voronoi diagram on the line p0 + t*v0
    if round(v0.amp) == 0 % TODO: should be limited, not rounded
        max_q1 = pts(1,:) + 3*v0UVec; % for orientation, constant could be used to control acceleration
    else
        max_q1 = pts(1,:) + 0.5*norm([obst.x, obst.y]-pts(1,:))^2/dot(v0UVec,[obst.x, obst.y]-pts(1,:)) * v0UVec; % assuming the voronoi edge is with the obstacle
    end
    % max_q2 is equal to max_q3
    % max_q3 is computed such that max_q1p1max_q3 is the biggest triangle not
    %   intersecting an obstacle.
    max_q1ToObstacle = obstVec - max_q1;
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
    a = 0.5; % q1-parameter; interval [0,1]
    b = 0.2; % q2-parameter; interval [0,1]
    c = 0.8; % q3-parameter; interval [0,1]
    
    % Determine control points
    Q(1,:) = pts(1,:); % q0 = p0
    Q(2,:) = pts(1,:) + a*(max_q1-pts(1,:)); % q1 = p0 + a*(max_q1-p0)
    Q(3,:) = pts(2,:) + b*(max_q2-pts(2,:)); % q2 = p1 + b*(max_q2-p1)
    Q(4,:) = pts(2,:) + c*(max_q3-pts(2,:)); % q3 = p1 + c*(max_q3-p1)
    
    disp('case III')
else
    %% Case II
    % Determine maxes for parameters
    % max_q2 and max_q1 lay on the edges of the voronoi diagram
    if round(v0.amp) == 0 % TODO: should be limited, not rounded
        max_q1 = pts(1,:) + 3*v0UVec; % for orientation, constant could be used to control acceleration
    else
        max_q1 = pts(1,:) + v0.amp*v0UVec; % TODO: incorporate Voronoi
    end
    max_q2 = pts(2,:) - 2 * (pts(3,:)-pts(2,:))/norm(pts(3,:)-pts(2,:)); % TODO: incorporate Voronoi
    
    % max_q3 is computed such that p0p1max_q3 is the biggest triangle not
    %   intersecting an obstacle.
    p0ToObstacle = obstVec - pts(1,:);
    rotDir = (atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(obst.y,obst.x)); % direction in which to rotate the vecToObst
    rotAng = rotDir*asin(obst.radius/norm(p0ToObstacle)); % angle to rotate p0ToObstacle by
    p0ToQ3 = p0ToObstacle * [cos(rotAng), sin(rotAng); -sin(rotAng), cos(rotAng)];
    alpha = [p0ToQ3(2)/p0ToQ3(1), -1]; % vector to make life easier
    s = (dot(pts(1,:),alpha) - dot(pts(2,:),alpha))/(dot(pts(3,:),alpha) - dot(pts(2,:),alpha));
    if s > 1 || s < 0 || isnan(s)
        s = 1;
    end
    max_q3 = pts(2,:) + s*(pts(3,:)-pts(2,:));
    
    % Determine parameters by minimizing curvature
    a = 0.5; % q1-parameter; interval [0,1]
    b = 0.5; % q2-parameter; interval [0,1]
    c = 0.5; % q3-parameter; interval [0,1]
    
    % Determine control points
    Q(1,:) = pts(1,:); % q0 = p0
    Q(2,:) = pts(1,:) + a*(max_q1-pts(1,:)); % q1 = p0 + a*(max_q1-p0)
    Q(3,:) = pts(2,:) + b*(max_q2-pts(2,:)); % q2 = p1 + b*(max_q2-p1)
    Q(4,:) = pts(2,:) + c*(max_q3-pts(2,:)); % q3 = p1 + c*(max_q3-p1)
    
    disp('case II')
end
if round(v0.amp) == 0 % TODO: should be limited, not rounded
    Q = [Q(1,:); Q(1,:); Q(2:end,:)];
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

    function [ang] = angleOf(vec)
        ang = [];
        if length(vec) == 2
            ang = atan2(vec(2),vec(1));
        else
            disp('Vector must be 2D');
        end
    end

    function [obstInPolygon] = findObstaclesInPolygon(pts,obst)
        % Points need to be in order of which they are connected
        obstInPolygon = struct('x',[],'y',[],'radius',[]);
        if isempty(pts) || length(pts(:,1)) < 3
            disp('Enter more than 2 points!');
            return;
        end
        if isempty(obst.x)
            disp('Enter obstacles!');
            return;
        end
        nPoints = length(pts(:,1));
        
        for k = 1:length(obst.x)
            isInside = false;
            obstvec = [obst.x(k), obst.y(k)];
            % Sum the angles between vectors from the obstacles to the points
            % Condition: points need to be in order of connection
            angSum = 0; % sum of all angles
            for i = 1:nPoints
                if i == nPoints
                    obstToP1 = pts(i,:) - obstvec;
                    obstToP2 = pts(1,:) - obstvec;
                else
                    obstToP1 = pts(i,:) - obstvec;
                    obstToP2 = pts(i+1,:) - obstvec;
                end
                obstToP1 = obstToP1/norm(obstToP1); % normalize
                obstToP2 = obstToP2/norm(obstToP2); % normalize
                
                angSum = angSum + acos(dot(obstToP1,obstToP2));
            end
            
            if abs(angSum-2*pi) < 0.002*pi % allow a deviation of 0.1 percent
                % the point is inside the polygon
                isInside = true;
            else
                % compute distance to every vertex
                for i = 1:nPoints
                    if i == nPoints
                        j = 1;
                    else
                        j = i+1;
                    end
                    p1p2 = pts(j,:)-pts(i,:);
                    r = dot(p1p2,obstvec-pts(i,:))/norm(p1p2)^2;
                    if r < 0
                        % p1 is closest
                        dist = norm(pts(i,:)-obstvec);
                    elseif r > 1
                        % p2 is closest
                        dist = norm(pts(j,:)-obstvec);
                    else
                        %  compute distance to line
                        % line: a + t*n (where a and n are vectors)
                        % point: p (vector)
                        p1 = pts(i,:);
                        n = p1p2/norm(p1p2);
                        p = obstvec;
                        dist = norm((p1-p) - dot(p1-p, n)*n);
                    end
                    if dist < obst.radius
                        isInside = true;
                    end
                end
            end
            
            if isInside
                obstInPolygon.x = [obstInPolygon.x; obst.x(k)];
                obstInPolygon.y = [obstInPolygon.y; obst.y(k)];
                obstInPolygon.radius = [obstInPolygon.radius; obst.radius(k)];
            end
        end
    end
end