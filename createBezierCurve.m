function [Q] = createBezierCurve(path,v0,obst)
% Takes in a path with points and spits out a smooth curve passed these
% points.
% -> v0:    initial velocity struct (amp, theta)
% -> obst:  list containing obstacles

% TODO: What if the minimum curvature is still too high?

if isempty(path) || length(path(:,1)) < 3
    Q = path(:,2:3);
    disp('Please insert a path of at least 3 points.');
    return;
end

Q = zeros(4,2); % control points
pts = path(:,2:3); % path points [X, Y]
obstInPolygon = findObstaclesInPolygon(pts(1:3,:),obst);
dangerObst = findMostDangerousObstacle(obstInPolygon,pts(1:3,:));


%% Determine which case we have
% Case I:   p0, p1 and p2 are on 1 line (not used, special case of II/III)
% Case II:  q1 is to the opposite side of p0p1 as p2
% Case III: q1 is to the same side of p0p1 as p2. p0p1 can not intersect
%   p1p2 without intersecting an obstacle (convex)
% Case IV:  p0q1 intersects p1p2 without intersecting an obstacle

obstVec = [dangerObst.x, dangerObst.y];
v0UVec = [cos(v0.theta), sin(v0.theta)];
angDiff = abs(angleOf(pts(2,:)-pts(1,:)) - angleOf(pts(3,:)-pts(1,:))); % angle between p0p1 and p0p2
if abs(v0.theta-angleOf(pts(2,:)-pts(1,:))) < angDiff && abs(v0.theta-angleOf(pts(3,:)-pts(1,:))) < angDiff && ~v0IntersectsObstacle(pts,obstInPolygon,v0UVec)
    % condition 1: p0+t*v0 must intersect with line segment p1p2
    % condition 2: p0+t*v0 cannot intersect with an obstacle
    
    % Case IV
    currentCase = 4;
    disp('case IV')
elseif onSameSide(pts(1:3,:),v0UVec)
    % Case III
    currentCase = 3;
    disp('case III')
else
    % Case II
    currentCase = 2;
    disp('case II')
end

%% Execute the current case
% This part of the code is split up in 2 different parts:
%       1 that takes into account obstacles and 1 that doesn't.
%       this is to reduce calculations
if isempty(obstInPolygon.x)
    % Simple case, no obstacles to take into account
    switch currentCase
        case 2
            % Determine maxes for parameters
            % max_q2 and max_q1 lay on the edges of the voronoi diagram
            if round(v0.amp) == 0 % TODO: should be limited, not rounded
                max_q1 = pts(1,:) + 10*v0UVec; % for orientation, constant could be used to control acceleration
            else
                max_q1 = pts(1,:) + v0.amp*v0UVec; % TODO: incorporate Voronoi
            end
            max_q2 = pts(2,:) - 2 * (pts(3,:)-pts(2,:))/norm(pts(3,:)-pts(2,:)); % TODO: incorporate Voronoi
            
            % max_q3 is computed such that p0p1max_q3 is the biggest triangle not
            %   intersecting an obstacle. No obstacles, so max_q3 = p2.
            max_q3 = pts(3,:);
            
            % Determine parameters by minimizing curvature
            a = 0.5; % q1-parameter; interval [0,1]
            b = 0.5; % q2-parameter; interval [0,1]
            c = 0.5; % q3-parameter; interval [0,1]
            
            % Determine control points
            Q(1,:) = pts(1,:); % q0 = p0
            Q(2,:) = pts(1,:) + a*(max_q1-pts(1,:)); % q1 = p0 + a*(max_q1-p0)
            Q(3,:) = pts(2,:) + b*(max_q2-pts(2,:)); % q2 = p1 + b*(max_q2-p1)
            Q(4,:) = pts(2,:) + c*(max_q3-pts(2,:)); % q3 = p1 + c*(max_q3-p1)
            
        case 3
            % Determine maxes for parameters
            % max_q1 lays on the edge of the voronoi diagram on the line p0 + t*v0
            if round(v0.amp) == 0 % TODO: should be limited, not rounded
                max_q1 = pts(1,:) + 10*v0UVec; % for orientation, constant could be used to control acceleration
            else
                max_q1 = pts(1,:) + 10 * v0UVec; % TODO: incorporate Voronoi
            end
            % max_q2 is equal to max_q3
            % max_q3 is computed such that max_q1p1max_q3 is the biggest triangle not
            %   intersecting an obstacle. No obstacles, so max_q3 = p2.
            max_q3 = pts(3,:);
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
            
        case 4
            % max_q2 is computed such that p0p1max_q2 is the biggest triangle not
            %   intersecting an obstacle. No obstacles, so max_q2 = p2.
            max_q2 = pts(3,:);
            
            s = (pts(2,2)-pts(1,2) - tan(v0.theta)*(pts(2,1)-pts(1,1))) / (tan(v0.theta)*(pts(3,1)-pts(2,1)) - (pts(3,2)-pts(2,2)));
            
            % only parameter to minimize curvature
            b = 1;
            
            Q(1,:) = pts(1,:); % q0 = p0
            Q(2,:) = pts(2,:) + s*(pts(3,:)-pts(2,:));
            Q(3,:) = Q(2,:) + b*(max_q2-Q(2,:));
            Q(4,:) = []; % only 3 control points needed
    end
else
    % Difficult case, take into account the obstacles
    switch currentCase
        case 2
            % Determine maxes for parameters
            % max_q2 and max_q1 lay on the edges of the voronoi diagram
            if round(v0.amp) == 0 % TODO: should be limited, not rounded
                max_q1 = pts(1,:) + 10*v0UVec; % for orientation, constant could be used to control acceleration
            else
                max_q1 = pts(1,:) + v0.amp*v0UVec; % TODO: incorporate Voronoi
            end
            max_q2 = pts(2,:) - 2 * (pts(3,:)-pts(2,:))/norm(pts(3,:)-pts(2,:)); % TODO: incorporate Voronoi
            
            % max_q3 is computed such that p0p1max_q3 is the biggest triangle not
            %   intersecting an obstacle.
            p0ToObstacle = obstVec - pts(1,:);
            rotDir = (atan2(pts(2,2),pts(2,1))-atan2(dangerObst.y,dangerObst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(dangerObst.y,dangerObst.x)); % direction in which to rotate the vecToObst
            rotAng = rotDir*asin(dangerObst.radius/norm(p0ToObstacle)); % angle to rotate p0ToObstacle by
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
            
        case 3
            % Determine maxes for parameters
            % max_q1 lays on the edge of the voronoi diagram on the line p0 + t*v0
            if round(v0.amp) == 0 % TODO: should be limited, not rounded
                max_q1 = pts(1,:) + 10*v0UVec; % for orientation, constant could be used to control acceleration
            else
                max_q1 = pts(1,:) + 10*v0UVec; % TODO: Voronoi
            end
            % max_q2 is equal to max_q3
            % max_q3 is computed such that max_q1p1max_q3 is the biggest triangle not
            %   intersecting an obstacle.
            max_q1ToObstacle = obstVec - max_q1;
            rotDir = (atan2(pts(2,2),pts(2,1))-atan2(dangerObst.y,dangerObst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(dangerObst.y,dangerObst.x)); % direction in which to rotate the vecToObst
            rotAng = rotDir*asin(dangerObst.radius/norm(max_q1ToObstacle)); % angle to rotate p0ToObstacle by
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
            
        case 4
            % max_q2 is computed such that p0p1max_q2 is the biggest triangle not
            %   intersecting an obstacle.
            p0ToObstacle = obstVec - pts(1,:);
            rotDir = (atan2(pts(2,2),pts(2,1))-atan2(dangerObst.y,dangerObst.x)) / abs(atan2(pts(2,2),pts(2,1))-atan2(dangerObst.y,dangerObst.x)); % direction in which to rotate the vecToObst
            rotAng = rotDir*asin(dangerObst.radius/norm(p0ToObstacle)); % angle to rotate p0ToObstacle by
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
            
            Q(1,:) = pts(1,:); % q0 = p0
            Q(2,:) = pts(2,:) + s*(pts(3,:)-pts(2,:));
            Q(3,:) = Q(2,:) + b*(max_q2-Q(2,:));
            Q(4,:) = []; % only 3 control points needed
    end
end


if round(v0.amp) == 0 % TODO: should be limited, not rounded
    Q = [Q(1,:); Q(1,:); Q(2:end,:)];
end


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

    function [bool] = v0IntersectsObstacle(pts,obst,v0UVec)
        bool = false;
        for i = 1:length(obst.x)
            obstvec = [obst.x(i), obst.y(i)];
            distObstToV0 = norm((pts(1,:)-obstvec) - dot(pts(1,:)-obstvec,v0UVec)*v0UVec);
            p0p1 = pts(2,:)-pts(1,:);
            angP0P1ToObst = acos(dot(obstvec/norm(obstvec), p0p1/norm(p0p1)));
            angP0P1ToV0 = acos(dot(v0UVec, p0p1/norm(p0p1)));
            if distObstToV0 < obst.radius(i) || angP0P1ToObst < angP0P1ToV0
                bool = true;
                return;
            end
        end
    end

    function [dangerObst] = findMostDangerousObstacle(obstInPolygon,pts)
        if isempty(obstInPolygon.x)
            dangerObst = struct('x',[],'y',[],'radius',[]);
        elseif length(obstInPolygon.x) == 1
            dangerObst = obstInPolygon;
        else
            angles = zeros(1,length(obstInPolygon.x));
            for i = 1:length(obstInPolygon.x)
                p0Obst = [obstInPolygon.x(i), obstInPolygon.y(i)]-pts(1,:);
                p0Obst_dist = norm(p0Obst);
                p0ObstUVec = p0Obst/p0Obst_dist;
                p0p1UVec = (pts(2,:)-pts(1,:))/norm(pts(2,:)-pts(1,:));
                % angle between p0p1 and edge of obstacle area
                % calculated as angle between p0p1 and p0-obstacle minus
                % angle between p0-obstacle and p0-edgeOfObstacle
                angles(i) = abs(acos(dot(p0ObstUVec,p0p1UVec))) - abs(asin(obstInPolygon.radius(i)/p0Obst_dist));
            end
            ind = find(angles==min(angles),1);
            dangerObst = struct('x',obstInPolygon.x(ind),'y',obstInPolygon.y(ind),'radius',obstInPolygon.radius(ind));
        end
    end
end