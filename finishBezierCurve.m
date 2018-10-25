function [curve,movementData] = finishBezierCurve(path,obst,Q,getMovementData)
% Takes in a path with points and spits out a smooth curve passed these
% points. This function assumes you already processed the first 3 points
% -> obst:  list containing obstacles
% -> ptStart: end point of the previous curve
% -> Q: set of control points
% -> getMovementData: if true, the function saves the movement data in a
% struct.

% TODO: What if the minimum curvature is still too high?

pts = path(:,2:3); % path points [X, Y]
pos_margin = 0.5; % max distance to which to approach the end point
curve = [];
movementData = struct('Vel',[],'Acc',[],'Ang',[]);
dT = 0.01; % distance between points for movement data

% If the path is not long enough, return
if isempty(path) || length(path(:,1)) < 3
    curve = points2Curve(Q);
    if getMovementData
        movementData.Vel = getVelocity(Q,dT);
        movementData.Acc = getAcceleration(Q,dT);
        movementData.Ang = getRotation(movementData.Vel);
    end
    return;
end

% If the curve has already ended, return
if norm(pts(end,:)-Q(end,:)) < pos_margin
    % Already at end
    curve = points2Curve(Q);
    movementData.Vel = getVelocity(Q,dT);
    movementData.Acc = getAcceleration(Q,dT);
    movementData.Ang = getRotation(movementData.Vel);
    return;
end

% start at the third node
% (set of control points now contains 2 points)

% If the starting point is equal to one of the path nodes, add an
% extrapolation of p1p2 for velocity continuity
if norm(pts(3,:)-Q(end,:)) < pos_margin
    Q = [Q; pts(4,:)];
    pStartCount = 5;
else
    Q = [Q; pts(3,:)];
    pStartCount = 4;
end

if length(pts(:,1)) >= pStartCount
    for pCount = pStartCount:length(pts(:,1))
        obstInPolygon = findObstaclesInPolygon([Q; pts(pCount,:)],obst);
        if isempty(obstInPolygon.x)% convex including next node does not contain any obstacle
            Q = [Q; pts(pCount,:)];% add node to set of control points
        else
            % choose point on edge from last node to next node such that the
            %   convex does not contain any obstacle
            % add this point to the set of control points
            [dangerObst] = findMostDangerousObstacle(obstInPolygon,[Q; pts(pCount,:)]); % avoid this obstacle and you avoid all of them
            p0ToObst = [[dangerObst.x,dangerObst.y]-Q(end-1,:), 0]; % make 3D for dot product
            ang = asin(dangerObst.radius/norm(p0ToObst)); % angle over which p0ToObst should be rotated
            p0PastObst = [p0ToObst(1:2) * [cos(ang), sin(ang); -sin(ang), cos(ang)], 0]; % make 3D for cross product
            p0p1Vec = [Q(end,:)-Q(end-1,:), 0]; % make 3D for cross product
            p1p2Vec = [pts(pCount,:)-Q(end,:), 0]; % make 3D for cross product
            if dot(p0PastObst/norm(p0PastObst),p0p1Vec) < dot(p0ToObst/norm(p0ToObst),p0p1Vec)
                % By rotating p0ToObst, the dot product with p0p1 should
                % become bigger. Otherwise, you should rotate the other
                % way around.
                ang = -ang;
                p0PastObst = [p0ToObst(1:2) * [cos(ang), sin(ang); -sin(ang), cos(ang)], 0];
            end
            s = norm(cross(p0PastObst,p0p1Vec))/norm(cross(p1p2Vec,p0PastObst));
            % limit s, should not be necessary?
            if s > 1
                s = 1;
            elseif s < 0
                s = 0;
            end
            nextCP = Q(end,:) + s*p1p2Vec(1:2); % next control point is somewhere on p1p2
            Q = [Q; nextCP];
            
            % make bezier curve and add it to the total curve
            tempCurve = points2Curve(Q);
            curve = [curve, tempCurve];
            V = getVelocity(Q,dT);
            movementData.Vel = [movementData.Vel, V];
            movementData.Acc = [movementData.Acc, getAcceleration(Q,dT)];
            movementData.Ang = [movementData.Ang, getRotation(V)];
            
            % empty set of control points
            % add last point of previous curve
            Q = [curve(:,end)'; pts(pCount,:)];
        end
    end
end

% Add last part to curve
if ~isempty(Q)
    tempCurve = points2Curve(Q);
    curve = [curve, tempCurve];
    V = getVelocity(Q,dT);
    movementData.Vel = [movementData.Vel, V];
    movementData.Acc = [movementData.Acc, getAcceleration(Q,dT)];
    movementData.Ang = [movementData.Ang, getRotation(V)];
end


%% FUNCTIONS
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

    function [V]=getVelocity(P, dt)
        T=0:dt:1;
        dXdT=zeros(1,length(T));
        dYdT=zeros(1,length(T));
        
        if isempty(P)
            disp('Please insert control points!');
            return;
        end
        
        n=length(P(:,1))-1; % degree of polynomial
        
        for i=0:n
            dXdT=dXdT+nchoosek(n,i)*(i*T.^(i-1).*(1-T).^(n-i)+T.^i.*-(n-i).*(1-T).^(n-i-1))*P(i+1,1);
            dYdT=dYdT+nchoosek(n,i)*(i*T.^(i-1).*(1-T).^(n-i)+T.^i.*-(n-i).*(1-T).^(n-i-1))*P(i+1,2);
        end
        
        % Fix NaN data by extrapolation
        dXdT(1) = dXdT(2) - (dXdT(3)-dXdT(2));
        dXdT(end) = dXdT(end-1) + (dXdT(end-1)-dXdT(end-2));
        dYdT(1) = dYdT(2) - (dYdT(3)-dYdT(2));
        dYdT(end) = dYdT(end-1) + (dYdT(end-1)-dYdT(end-2));
        
        V=[dXdT; dYdT];
    end

    function [A]=getAcceleration(P, dt)
        T = 0:dt:1;
        d2XdT2 = zeros(1,length(T));
        d2YdT2 = zeros(1,length(T));
        
        if isempty(P)
            disp('Please insert control points!');
            return;
        end
        
        n = length(P(:,1)) - 1; % degree of polynomial
        
        for i = 0:n
            d2XdT2 = d2XdT2 + nchoosek(n,i) * (i*(i-1)*T.^(i-2).*(1-T).^(n-i) - 2*i*(n-i)*T.^(i-1).*(1-T).^(n-i-1) + (n-i)*(n-i-1)*T.^i.*(1-T).^(n-i-2)) * P(i+1,1);
            d2YdT2 = d2YdT2 + nchoosek(n,i) * (i*(i-1)*T.^(i-2).*(1-T).^(n-i) - 2*i*(n-i)*T.^(i-1).*(1-T).^(n-i-1) + (n-i)*(n-i-1)*T.^i.*(1-T).^(n-i-2)) * P(i+1,2);
        end
        
        % Fix NaN data by extrapolation
        d2XdT2(1) = d2XdT2(2) - (d2XdT2(3)-d2XdT2(2));
        d2XdT2(end) = d2XdT2(end-1) + (d2XdT2(end-1)-d2XdT2(end-2));
        d2YdT2(1) = d2YdT2(2) - (d2YdT2(3)-d2YdT2(2));
        d2YdT2(end) = d2YdT2(end-1) + (d2YdT2(end-1)-d2YdT2(end-2));
        
        A = [d2XdT2; d2YdT2];
    end

    function [ang]=getRotation(V)
        ang=zeros(1,length(V(1,:)));
        for i=1:length(V(1,:))
            ang(i)=atan(V(2,i)/V(1,i));
            if V(2,i)<0 && V(1,i)<0
                ang(i)=ang(i)-pi;
            elseif V(1,i) < 0
                ang(i)=ang(i)+pi;
            end
        end
        
        % Fix NaN data by extrapolation
        ang(1) = ang(2) - (ang(3)-ang(2));
        ang(end) = ang(end-1) + (ang(end-2)-ang(end-3));
    end
end