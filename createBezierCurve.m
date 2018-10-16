function [curve] = createBezierCurve(path,v0)
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
    % Determine maxes for parameters
    
    
    % Determine parameters
    a = 1; % q1-parameter to minimize curvature
    b = 1; % q2-parameter to minimize curvature
    c = 1; % q3-parameter to minimize curvature
    
    % Determine control points
    Q(1,:) = pts(1,:); % q0 = p0
    Q(2,:) = pts(1,:) + a*v0/norm(v0); % q1 = p0 + a*v0/|v0|
    Q(3,:) = pts(2,:) - b*(pts(3,:)-pts(2,:))/norm(pts(3,:)-pts(2,:)); % q2 = p1 - b*(p2-p1)/|p2-p1|
    Q(4,:) = pts(2,:) + c*(pts(3,:)-pts(2,:))/norm(pts(3,:)-pts(2,:)); % q3 = p1 + c*(p2-p1)/|p2-p1|
    
    disp('opposite side!')
else
    %% Case III
    disp('same side!')
end
curve = [];

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
        



end