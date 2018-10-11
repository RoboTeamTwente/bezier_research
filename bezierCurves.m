clear; clc; close all;
% The purpose of this file is to make the planned path responsive to moving
% obstacles. Only for 1 obstacle atm.

%% Set variables
robotRadius = 10;
updateTimes = 50; % number of time steps between 2 plot updates

dt = 1/2000; % normalized time step
minRadius=30; % circle around object
ptStart=[5 5]; % start position
ptEnd=[100 100]; % ball position
ptGoal=[130 80]; % point where the ball should go to
ptObject=[60 65]; % objects that are in the way
[nObstacles,~]=size(ptObject);

% State variables
initVel = 0;
initAng = -0.3*pi;
finalVel = 0;
finalAng = atan2(ptGoal(2)-ptEnd(2), ptGoal(1)-ptEnd(1));
orientDist = 30; % could be used to limit the angular velocity, for now to make the graph more pretty

%% SIMULATION: Initialize
self = struct('X',[],'Y',[],'W',[]);
obst = struct('X',[],'Y',[],'W',[]);

figure
hold on
grid on
xlim([ptStart(1)-4*robotRadius, ptEnd(1)+4*robotRadius]);
ylim([ptStart(2)-4*robotRadius, ptEnd(2)+4*robotRadius]);
axis equal

% Self
selfPathHandle = plot(ptStart(1), ptStart(2), '--k');
plot(ptStart(1), ptStart(2), '.g', 'MarkerSize', 20)
plot(ptEnd(1), ptEnd(2), '.r', 'MarkerSize', 20)

selfPosHandle = rectangle('Position', [ptStart(1)-robotRadius, ptStart(2)-robotRadius, 2*robotRadius, 2*robotRadius], 'Curvature', [1, 1], 'FaceColor', 'y');
selfDirHandle = quiver(0,0,robotRadius*cos(initAng),robotRadius*sin(initAng), 'linewidth', 5,'color',[1 0.5 0]);

% Obstacles
if ~isempty(ptObject)
    for j = 1:length(ptObject(:,1))
        obstPosHandle(j) = rectangle('Position', [ptObject(j,1)-robotRadius, ptObject(j,2)-robotRadius, 2*robotRadius, 2*robotRadius], 'Curvature', [1, 1], 'FaceColor', 'b');
        obstDirHandle(j) = quiver(0,0,robotRadius*cos(0),robotRadius*sin(0), 'linewidth', 5,'color',[1 0.5 0]);
    end
end

%% Go from state to state
makeNewPath = true;
count = 0;
time = 1;
while true
    %% Check if goal is reached
    if ~isempty(self.X) && self.X(time) == ptEnd(1) && self.Y(time) == ptEnd(2)
        disp('Goal reached!');
        break;
    end
    
    if makeNewPath
        %% Set initial and final states
        if round(initVel) == 0 % Should be limited and not rounded
            ptInitState = ptStart + orientDist*[0, 0; cos(initAng), sin(initAng)];
        else
            NumPoints = 4; % Start, initial state, final state, end
            ptInitState = ptStart + initVel/(NumPoints-1)*[cos(initAng), sin(initAng)];
        end
        
        if round(finalVel) == 0 % Should be limited and not rounded
            ptFinalState = ptEnd + orientDist*[cos(pi-finalAng), -sin(pi-finalAng); 0, 0];
        else
            NumPoints = 4; % Start, initial state, final state, end
            ptFinalState = ptEnd + finalVel/(NumPoints-1)*[cos(pi-finalAng), -sin(pi-finalAng)];
        end
        
        %% Initial path
        pts=[ptStart; ptInitState; ptFinalState; ptEnd];
        [X, Y]=bezierCurve(pts, dt);
        ptDanger=findDanger(ptObject, X, Y, minRadius); % check if circle crosses path
        [nDanger,~]=size(ptDanger); % amount of danger points
        
        if nDanger > 0
            %% Avoid obstacle by adding control point
            [ptControl]=createCP(ptDanger, X, Y, minRadius);
            if round(initVel) > 0 && ~isempty(ptControl) % Should be limited and not rounded
                NumPoints = 4 + length(ptControl(:,1)); % Same as before + number of control points
                ptInitState = ptStart + initVel/(NumPoints-1)*[cos(initAng), sin(initAng)];
            end
            pts=[ptStart; ptInitState; ptControl; ptFinalState; ptEnd];
            
            %% Calculate new Bezier Curve
            [X, Y]=bezierCurve(pts, dt);
            
            %makeNewPath = true;
            makeNewPath = false;
        else
            makeNewPath = false;
        end
        
        %% Movement data
        V=getVelocity(pts, dt);
        A=getAcceleration(pts, dt);
        ang=getRotation(V);
        angVel=diff(ang)/dt;
        
        self.X = X; self.Y = Y; self.W = ang;
        obst.X = ptObject(:,1)*ones(1,length(self.X));
        obst.Y = ptObject(:,2)*ones(1,length(self.X));
        obst.W = zeros(1,length(self.X));
        
        time = 1; % reset time every new path
    end
    
    %% SIMULATION: Update
    if mod(count,updateTimes) == 0
        % Self
        selfPathHandle.XData = self.X;
        selfPathHandle.YData = self.Y;
        selfPosHandle.Position = [self.X(time)-robotRadius, self.Y(time)-robotRadius, 2*robotRadius, 2*robotRadius];
        selfDirHandle.XData = self.X(time);
        selfDirHandle.YData = self.Y(time);
        selfDirHandle.UData = robotRadius*cos(self.W(time));
        selfDirHandle.VData = robotRadius*sin(self.W(time));
        
        % Obstacles
        if ~isempty(obst.X)
            for j = 1:length(obst.X(:,1))
                obstPosHandle(j).Position = [obst.X(j,time)-robotRadius, obst.Y(j,time)-robotRadius, 2*robotRadius, 2*robotRadius];
                obstDirHandle(j).XData = obst.X(j,time);
                obstDirHandle(j).YData = obst.Y(j,time);
                obstDirHandle(j).UData = robotRadius*cos(obst.W(j,time));
                obstDirHandle(j).VData = robotRadius*sin(obst.W(j,time));
            end
        end
        drawnow;
    end
    
    %% Replace initial data with current data
    initVel = norm(V(:,time));
    initAng = self.W(time);
    ptStart = [self.X(time), self.Y(time)];
    
    time = time + 1;
    count = count + 1;
end

%% Functions
function [X, Y] = bezierCurve(P, dt)
% P: n by 2 matrix with control points (x, y)
% dt: distance between two adjacent points
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
end

function [CP] = createCP(obj, BZ_X, BZ_Y, minRadius)
% Assumes all objects are dangerous
CP=[];
if isempty(obj)
    return;
elseif length(obj(:,1))==1
    x=obj(1,1);
    y=obj(1,2);
    D=sqrt((BZ_X-x).^2+(BZ_Y-y).^2);
    if min(D)<minRadius
        ind=find(D==min(D),1);
        cur_x=BZ_X(ind);
        cur_y=BZ_Y(ind);
        
        vecToCurve=[cur_x, cur_y]-[x, y];
        CP=[cur_x, cur_y]+3*(minRadius-norm(vecToCurve))*vecToCurve/norm(vecToCurve);
        return;
    end
else
    disp("HELP! So much danger, I don't know what to do...YET");
end
end

function [dangerous]=findDanger(obj, BZ_X, BZ_Y, minRadius)
dangerous=[];
if isempty(obj)
    return;
end
for i=1:length(obj(:,1))
    x=obj(i,1);
    y=obj(i,2);
    D=sqrt((BZ_X-x).^2+(BZ_Y-y).^2);
    if min(D)<minRadius
        dangerous=[dangerous; x, y];
        
        % If the path goes through 2 danger zones, add the other obstacle
        % too
        for j=1:length(obj(:,1))
            if i~=j && norm(obj(i,:)-obj(j,:))<2*minRadius
                for k=1:length(BZ_X)
                    pathToObj1 = (obj(i,:)-[BZ_X(k), BZ_Y(k)])/norm(obj(i,:)-[BZ_X(k), BZ_Y(k)]);
                    pathToObj2 = (obj(j,:)-[BZ_X(k), BZ_Y(k)])/norm(obj(j,:)-[BZ_X(k), BZ_Y(k)]);
                    if dot(pathToObj1, pathToObj2)<-0.95
                        dangerous=[dangerous; obj(j,:)];
                    end
                end
            end
        end
    end
end
dangerous=unique(dangerous,'rows');
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