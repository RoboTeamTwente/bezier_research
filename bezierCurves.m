clear; clc; clf(1); clf(2);
% The purpose of this file is to make the planned path responsive to moving
% obstacles. Only for 1 obstacle atm.

%% Set variables
robotRadius = 10;
updateTimes = 30; % number of times to update the plot

dt = 1/2000; % normalized time step
minRadius=30; % circle around object
ptStart=[5 5]; % start position
ptEnd=[100 100]; % ball position
ptGoal=[130 80]; % point where the ball should go to
ptObject=[60 65]; % objects that are in the way
[nObstacles,~]=size(ptObject);

% State variables
initVel = 0;
initAng = 0.5*pi;
finalVel = 0;
finalAng = atan2(ptGoal(2)-ptEnd(2), ptGoal(1)-ptEnd(1));
orientDist = 30; % could be used to limit the angular velocity, for now to make the graph more pretty

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
[initX, initY]=bezierCurve(pts, dt);
ptDanger=findDanger(ptObject, initX, initY, minRadius); % check if circle crosses path
[nDanger,~]=size(ptDanger); % amount of danger points

%% Avoid obstacle by adding control point
[ptControl]=createCP(ptDanger, initX, initY, minRadius);
if round(initVel) > 0 && ~isempty(ptControl) % Should be limited and not rounded
    NumPoints = 4 + length(ptControl(:,1)); % Same as before + number of control points
    ptInitState = ptStart + initVel/(NumPoints-1)*[cos(initAng), sin(initAng)];
end
pts=[ptStart; ptInitState; ptControl; ptFinalState; ptEnd];

%% Calculate Bezier Curve
[X, Y]=bezierCurve(pts, dt);

%% Other stuff
% Calculate data for analysis
V=getVelocity(pts, dt);
A=getAcceleration(pts, dt);
ang=getRotation(V);
angVel=diff(ang)/dt;

self = struct('X',X,'Y',Y,'W',ang);
obst = struct('X',[],'Y',[],'W',[]);


%% Visuals
figure(1)
%set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
subplot(2,3,[1,2,4,5])
hold on
plot(initX, initY, '--', 'color', [0.6, 0.6, 0.6])
plot(X, Y, '--k')
plot(ptStart(1),ptStart(2),'g*');
plot(ptEnd(1),ptEnd(2),'r*');
if ~isempty(ptControl)
    plot(ptControl(:,1),ptControl(:,2),'b*')
end
plot([ptInitState(2,1), ptFinalState(1,1)],[ptInitState(2,2), ptFinalState(1,2)],'m*');
plot(ptGoal(1),ptGoal(2),'c*');
if ~isempty(ptObject)
    plot(ptObject(:,1),ptObject(:,2),'k*')
    circlesX = ptObject(:,1)*ones(1,100) + ones(nObstacles,1)*minRadius*cos(linspace(0,2*pi,100));
    circlesY = ptObject(:,2)*ones(1,100) + ones(nObstacles,1)*minRadius*sin(linspace(0,2*pi,100));
    plot(circlesX, circlesY, '.k', 'MarkerSize', 2)
end
ylim([-50 ptGoal(2)+50]); xlim([-50 ptGoal(1)+20]);
grid on
axis equal
if ~isempty(ptControl)
    legend('Initial path', 'Path', 'Start point', 'End Point', 'Control points', 'Orientation points', 'Goal', 'Objects', 'location', 'best')
else
    legend('Initial path', 'Path', 'Start point', 'End Point', 'Orientation points', 'Goal', 'Objects', 'location', 'best')
end
hold off

subplot(2,3,3)
T = linspace(0,2,length(V));
plot(T,V(1,:),T,V(2,:),T,sqrt(V(1,:).^2 + V(2,:).^2))
grid on
xlabel('Normalized time')
ylabel('Velocity')
legend('V_x','V_y','V_t_o_t','location','best')

subplot(2,3,6)
plot(T,A(1,:),T,A(2,:),T,sqrt(A(1,:).^2 + A(2,:).^2))
grid on
xlabel('Normalized time')
ylabel('Acceleration')
legend('A_x','A_y','A_t_o_t','location','best')

figure(2)
subplot(1,2,1)
plot(T,ang);
grid on
xlabel('Normalized time')
ylabel('Angle [rad]')

subplot(1,2,2)
plot(T(2:end),angVel);
grid on
xlabel('Normalized time')
ylabel('Angular velocity [rad/(a.u.)]')

%% SIMULATION: Make initial plot with handles
figure(10)
hold on
grid on
xlim([min(self.X)-2*robotRadius, max(self.X)+2*robotRadius]);
ylim([min(self.Y)-2*robotRadius, max(self.Y)+2*robotRadius]);
axis equal

% Self
plot(self.X, self.Y, '--k')
plot(self.X(1), self.Y(1), '.g', 'MarkerSize', 20)
plot(self.X(end), self.Y(end), '.r', 'MarkerSize', 20)

selfPosHandle = rectangle('Position', [self.X(1)-robotRadius, self.Y(1)-robotRadius, 2*robotRadius, 2*robotRadius], 'Curvature', [1, 1], 'FaceColor', 'y');
selfDirHandle = quiver(0,0,robotRadius*cos(self.W(1)),robotRadius*sin(self.W(1)), 'linewidth', 5,'color',[1 0.5 0]);

% Obstacles
if ~isempty(obst.X)
    for j = 1:length(obst.X(:,1))
        obstPosHandle(j) = rectangle('Position', [obst.X(j,1)-robotRadius, obst.Y(j,1)-robotRadius, 2*robotRadius, 2*robotRadius], 'Curvature', [1, 1], 'FaceColor', 'b');
        obstDirHandle(j) = quiver(0,0,robotRadius*cos(obst.W(j,1)),robotRadius*sin(self.W(j,1)), 'linewidth', 5,'color',[1 0.5 0]);
    end
end

%% SIMULATION: Loop through path
for i = 1:length(self.X)
    if mod(i,round(length(self.X)/updateTimes)) == 0
        % Self
        selfPosHandle.Position = [self.X(i)-robotRadius, self.Y(i)-robotRadius, 2*robotRadius, 2*robotRadius];
        selfDirHandle.XData = self.X(i);
        selfDirHandle.YData = self.Y(i);
        selfDirHandle.UData = robotRadius*cos(self.W(i));
        selfDirHandle.VData = robotRadius*sin(self.W(i));
        
        % Obstacles
        if ~isempty(obst.X)
            for j = 1:length(obst.X(:,1))
                obstPosHandle(j).Position = [obst.X(j,i)-robotRadius, obst.Y(j,i)-robotRadius, 2*robotRadius, 2*robotRadius];
                obstDirHandle(j).XData = obst.X(j,i);
                obstDirHandle(j).YData = obst.Y(j,i);
                obstDirHandle(j).UData = robotRadius*cos(obst.W(j,i));
                obstDirHandle(j).VData = robotRadius*sin(obst.W(j,i));
            end
        end
        drawnow;
    end
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