clear all; clc; close all;
% This works for all orientations
% This only works for an initial path that goes between the centers of two 
% overlapping obstacles
% Start and end points cannot be too close to the obstacles

% TODO: take in account physical limitations
% TODO: set initial velocity to zero
% TODO: multiple opponents in different configurations
% TODO: take into consideration orientation of opponent? (nice addition
% maybe)
% TODO: make calculation for more than 2 obstacles

%% Set variables
dt = 1/2000; % normalized time step
minRadius=25; % circle around object
startOrientationAngle=0.5*pi; % 0-2pi
orientationMargin=20; % distance for the orientation control point
ptStart=[5 5]; % start position
ptEnd=[100 100]; % ball position
ptGoal=[110 130]; % point where the ball should go to

% Random number stuff
a=ptStart(1)-10; b=ptEnd(1);
ptObject=[rand(1,2);rand(1,2)]*(b-a)+5;

% ptObject=[60 30; 20 50]; % objects that are in the way
[nObstacles,~]=size(ptObject);
 
%% Orientation points
h=orientationMargin*sin(startOrientationAngle);
l=orientationMargin*cos(startOrientationAngle);
ptStartOrientation=[ptStart(1)+l, ptStart(2)+h];

endOrientationDistance=sqrt((ptGoal(1)-ptEnd(1))^2+(ptGoal(2)-ptEnd(2))^2)+orientationMargin;
endOrientationAngle=atan(abs((ptGoal(2)-ptEnd(2)))/abs((ptGoal(1)-ptEnd(2))));

ptEndOrientation=ptEnd+(ptGoal-ptEnd)*[cos(pi), -sin(pi); sin(pi), cos(pi)];

%% Initial path
pts=[ptStart; ptStartOrientation; ptEndOrientation; ptEnd];
[initX, initY]=bezierCurve(pts, dt);
ptDanger=findDanger(ptObject, initX, initY, minRadius); % check if circle crosses path
[nDanger,~]=size(ptDanger); % amount of danger points

%% Points of first half of path
ptInter=createInter(nDanger, ptDanger, ptStart, ptEnd, minRadius, ptStartOrientation);

pts=[ptStart; ptStartOrientation; ptInter];
[firstX, firstY]=bezierCurve(pts, dt);
[ptInterObject]=findDanger(ptDanger, firstX, firstY, minRadius);
[ptControl]=createCP(ptInterObject, firstX, firstY, minRadius);
ptsFirst=[ptStart; ptStartOrientation; ptControl; ptInter];

% Points of second half of path
ptsSecond=[ptInter; ptEndOrientation; ptEnd];

% Bezier curves
% Calculate points on both sides of Inter Point to assure continuity
n1=length(ptsFirst(:,1)); % degree of first curve
n2=length(ptsSecond(:,1)); % degree of second curve

a=n1/n2;
b=(n1-1)/(n2-1);

ptInterOrientation1=(1+0.5/a+0.5*b)/(1+b)*ptInter+b/(2+2*b)*ptsFirst(end-1,:)-1/(2*a+2*a*b)*ptsSecond(2,:);
ptInterOrientation2=(1+a)*ptInter-a*ptInterOrientation1;

ptsFirst=[ptsFirst(1:end-1,:); ptInterOrientation1; ptsFirst(end,:)];
ptsSecond=[ptsSecond(1,:); ptInterOrientation2; ptsSecond(2:end,:)];

%% Calculate Bezier Curves
[firstX, firstY]=bezierCurve(ptsFirst, dt);
[secondX, secondY]=bezierCurve(ptsSecond, dt);

%% Other stuff
% Calculate data for analysis
firstV=getVelocity(ptsFirst, dt);
firstAng=getRotation(firstV);
secondV=getVelocity(ptsSecond, dt);
secondAng=getRotation(secondV);

% Combine halfs
X=[firstX, secondX];
Y=[firstY, secondY];
V=[firstV, secondV];
ang=[firstAng, secondAng];
angVel=diff(ang)/dt;


%% Visuals
figure(1)
set(gcf,'Position',[1367 -255 1280 1026]) % to put figure on second monitor, selina laptop
subplot(2,3,[1,2,4,5])
hold on
plot(X, Y, '--', 'color', [0.6, 0.6, 0.6])
plot(ptStart(1),ptStart(2),'g*');
plot(ptEnd(1),ptEnd(2),'r*');
if ptInter(1)~=0 && ptInter(2)~=0
    plot(ptInter(1),ptInter(2),'*','color',[1 0.5 0])
end
if ~isempty(ptControl)
    plot(ptControl(:,1),ptControl(:,2),'b*')
end
plot([ptStartOrientation(1),ptEndOrientation(1)],[ptStartOrientation(2),ptEndOrientation(2)],'m*');
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
    legend('Path', 'Start point', 'End Point', 'Intermediate point', 'Control Points', 'Orientation points', 'Goal', 'Objects', 'location', 'best')
else
    legend('Path', 'Start point', 'End Point', 'Intermediate point', 'Orientation points', 'Goal', 'Objects', 'location', 'best')
end
hold off

subplot(2,3,3)
T = linspace(0,2,length(V));
plot(T,V(1,:),T,V(2,:),T,sqrt(V(1,:).^2 + V(2,:).^2))
grid on
xlabel('Normalized time')
ylabel('Velocity')
legend('V_x','V_y','V_t_o_t')

% subplot(2,3,5)
% plot(T,ang);
% grid on
% xlabel('Normalized time')
% ylabel('Angle [rad]')

subplot(2,3,6)
plot(T(2:end),angVel);
grid on
xlabel('Normalized time')
ylabel('Angular velocity [rad/(a.u.)]')


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

function [ptInter]=createInter(nDanger, obj, ptStart, ptEnd, minRadius, ptStartOrientation)
% Decide whether to go left or right past objects
startToObj=ones(nDanger,2); angToObj=ones(nDanger,2); dist=ones(nDanger,2);

startToOr=ptStartOrientation-[ptStart(1), ptEnd(1)];
startToOr=startToOr/norm(startToOr);
for i=1:nDanger
    startToObj(i,:)=obj(i,:)-[ptStart(1), ptEnd(1)];
    startToObj(i,:)=startToObj(i,:)/norm(startToObj(i,:));
    angToObj(i)=acos(dot(startToOr, startToObj(i,:)));
    dist(i)=sqrt((norm([ptStart(end), ptEnd(end)]-obj(i,:)))^2-minRadius^2);
end
if nDanger==0
    return;
elseif nDanger==1
    ang=90-(180-90-atan(obj(2)/obj(1)));
    ptInter=[obj(1)+sin(ang)*minRadius obj(2)+cos(ang)*minRadius]; % should be better
elseif nDanger==2
    if  angToObj(1) < angToObj(2)
        % It's faster to go past object 1
        % Construct intermediary point
        if atan((obj(1,2)-ptEnd(end))/(obj(1,1)-ptStart(end))) > atan((obj(2,2)-ptEnd(end))/(obj(2,1)-ptStart(end)))
            or=-1;
        else
            or=1;
        end
        selfToObject=obj(1,:)-[ptStart(end), ptEnd(end)];
        ang=or*atan(minRadius/dist(1));
        ptInter=[ptStart(end), ptEnd(end)]+dist(1)*selfToObject/norm(selfToObject)*[cos(ang), -sin(ang); sin(ang), cos(ang)];
    else
        % It's faster to go past object 2
        % Construct intermediary point
        if atan((obj(1,2)-ptEnd(end))/(obj(1,1)-ptStart(end)))>atan((obj(2,2)-ptEnd(end))/(obj(2,1)-ptStart(end)))
            or=1;
        else
            or=-1;
        end
        selfToObject=obj(2,:)-[ptStart(end), ptEnd(end)];
        ang=or*atan(minRadius/dist(2));
        ptInter=[ptStart(end), ptEnd(end)]+dist(2)*selfToObject/norm(selfToObject)*[cos(ang), -sin(ang); sin(ang), cos(ang)];
    end
elseif nDanger>2
    disp('More than 2 objects????? I dont know man');
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
elseif length(obj(:,1))==2
    if norm(obj(1,:)-obj(2,:))<2*minRadius
        % If their circles are overlapping
        % Decide whether to go left or right past the 2 objects
        dist1=sqrt((norm([BZ_X(end), BZ_Y(end)]-obj(1,:)))^2-minRadius^2);
        dist2=sqrt((norm([BZ_X(end), BZ_Y(end)]-obj(2,:)))^2-minRadius^2);
        if  dist1 < dist2
            % It's faster to go past object 1.
            % Construct intermediary point
            if atan((obj(1,2)-BZ_Y(end))/(obj(1,1)-BZ_X(end))) > atan((obj(2,2)-BZ_Y(end))/(obj(2,1)-BZ_X(end)))
                or=-1;
            else
                or=1;
            end
            selfToObject=obj(1,:)-[BZ_X(end), BZ_Y(end)];
            ang=or*atan(minRadius/dist1);
            ptInter=[BZ_X(end), BZ_Y(end)]+dist1*selfToObject/norm(selfToObject)*[cos(ang), -sin(ang); sin(ang), cos(ang)];
        else
            % It's faster to go past object 2.
            % Construct intermediary point
            if atan((obj(1,2)-BZ_Y(end))/(obj(1,1)-BZ_X(end)))>atan((obj(2,2)-BZ_Y(end))/(obj(2,1)-BZ_X(end)))
                or=1;
            else
                or=-1;
            end
            selfToObject=obj(2,:)-[BZ_X(end), BZ_Y(end)];
            ang=or*atan(minRadius/dist2);
            ptInter=[BZ_X(end), BZ_Y(end)]+dist2*selfToObject/norm(selfToObject)*[cos(ang), -sin(ang); sin(ang), cos(ang)];
        end
        CP=ptInter;
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
                    if dot(obj(i,:)-[BZ_X(k), BZ_Y(k)], obj(j,:)-[BZ_X(k), BZ_Y(k)])<-0.95
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

V=[dXdT; dYdT];
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
end