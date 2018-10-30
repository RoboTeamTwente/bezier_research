function plotter(allComb, center, path, ptObject, curve, v0, nObjects, objectRadius, fieldSize, ptStart, startOrientationAngle, endOrientationAngle, ptEnd)
% Plot all points and line segments for debugging purposes
hold on

%% Field
rectangle('Position',[-fieldSize(1)/2, -fieldSize(2)/2, fieldSize(1), fieldSize(2)],'linewidth',5,'FaceColor',[0 1 0 0.3]);
rectangle('position',[-fieldSize(1)/2-30, -fieldSize(2)/2-30, fieldSize(1)+60, fieldSize(2)+60], 'linewidth',2,'edgecolor','k');

%% Field lines
% Penalty area
penaltyWidth = 120;
penaltyLength = 240;
penaltyCoordinates = [-fieldSize(1)/2 -penaltyLength/2; -fieldSize(1)/2 penaltyLength/2; ...
    -fieldSize(1)/2 + penaltyWidth penaltyLength/2; -fieldSize(1)/2 + penaltyWidth -penaltyLength/2];
k = 3;
for i = 2:length(penaltyCoordinates(:,1))
    if k == 5
        k = 1;
    end
    plot([penaltyCoordinates(i,1); penaltyCoordinates(k,1)], [penaltyCoordinates(i,2); ...
        penaltyCoordinates(k,2)],'w-','linewidth',2);
    plot([penaltyCoordinates(i,1); penaltyCoordinates(k,1)]*-1, [penaltyCoordinates(i,2); ...
        penaltyCoordinates(k,2)]*-1,'w-','linewidth',2);
    k = k + 1;
end

% Centerlines
plot([0 0],[fieldSize(2)/2 -fieldSize(2)/2],'w-','linewidth',2);
plot([-fieldSize(1)/2 fieldSize(1)/2],[0 0],'w-','linewidth',2);

% Center arc
viscircles([0 0],100,'color','w');

% Goal
goalLength = 120;
goalWidth = 20;
goalCoordinates = [-fieldSize(1)/2 goalLength/2; -fieldSize(1)/2-goalWidth goalLength/2; ...
    -fieldSize(1)/2-goalWidth -goalLength/2; -fieldSize(1)/2 -goalLength/2];
k = 2;
for i = 1:length(goalCoordinates(:,1))-1
    plot([goalCoordinates(i,1); goalCoordinates(k,1)], [goalCoordinates(i,2); ...
        goalCoordinates(k,2)],'color',[1 0.5 0],'linestyle','-','linewidth',2);
    plot([goalCoordinates(i,1); goalCoordinates(k,1)]*-1, [goalCoordinates(i,2); ...
        goalCoordinates(k,2)]*-1,'color',[1 0.5 0],'linestyle','-','linewidth',2);
    k = k + 1;
end

rectangle('Position',[-fieldSize(1)/2, -fieldSize(2)/2, fieldSize(1), fieldSize(2)], 'linewidth',5,'EdgeColor',[0 0.6 0]);

%% Lines
for i = 1:length(allComb(:,1))
    n1 = allComb(i,2);
    n2 = allComb(i,3);
    ind1 = find(center(:,1)==n1);
    ind2 = find(center(:,1)==n2);
    line(center([ind1,ind2],2),center([ind1,ind2],3),'color',[0.5 0.5 0.5],'linestyle','--');
end
for i = 2:length(path(:,1))
    line([path(i,2), path(i-1,2)],[path(i,3), path(i-1,3)],'color','k','linestyle','--','linewidth',2)
end
plot(curve(1,:),curve(2,:),'-m','linewidth',2);
%quiver(center(end-1,2),center(end-1,3),v0.amp*cos(v0.theta),v0.amp*sin(v0.theta),'color','b','linewidth',3);

%% Points
plot(ptObject(:,1), ptObject(:,2),'xr');
viscircles([ptObject(:,1) ptObject(:,2)], objectRadius*ones(length(ptObject(:,1)),1));
plot(center(:,2),center(:,3),'.b','MarkerSize',15);
plot(center(end-1,2),center(end-1,3),'.g','MarkerSize',25); % start
plot(center(end,2),center(end,3),'.r','MarkerSize',25); % end
orientationMargin = 100;
h = orientationMargin * sin(startOrientationAngle);
l = orientationMargin * cos(startOrientationAngle);
ptStartOrientation = [ptStart(1)+l, ptStart(2)+h];
dp = ptStartOrientation - ptStart; 
quiver(ptStart(1), ptStart(2), dp(1), dp(2),0,'MaxHeadSize',1);
h = orientationMargin * sin(endOrientationAngle);
l = orientationMargin * cos(endOrientationAngle);
ptEndOrientation = [ptEnd(1)+l, ptEnd(2)+h];
dp = ptEndOrientation - ptEnd; 
quiver(ptEnd(1), ptEnd(2), dp(1), dp(2),0,'MaxHeadSize',1);
axis equal
grid on
end