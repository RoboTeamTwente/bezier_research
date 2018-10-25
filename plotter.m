function plotter(allComb, center, path, ptObject, curve, v0, nObjects, objectRadius, fieldSize, ptStart, startOrientationAngle)
% Plot all points and line segments for debugging purposes
hold on
%% Field
rectangle('Position',[-fieldSize(1)/2, -fieldSize(2)/2, fieldSize(1), fieldSize(2)],'linewidth',5,'EdgeColor',[0 0.6 0],'FaceColor',[0 1 0 0.3]);

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
viscircles([ptObject(:,1) ptObject(:,2)], objectRadius*ones(nObjects,1));
plot(center(:,2),center(:,3),'.b','MarkerSize',15);
plot(center(end-1,2),center(end-1,3),'.g','MarkerSize',25); % start
plot(center(end,2),center(end,3),'.r','MarkerSize',25); % end
% for i = 1:4
%     if i == 4
%         x = [ptObject(i,1); ptObject(1,1)];
%         y = [ptObject(i,2); ptObject(1,2)];
%     else
%         x = [ptObject(i,1); ptObject(i+1,1)];
%         y = [ptObject(i,2); ptObject(i+1,2)];
%     end
%     plot(x,y,'color',[0.5 0.5 0.5],'linestyle','--');
% end
orientationMargin = 100;
h = orientationMargin * sin(startOrientationAngle);
l = orientationMargin * cos(startOrientationAngle);
ptStartOrientation = [ptStart(1)+l, ptStart(2)+h];
dp = ptStartOrientation - ptStart; 
quiver(ptStart(1), ptStart(2), dp(1), dp(2),0,'MaxHeadSize',1);
axis equal
grid on
end