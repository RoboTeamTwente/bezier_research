function plotter(allComb, center, path, ptObject, curve, v0, nObjects)
% Plot all points and line segments for debugging purposes
hold on
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
quiver(center(end-1,2),center(end-1,3),v0.amp*cos(v0.theta),v0.amp*sin(v0.theta),'color','b','linewidth',3);

%% Points
plot(ptObject(:,1),ptObject(:,2),'xr');
viscircles([ptObject(:,1) ptObject(:,2)], 18*ones(nObjects,1));
plot(center(:,2),center(:,3),'.b','MarkerSize',15);
plot(center(end-1,2),center(end-1,3),'.g','MarkerSize',25);
plot(center(end,2),center(end,3),'.r','MarkerSize',25);
axis equal
grid on
end