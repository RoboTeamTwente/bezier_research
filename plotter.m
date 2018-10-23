function plotter(allComb, center, path, ptObject)
% Plot all points and line segments for debugging purposes
hold on
for i = 1:length(allComb(:,1))
    n1 = allComb(i,2);
    n2 = allComb(i,3);
    ind1 = find(center(:,1)==n1);
    ind2 = find(center(:,1)==n2);
    line(center([ind1,ind2],2),center([ind1,ind2],3),'color',[0.5 0.5 0.5],'linestyle','--');
end
for i = 2:length(path(:,1))
    line([path(i,2), path(i-1,2)],[path(i,3), path(i-1,3)],'color','m','linestyle','-','linewidth',2)
end
plot(ptObject(:,1),ptObject(:,2),'or','MarkerSize',20);
plot(center(:,2),center(:,3),'.b','MarkerSize',15);
plot(center(end-1,2),center(end-1,3),'.g','MarkerSize',25);
plot(center(end,2),center(end,3),'.r','MarkerSize',25);
grid on
end