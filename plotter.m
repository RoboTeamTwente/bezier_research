function plotter(allComb, center)
% Plot all points and line segments for debugging purposes
figure
hold on
for i = 1:length(allComb(:,1))
    n1 = allComb(i,2);
    n2 = allComb(i,3);
    ind1 = find(center(:,1)==n1);
    ind2 = find(center(:,1)==n2);
    line(center([ind1,ind2],2),center([ind1,ind2],3),'color','m');
end
plot(center(:,2),center(:,3),'.b','MarkerSize',15);
plot(center(end-1,2),center(end-1,3),'.g','MarkerSize',25);
plot(center(end,2),center(end,3),'.r','MarkerSize',25);
grid on
end