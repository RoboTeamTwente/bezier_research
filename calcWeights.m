function [center_weights] = calcWeights(allComb, center, v0, vf)
startPolygon = allComb(allComb(:,2)==6493,3); % ID's of all points connected with start
endPolygon = allComb(allComb(:,2)==6494,3); % ID's of all points connected with end

center_weights = ones(length(center(:,1)),1);
%% Start weights
for i = 1:length(startPolygon)
    ID = startPolygon(i);
    startToNode = center(ID,2:3) - center(end-1,2:3);
    angleToNode = v0.theta-atan2(startToNode(2),startToNode(1));
    center_weights(ID) = sin(angleToNode/2)^2; % if angle is 0, result is 0. if angle is pi, result is 1
end

%% End weights
for i = 1:length(endPolygon)
    ID = endPolygon(i);
    endToNode = center(ID,2:3) - center(end,2:3);
    angleToNode = vf.theta-atan2(endToNode(2),endToNode(1));
    center_weights(ID) = 10*cos(angleToNode/2)^2; % if angle is 0, result is 1. if angle is pi, result is 0
end

% Set the weight of the end node to a low value, so that it goes to the end
% point when the angle is quite okay
center_weights(end) = 0.2;
end