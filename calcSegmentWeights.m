function [segment_weights] = calcSegmentWeights(allComb, center, v0, vf)
factor = 10; % random factor to balance the importance of the orientation

startIndex = find(allComb(:,2)==6493,1); % index of first occurence of start ID
endIndex = find(allComb(:,2)==6494,1); % index of first occurence of end ID
startPolygon = allComb(allComb(:,2)==6493,3); % ID's of all points connected with start
endPolygon = allComb(allComb(:,2)==6494,3); % ID's of all points connected with end

segment_weights = ones(length(allComb(:,1)),1);
%% End weights
% for i = 1:length(endPolygon)
%     ID = endPolygon(i);
%     endToNode = center(ID,2:3) - center(end,2:3);
%     angleToNode = vf.theta-atan2(endToNode(2),endToNode(1));
%     segment_weights(endIndex+i-1) = factor*cos(angleToNode/2)^2; % if angle is 0, result is 1. if angle is pi, result is 0
% end

%% Start weights
for i = 1:length(startPolygon)
    ID = startPolygon(i);
    ind = find(center(:,1)==ID,1);
    if ~isempty(ind)
        startToNode = center(ind,2:3) - center(end-1,2:3);
        angleToNode = v0.theta-atan2(startToNode(2),startToNode(1));
        segment_weights(startIndex+i-1) = factor*sin(angleToNode/2)^2; % if angle is 0, result is 0. if angle is pi, result is 1
    end
end
end