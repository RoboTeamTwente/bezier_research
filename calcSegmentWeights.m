function [segment_weights] = calcSegmentWeights(allComb, center, v0, vf, ptObject, robotDiameter)
factor = 10; % random factor to balance the importance of the orientation

startIndex = find(allComb(:,2)==6493,1); % index of first occurence of start ID
endIndex = find(allComb(:,2)==6494,1); % index of first occurence of end ID
startPolygon = allComb(allComb(:,2)==6493,3); % ID's of all points connected with start
endPolygon = allComb(allComb(:,2)==6494,3); % ID's of all points connected with end

segment_weights = ones(length(allComb(:,1)),1);
%% End weights
for i = 1:length(endPolygon)
    ID = endPolygon(i);
    ind = find(center(:,1)==ID,1);
    if ~isempty(ind)
        endToNode = center(ind,2:3) - center(end,2:3);
        angleToNode = vf.theta-atan2(endToNode(2),endToNode(1));
        segment_weights(endIndex+i-1) = factor*cos(angleToNode/2)^2; % if angle is 0, result is 1. if angle is pi, result is 0
    end
end

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

%% Rest of the weights
normal_dist = 2*robotDiameter; % distance at which the weight is equal to 2.
for i = 1:length(allComb(:,1))
    ID1 = allComb(i,2);
    ID2 = allComb(i,3);
    if ID1 ~= 6493 && ID1 ~= 6494 && ID2 ~= 6493 && ID2 ~= 6494
        distances = zeros(1,length(ptObject(:,1)));
        linepoint1 = center(center(:,1)==ID1,2:3);
        linepoint2 = center(center(:,1)==ID2,2:3);
        if ~isempty(linepoint1) && ~isempty(linepoint2)
            for j = 1:length(ptObject(:,1))
                distances(j) = distancePointToLine(ptObject(j,:), linepoint1, linepoint2);
            end
            segment_weights(i) = 1 + normal_dist/min(distances); % goes as 1/x and converges to 1
        end
    end
end

%% FUNCTIONS
    function [d] = distancePointToLine(point, linepoint1, linepoint2)
        % use vector formulation
        % line: x = a+t*n
        % point: p
        a = linepoint1;
        p = point;
        n = (linepoint2-linepoint1)/norm(linepoint2-linepoint1);
        
        t = -dot(a-p, n);
        if t < 0
            % linepoint1 is closest
            d = sqrt((point(1)-linepoint1(1))^2+(point(2)-linepoint1(2))^2);
        elseif t > norm(linepoint2-linepoint1)
            % linepoint2 is closest
            d = sqrt((point(1)-linepoint2(1))^2+(point(2)-linepoint2(2))^2);
        else
            % closest point is on the line segment
            d = norm(a-p+t*n);
        end
    end
end