function [x, y, modelInliers] = ransacLine(points, threshold)
    sampleSize = floor(0.6*size(points,1)); % number of points to sample per trial
    maxDistance = threshold; % max allowable distance for inliers
    fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
    evalLineFcn = ...   % distance evaluation function
      @(model, points) sqrt(sum((points(:, 2) - polyval(model, points(:,1))).^2,2))/length(points(:,2));

    [~, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...
      sampleSize,maxDistance);
    modelInliers = polyfit(points(inlierIdx,1), points(inlierIdx,2), 1);
%             scatter(points(inlierIdx,1), points(inlierIdx,2), 'bo')
    inlierPts = points(inlierIdx,:);
    x = [min(inlierPts(:,1)) max(inlierPts(:,1))];
    y = modelInliers(1)*x + modelInliers(2);
end