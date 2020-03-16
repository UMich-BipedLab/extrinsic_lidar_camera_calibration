%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [x, y, modelInliers, inlierPts] = ransacLineWithInlier(points, threshold, fraction)
    if nargin < 3
        sampleSize = floor(0.6*size(points,1)); % number of points to sample per trial
    else
        sampleSize = floor(fraction*size(points,1));
    end
    maxDistance = threshold; % max allowable distance for inliers
    fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
    evalLineFcn = ...   % distance evaluation function
      @(model, points) sqrt(sum((points(:, 2) - polyval(model, points(:,1))).^2,2))/length(points(:,2));

    [~, inlierIdx] = ransac(points,fitLineFcn,evalLineFcn, ...
                            sampleSize,maxDistance, ...
                            'MaxSamplingAttempts', 1000,...
                            'MaxNumTrials', 3000, ...
                            'Confidence', 99);
    modelInliers = polyfit(points(inlierIdx,1), points(inlierIdx,2), 1);
%             scatter(points(inlierIdx,1), points(inlierIdx,2), 'bo')
    inlierPts = points(inlierIdx, :);
    x = [min(inlierPts(:,1)) max(inlierPts(:,1))];
    y = modelInliers(1)*x + modelInliers(2);
end