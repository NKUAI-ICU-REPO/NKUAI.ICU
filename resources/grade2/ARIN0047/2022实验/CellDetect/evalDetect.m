function [precision, recall, tp, fp, fn] =...
    evalDetect(x, y, xGT, yGT, weightMap, threshold, image)
%Evaluation script to match detections based on the hungarian algorithm 
%OUTPUT
%   precision, recall, tp (#true positives), fp(#false positives, fn(#false negatives)
%INPUT
%   x,y,xGT,yGT - column vectors with x and y coordinates of detections and GT dots     
%   weightMap = used to recompute the tolerance threshold to accept detections
%       base on depthMaps
%   threshold = maximum distance between detection and GT dot to be matched.
%   image = (optional) argument to visualize the matches and misses. 

depthMap = 1.0./sqrt(weightMap);

xGT_ = int32(xGT);
yGT_ = int32(yGT);

xGT_(xGT_ < 1) = 1;
yGT_(yGT_ < 1) = 1;
xGT_(xGT_ > size(weightMap,2)) = size(weightMap,2);
yGT_(yGT_ > size(weightMap,1)) = size(weightMap,1);

thresh = threshold*depthMap(sub2ind(size(depthMap),yGT_,xGT_));

dy = repmat(double(yGT), 1, size(y,1))- repmat(double(y)', size(yGT,1), 1);
dx = repmat(double(xGT), 1, size(x,1))- repmat(double(x)', size(xGT,1), 1);
dR = sqrt(dx.*dx+dy.*dy);
dR(dR > repmat(thresh, 1, size(y,1))) = +inf;
matching = Hungarian(dR);

fp = numel(x)-sum(matching(:));
fn = numel(xGT)-sum(matching(:));
tp = sum(matching(:));

precision = tp / (tp + fp);
recall = tp / (tp + fn);

if nargin == 7
    imshow(image); hold on;
    scatter(y(any(matching)),x(any(matching)),'b','o');
    scatter(y(~any(matching)),x(~any(matching)),'r','o');
    scatter(yGT(any(matching,2)),xGT(any(matching,2)),'b','x');
    scatter(yGT(~any(matching,2)),xGT(~any(matching,2)),'r','x');
    hold off;
end
