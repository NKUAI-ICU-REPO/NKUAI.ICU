function [mask,labels, MSERtree, idMask] =...
    PylonInference(img, prediction, sizeMSER, r, additionalU, MSERtree)
%Picks the best subset of extremal regions in 'r' using the pylon inference
%code.
%OUTPUT
%   mask = logical mask of regions labeled as 1
%   labels = labels of the regions in r; labels(i) = 1 if r(i) has been picked,
%       and 0 otherwise
%   MSERtree = tree structure containing of the MSERs in r
%   idMask = similar to mask, but contains indexes for many classes (if
%       available)
%INPUT
%   img = grayscale image
%   prediction = score of each MSER in r
%   sizeMSER = size (in pixels) of each region in r
%   r = MSERs of img as given by vl_mser
%   MSERtree = forest of MSER trees computed with buildPylonMSER

largeVal = -50;
hardUVal = 50;
mask = zeros(size(img),'uint8');
idMask = zeros(size(img));
svmTH = 0.001;

if nargin < 7
     MSERtree = buildPylonMSER(img,r,sizeMSER);
end

if nargin < 6
    additionalU = 0;
end

%hold on; plot(gt(:,1), gt(:,2),'*b','LineWidth',2)

labels = logical(false(size(r)));
for k = 1:numel(MSERtree.forest)
    treeInfo =  MSERtree.references{k};
    treeInfo = [treeInfo largeVal*ones(size(treeInfo,1),1)];
    %Add predictions
    for i = 1:size(treeInfo,1)
        if treeInfo(i) == 0
            continue;
        else
            elemPrediction = r == treeInfo(i);
            treeInfo(i, 2) = prediction(elemPrediction);
        end
    end

    hardU = [];
    if additionalU
        if numel(find(treeInfo(1:MSERtree.nLeafs(k),2) > 0)) > 0 %If there is any GT in the leafs in this tree
            hardU = [zeros(1,MSERtree.nLeafs(k)) ; ...
                hardUVal*(treeInfo(1:MSERtree.nLeafs(k),2)>0)'];
        end
    end
    
    if isempty(MSERtree.forest{k})
        
        [response,winner] = max(treeInfo(:,2));
        if response >= svmTH
            auxMask = uint8(zeros(size(img,1), size(img,2)));
            sel = vl_erfill(img,treeInfo(winner,1)) ;
            auxMask(sel) = 1;
            auxMask = bwmorph(auxMask, 'close');
            sel = find(auxMask == 1);
            mask(sel) = 1;
            correspondance = find(r == treeInfo(winner,1));
            labels(correspondance) = 1;
            idMask(sel) = 1*correspondance;
        end
        
    else
        V = zeros(3,0);
        mappedLabels = pylonInference1Class( MSERtree.nLeafs(k),...
            MSERtree.forest{k}, -1*(treeInfo(:,2)-svmTH)', V, -1*hardU);
        mappedLabels = pylonConvertLabels(mappedLabels,...
            MSERtree.forest{k},  MSERtree.nLeafs(k));
        %           finalLabels = false(size(mappedLabels));
        %asd = uint8(zeros(size(img,1), size(img,2)));
        for i=1:length(mappedLabels)
            
            if mappedLabels(i)
                if treeInfo(i,1) ~= 0
                    auxMask = uint8(zeros(size(img,1), size(img,2)));
                    sel = vl_erfill(img,treeInfo(i,1)) ;
                    auxMask(sel) = 1;
                    auxMask = bwmorph(auxMask, 'close');
                    sel = find(auxMask == 1);
                    mask(sel) = 1;
                    %figure, imagesc(mask)
                    correspondance = find(r == treeInfo(i,1));
                    labels(correspondance) = 1;
                    idMask(sel) = 1*correspondance;
                end
            end
        end
    end
    % figure, imagesc(mask)
end


end