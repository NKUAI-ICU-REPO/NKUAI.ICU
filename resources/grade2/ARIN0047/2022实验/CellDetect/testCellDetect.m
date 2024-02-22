function [centers, mask, dots, prediction, img, sizeMSER, r, gt, nFeatures]...
    = testCellDetect(w,dataset,imNum,parms,ctrl,verbosity)
%Detect cells in an image given the W vector
%OUTPUT
%   centers = logical image with centroids of the regions selected
%   mask = logical image with the regions selected
%   dots = vector with the centroids of the regions selected
%   prediction = score of each MSER in r obtained as <w,X>
%   img = original image
%   sizeMSER = size (in pixels) of each MSER in r
%   gt = vector with the gt annotations
%   nFeatures = total number of features used
%INPUT
%   w = vector learned with the structural-SVM
%   dataset = dataset identifier
%   parms and ctrl = structures set in setFeatures
%   verbosity = to show figures of the results. 
%       0 doesn't show anything
%       1 shows the image with the regions boundaries and centroids
%       2 shows all candidate regions 

%--------------------------------------------------------------Encode Image
withGT = 0;
additionalU = 0;

[files, imExt, dataFolder, outFolder, mserParms] = loadDatasetInfo(dataset);

if exist([outFolder '/feats_' files{imNum} '_test.mat'],'file') == 0
    
    [img, gt, X, ~, r, ell, MSERtree, ~, sizeMSER, nFeatures] =...
        encodeImage(dataFolder, files{imNum}, imExt, withGT, parms, mserParms);
    %save([outFolder '/feats_' files{imNum} '_test.mat']...
    %    ,'img', 'gt', 'X', 'r', 'ell', 'sizeMSER', 'MSERtree', 'nFeatures');
else
    load([outFolder '/feats_' files{imNum} '_test.mat']);
end

%-------------------------------------------------------Evaluate Hypotheses
prediction = w'*X';
biasedPrediction = prediction  + ctrl.bias;
%-----------------------------------------------------------------Inference
[mask, labels] = PylonInference(img, biasedPrediction',...
    sizeMSER, r, additionalU, MSERtree);
mask = logical(mask);

%---------------------------------------------------------------Get Centres
regions = regionprops(mask, 'Centroid','PixelList');
nRegions = numel(regions);
centers = zeros(size(mask), 'uint8');

for i = 1:nRegions
    dots = round(regions(i).Centroid);
    centers(dots(2), dots(1)) = 255;
end

[a b] = find(centers > 0);
dots = [b a];

%------------------------------------------------Post processing the masks?
%mask = bwmorph(mask, 'close');
%---------------------------------------------------------------Plot Result

if verbosity > 0
    
    orImg = imread([dataFolder '/' files{imNum} '.' imExt] ,imExt);
    
    if exist([dataFolder '/' files{imNum} '.mat'],'file') == 0
        gt = [];
    else
        gt = load([dataFolder '/' files{imNum} '.mat']);
        inGT = fieldnames(gt);
        gt = gt.(inGT{1});
    end
    
    if size(orImg,3) > 3
        orImg = orImg(:,:,1:3);
    end
    
    screen_size = get(0,'ScreenSize');
    f1 = figure('Name','Detected Cells'); 
    imshow(orImg);
    title({'Extremal region boundaries = green/red. Centroids = yellow'},'FontSize',12);
    
    set(f1,'Position', [0 0 screen_size(3)/2 screen_size(4)]);
    hold on;
    
    [B,L,N,A] = bwboundaries(mask);
    
    for i=1:numel(B)
        line(B{i}(:,2),B{i}(:,1),'Color','r','LineWidth',3, 'LineStyle','-');
    end
    for i=1:numel(B)
        line(B{i}(:,2),B{i}(:,1),'Color','g','LineWidth',3, 'LineStyle','--');
    end
    
    hold on;
    plot(dots(:,1), dots(:,2),'xy','LineWidth',5,'MarkerSize',5)
    if ~isempty(gt) && verbosity > 1
        plot(gt(:,1), gt(:,2),'xb','LineWidth',2,'MarkerSize',4)
        title({'Extremal region boundaries = green/red. Centroids = yellow',...
            'Ground truth = blue'},'FontSize',12);
    end
    hold off;
end


if verbosity > 1
    figure('Name','Result Details'), imshow(img); hold on;
    title('All regions (fitted Ellipses). G = selected, B = score > 0, R = score < 0',...
        'FontSize',12);
    for k = 1:size(r,1)
        
        if biasedPrediction(k) > 0
            if labels(k)
                vl_plotframe(ell(:,k),'color','g');
            else
                vl_plotframe(ell(:,k),'color','b');
            end
        else
            vl_plotframe(ell(:,k),'color','r') ;
        end
        
    end
end

%--Good quality export
%     screen_size = get(0,'ScreenSize');
%     f1 = figure(1);
%     set(f1,'Position', [0 0 screen_size(3) screen_size(4)]);
%     imshow(orImg);
%     hold on;
% 
%     [B,L,N,A] = bwboundaries(mask);
% 
%     for i=1:numel(B)
%         line(B{i}(:,2),B{i}(:,1),'Color','g','LineWidth',4, 'LineStyle','-');
%     end
%     for i=1:numel(B)
%         line(B{i}(:,2),B{i}(:,1),'Color','r','LineWidth',3, 'LineStyle','--');
%     end
% 
%     if ~isempty(gt)
%         plot(gt(:,1), gt(:,2),'or','LineWidth',5,'MarkerSize',3)
%     end
%     plot(dots(:,1), dots(:,2),'xb','LineWidth',5,'MarkerSize',5)
%     hold off;
% 
%     export_fig([outFolder '/' files{imNum}],'-transparent','-q100','-m1.5','-a2','-png');

end