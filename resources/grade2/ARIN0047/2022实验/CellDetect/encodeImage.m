function [img, gt, X, Y, r, ell, MSERtree, gtInMSER, sizeMSER, nFeatures] =...
    encodeImage(dataFolder, imName, imExt, withGT, parms, mserParms)
%Collects the MSERs in an image and encodes individually with the selected 
%features.
%
%OUTPUT
%   img = original image
%   gt = ground truth annotations (if available) 
%   X = feature vector
%   Y = labels (if training data)
%   r,ell = MSER results from vl_mser
%   MSERtree = tree structure from the MSERs in r
%   gtInMSER = number of ground truth annotations inside each MSER
%   sizeMSER = size of each MSER
%   nFeature = total number of features used in the encoding
%
%INPUT
%   dataFolder = directory that contains the image
%   imName = image name
%   imExt = image extension
%   withGT = indicates if the image is for training (1) or testing (0)
%   parms = structure with the selected parameters as set in setFeatures
%   mserParms = parameters for the mser computation set in loadDatasetInfo

minArea = parms.minArea; 
maxArea = parms.maxArea; 
sigma = 0.5;
gaussWinSize = 8;
gaussRange = -gaussWinSize/2:gaussWinSize/2;
gaussFilter = 1/(sqrt(2*pi)*sigma)*exp(-0.5*gaussRange.^2/(sigma^2));
gaussFilter = gaussFilter / sum(gaussFilter); % Normalize.
jitterPos = [0 0 ; -1 -1 ; -1 0 ; -1 1 ; 0 -1 ; 0 1 ; 1 -1 ; 1 0 ; 1 1];
jitterPos = jitterPos*parms.jitterSize;
if strcmp(parms.areaBinType,'log')
    areaBins = logspace(log10(minArea),log10(maxArea),parms.nBinsArea);
else
    areaBins = linspace(minArea,maxArea,parms.nBinsArea);
end

img = imread([dataFolder '/' imName '.' imExt] ,imExt);

if exist([dataFolder '/' imName '.mat'],'file') == 0
    if withGT
        error(['Did not find annotations for training image ' imName]);
    else
        gt = [];
    end
else
    gt = load([dataFolder '/' imName '.mat']);
    inGT = fieldnames(gt);
    gt = gt.(inGT{1});
end

if size(img,3) >= 3
    colorImg = img(:,:,1:3);
    img = rgb2gray(colorImg);
else
    colorImg = [];
end

if parms.eqHist
    img = histeq(img);
end

if parms.addEdges
    edgeImg = edge(img,'canny');
else
    edgeImg = [];
end

if parms.addOrientGrad
    [gradImg, orientGrad] = canny(img,0.5);
else
    gradImg = [];
    orientGrad = [];
end

nFeatures = parms.nBinsArea*parms.addArea + 2*parms.addPos...
    + (size(img,3)+3*~isempty(colorImg))*parms.nBinsIntHist*parms.addIntHist...
    + parms.nBinsDiffHist*parms.addDiffHist*parms.nDilationScales...
    + parms.nAngBins*parms.nRadBins*parms.addShape + parms.addBias...
    + parms.addOrientation*parms.nBinsOrient + parms.addEdges ...
    + parms.addOrientGrad*parms.nBinsOrientGrad;


parms.nFeatures = nFeatures;
numPixels = size(img,1)*size(img,2);
minPix = mserParms.minPix;
if isempty(mserParms.maxPix)
    maxPix = numPixels/4;
else
    maxPix = mserParms.maxPix;
end

%Compute MSERs
[r,ell] = vl_mser(img,'MaxArea', maxPix/numPixels,'MinArea',...
    minPix/numPixels,'MaxVariation',2,'MinDiversity',0.2,...
    'Delta',1,'BrightOnDark',mserParms.bod,'DarkOnBright',mserParms.dob);
ell = ell([2 1 5 4 3],:);

%feature Vector
X = zeros(length(r), nFeatures);
%labels
Y = zeros(length(r)+parms.jitter*8*length(r),1);
%Sizes
sizeMSER = zeros(length(r), 1);

if withGT %Ground Truth available = train image
    
    %Number of GT points contained within the MSER
    gtInMSER = uint8(zeros(length(r), 1 + 8*parms.jitter));
    
    parfor k = 1:length(r) %Lets check every MSER
        %clf ;
        mask = uint8(zeros(size(img,1), size(img,2)));
        
        sel = vl_erfill(img,r(k)) ;
        mask( sel ) = 1;
        %         mask = bwmorph(mask, 'close');
        %         sel = find(mask == 1);
        
        statsMSER = regionprops(mask, 'Centroid','Area');
        
        if numel(statsMSER) > 1;
            %disp('2 elem');
            mask = bwmorph(mask, 'close',2);
            sel = find(mask == 1);
            statsMSER = regionprops(mask, 'Centroid');
        end
        
        sizeMSER(k) = numel(sel);
        
        if isempty(sel) || numel(sel) < minPix/2;
            X(k,:) = zeros(1, nFeatures);
            gtInMSER(k,:) = 0;
        else
            X(k,:) = encodeMSER(img, colorImg, edgeImg, gradImg,...
                orientGrad, sel, ell(:,k), parms);
            %X(k,1:parameters.nBinsArea) = hist(X(k,1),areaBins);
            
            %GT evaluation
            j = 1 + 8*parms.jitter;
            auxgtInMSER = zeros(1,j);
            while j>0
                
                %Apply the jitter--
                annot = zeros(size(img,1), size(img,2),'uint8');
                
                outCol = gt(:,1) + jitterPos(j,1) > size(img,2) |...
                    gt(:,1) + jitterPos(j,1) < 1;
                
                outRow = gt(:,2) + jitterPos(j,2) > size(img,1) |...
                    gt(:,2) + jitterPos(j,2) < 1;
                
                inJitter = find((outCol | outRow) ~= 1);
                
                jGT = gt;
                jGT(inJitter,1) =  jGT(inJitter,1) + jitterPos(j,1);
                jGT(inJitter,2) =  jGT(inJitter,2) + jitterPos(j,2);
                
                annot(sub2ind(size(img), jGT(:,2), jGT(:,1))) = 1;
                %----------------
                
                auxgtInMSER(j) = numel(find((mask & annot) == 1));
                j = j - 1;
            end
            gtInMSER(k,:) = auxgtInMSER;
        end
    end
    
    %Area quantization. Or Codebook Building in General
    %     maxArea = max(X(:,1));
    %     minArea = min(X(X(:,1)~=0,1));
    %    areaBins = logspace(log10(minArea),log10(maxArea),nBinsArea);
    
    %Lets build the MSERtree right here
    MSERtree = buildPylonMSER(img,r,sizeMSER);
    
    if parms.addArea
        for k = 1:length(r);
            if parms.addArea
                X(k,1:parms.nBinsArea) = conv(hist(X(k,1),areaBins),...
                    gaussFilter,'same');%areaHist;
            end
        end
    end
    
    %Repeat feature matrix to add examples with jittered GT
    if parms.jitter
        X = repmat(X,[9 1]);
    end
    
    %Get all labels for all classes based on the number of GT inside each Extremal Region
    idx = 1;
    for j = 1:1+parms.jitter*8
        for k = 1:length(r)
            
            if gtInMSER(k,j) == 1 % && gtInMSERcentre(k) == 1
                temp = 1;
                %costMSER(k,class) = 0;
            else
                temp = -1;
            end
            
            Y(idx) = temp;
            idx = idx + 1;
        end
    end
    gtInMSER = reshape(gtInMSER,length(r)+parms.jitter*8*length(r),1);
    
else %No ground Truth = test image
    gtInMSER = 0;
    Y = 0;
    
    parfor k = 1:size(r,1) %Lets check every MSER
        
        sel = vl_erfill(img,r(k)) ;
        
        sizeMSER(k) = numel(sel);
        if isempty(sel) %|| numel(sel) < minPix/2;
            X(k,:) = zeros(1,nFeatures);
        else
            X(k,:) = encodeMSER(img, colorImg, edgeImg, gradImg,...
                orientGrad, sel, ell(:,k), parms);
        end
        %X(k,1:parameters.nBinsArea) =  hist(X(k,1),areaBins); 
    end
    
    MSERtree = buildPylonMSER(img,r,sizeMSER);
    
    if parms.addArea
        for k = 1:length(r);
            
            if parms.addArea
                X(k,1:parms.nBinsArea) = conv(hist(X(k,1),areaBins),...
                    gaussFilter,'same');%areaHist;
            end
            
        end
    end
    
end

end
