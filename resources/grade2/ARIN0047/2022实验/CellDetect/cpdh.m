function [count,angle] = cpdh(imBW, angBins, radBins)
%Rotationally Invariant Contour Points Distribution Histogram
%function count = cpdh(imBW, angBins, radBins)
%
%OUTPUT
%   out = Contour Points Distribution Histogram
%
%INPUT
%   imBW = Binary image (logical type) containing the silloute or the contour of interest
%   angBins = number of angular bins
%   radBins = number of radial bins

%Detect number of objects in the image, select the biggest one only for
%computation

stats = regionprops(imBW, 'Area','BoundingBox','Orientation','PixelIdxList');

object = uint8(zeros(size(imBW)));
if numel(stats) > 1
    maxIndx = 1;
    for i = 2:numel(stats)
        if stats(i).Area > stats(i-1).Area
            maxIndx = i;
        end
    end
    stats = stats(maxIndx);
    object(stats.PixelIdxList) = 1;
else
    object = uint8(imBW);
end

object = imrotate(object,-stats.Orientation);
stats = regionprops(object,'BoundingBox','Orientation');
angle = stats.Orientation;

if numel(stats) > 1
    object = bwmorph(object, 'clean');
    stats = regionprops(object,'BoundingBox','Orientation');
end

%Get the object Only inside the image
if ceil(stats.BoundingBox(2)+stats.BoundingBox(4)) < size(object,1)
    object( ceil(stats.BoundingBox(2)+stats.BoundingBox(4)):end,:) = [];
end
if ceil(stats.BoundingBox(1)+stats.BoundingBox(3)) < size(object,2)
    object(:,ceil(stats.BoundingBox(1)+stats.BoundingBox(3)):end) = [];
end
if round(stats.BoundingBox(2)) > 1
    object(1:round(stats.BoundingBox(2))-1,:) = [];
end
if round(stats.BoundingBox(1)) > 1
    object(:,1:round(stats.BoundingBox(1))-1) = [];
end


%Get the object boundary only
object = bwmorph(object, 'remove');
stats = regionprops(object,'Centroid','PixelList');

numClean = 1;
if numel(stats) > 1
    while numel(stats) > 1
        object = bwmorph(object, 'clean');
        stats = regionprops(object,'Centroid','PixelList','Area','PixelIdxList','BoundingBox');
        numClean = numClean + 1;
        if numClean > 3
          
            maxIndx = 1;
            for i = 2:numel(stats)
                if stats(i).Area > stats(i-1).Area
                    maxIndx = i;
                end
            end
            stats = stats(maxIndx);
            object(:) = 0;
            object(stats.PixelIdxList) = 1;
            object = bwmorph(object, 'clean');
            
        end
    end
    %Get the object Only inside the image
    if ~isempty(stats)
        if ceil(stats.BoundingBox(2)+stats.BoundingBox(4)) < size(object,1)
            object( ceil(stats.BoundingBox(2)+stats.BoundingBox(4)):end,:) = [];
        end
        if ceil(stats.BoundingBox(1)+stats.BoundingBox(3)) < size(object,2)
            object(:,ceil(stats.BoundingBox(1)+stats.BoundingBox(3)):end) = [];
        end
        if round(stats.BoundingBox(2)) > 1
            object(1:round(stats.BoundingBox(2))-1,:) = [];
        end
        if round(stats.BoundingBox(1)) > 1
            object(:,1:round(stats.BoundingBox(1))-1) = [];
        end
        stats = regionprops(object,'Centroid','PixelList');
    end
end

%figure, imagesc(object);

if ~isempty(stats) && numel(stats.PixelList) > 2
    %Translate into polar coordinates
    centroid = stats.Centroid;
    cartesian = stats.PixelList;
    polar = zeros(size(cartesian));
    
    polar(:,1) = sqrt( (cartesian(:,1) - centroid(1)).^2 + (cartesian(:,2) - centroid(2)).^2 );
    polar(:,2) = atan2( cartesian(:,2) - centroid(2) , cartesian(:,1) - centroid(1) );
    
    %Spatial Partitions
    maxRo = max(polar(:,1));
    radii = maxRo/radBins;
    angles = 2*pi/angBins;
    
    count = hist3(polar,{0+radii/2:radii:maxRo-radii/2  -pi+angles/2:angles:pi-angles/2});
    
    %%%%PLOT HISTOGRAM%%%%
    %figure, bar3(count); xlabel('angles'); ylabel('radii');
    %%%%%%%%%%%%%%%%%%%%%%
    
    count = reshape(count, radBins*angBins,1);
    count = count/norm(count,2); %normalization
else
    count = zeros(radBins*angBins, 1);
end


