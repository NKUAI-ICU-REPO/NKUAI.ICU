%Would this work on my data?
%
%This script should give you a good selection of candidates objects (with 
%very high recall) in an image of your application. If it does, the method
%based on non-overlapping extremal regions could be applicable. 
%
%Images need to be UINT8 to work with VLfeat's MSER function. If it is not,
%some good nomalization could be required.
%
%Other pre-processing can also be useful. For example, in noisy images, some
%smoothing can help you obtain more regular candidate regions. 
%
%The setup of the MSER algorithm allows to choose the min and max sizes of 
%the MSER to be considered, and Bright or Dark regions (see help vl_mesr). 
%This configuration can be very general, i.e. minPxiels = 1, 
%maxPixles = numPixels, BoD = 1, DoB = 1. 
%However, roughly modifying these parameters can considerably speed up the 
%cell detector.

%Check for VL_feat
if ~exist('vl_setup','file')
    error('To run this example you also need to install the vl_feat toolbox.');
end

%Load an Image
imgFull = imread('/phasecontrast/trainPhasecontrast/im01.pgm');

%Smooth the image
img = imgFull;
%img = uint8(vl_imsmooth(double(imgFull),1));

%Setup for the MSER algorithm
numPixels = size(img,1)*size(img,2);
minPixels = 20;
maxPixels = 10000;
BoD = 0; %Bright on Dark
DoB = 1; %Dark on Bright 

%Convert to gray-scale, UINT8
if size(img,3) == 3
    img = rgb2gray(img);
end

imgClass = class(img);

if ~strcmp(imgClass,'uint8')
    img = uint8(img);
end

%Compute MSERs
[r,ell] = vl_mser(img,'MaxArea',maxPixels/numPixels,'MinArea',...
    minPixels/numPixels,'MaxVariation',2,'MinDiversity',0.1,...
    'Delta',1, 'BrightOnDark',BoD, 'DarkOnBright',DoB);
ell = ell([2 1 5 4 3],:);

%Get the boundaries and plot
boundary = zeros(size(img), 'uint8');
figure;
subplot(1,2,1), imshow(imgFull); 
title('Ellipses fitted to MSERs');
for k = 1:size(r,1)
    sel = vl_erfill(img,r(k)) ;
    mask = zeros(size(img), 'uint8');
    mask(sel) = 255;
    mask = bwmorph(mask, 'remove');
    boundary(mask == 1) = 255;
    vl_plotframe(ell(:,k),'color','g') ;
end

R = img;
G = img;
B = img;

R(boundary == 255) = 0;
G(boundary == 255) = 255;
B(boundary == 255) = 0;

img = cat(3,R,G,B);
subplot(1,2,2),imshow(img);
title('Boundaries');
