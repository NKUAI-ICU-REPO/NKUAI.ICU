function [parameters, ctrl] = setFeatures()
%To set the features and control for training/testing
%OUTPUT
%   parameters = strucutre with learning parameters
%   ctrl = structure with some control parameters
%-------------------------------------------------------------------Control
%-Parallel computing-%
ctrl.runPar = 0; %Set to activate parallel computing  
ctrl.workers = 4;

%-Some Control Parameters-%
ctrl.c = 1; %C in the optimizarion objective
ctrl.o = 2; %Slack rescaling = 1, Margin rescaling = 2.
ctrl.bias = 0.5; %Offset in the output of the classifier's score
ctrl.alpha = 0;%Controls the precision/recall from within the optimization
%(penalization cost of regions with no dot inside is 1-alpha)
ctrl.ssvm = 1;
ctrl.maxOuterIter = 2;

%------------------------------------------------------------------Features

eqHist = 0; %Equalize histogram
addArea = 1; %Adds area descriptor of the MSER
nBinsArea = 15;
areaBinType = 'log';%linear = 'lin', logarithmic = 'log'
minArea = 0.01/100;
maxArea = 2/100;
addIntHist = 1;%Adds intensity information
nBinsIntHist = 15; %Per color channel
addPos = 0; %Adds XY position
addDiffHist = 1; %Adds histogram of difference between MSER and its context
nBinsDiffHist = 8;
nDilations = 2;
nDilationScales = 2;
addShape = 1; %Adds shape descriptor
nAngBins = 12;
nRadBins = 5;
addOrientation = 0;
nBinsOrient = 8;
addEdges = 0;
addOrientGrad = 0;
nBinsOrientGrad = 10;
addBias = 1;
bias = 100;
jitterSize = 3; %Size (in pixels) of the jitter introduced to the annotations

parameters.addArea = addArea;
parameters.nBinsArea = nBinsArea;
parameters.minArea = minArea;
parameters.maxArea = maxArea;
parameters.addPos = addPos;
parameters.addIntHist = addIntHist;
parameters.nBinsIntHist = nBinsIntHist;
parameters.addDiffHist = addDiffHist;
parameters.nBinsDiffHist = nBinsDiffHist;
parameters.nDilations = nDilations;
parameters.nDilationScales = nDilationScales;
parameters.addShape = addShape;
parameters.nAngBins = nAngBins;
parameters.nRadBins = nRadBins;
parameters.addBias = addBias;
parameters.bias = bias;
parameters.eqHist = eqHist;
parameters.jitter = 1;
parameters.jitterSize = jitterSize;
parameters.areaBinType = areaBinType;
parameters.addOrientation = addOrientation;
parameters.nBinsOrient = nBinsOrient;
parameters.addOrientGrad = addOrientGrad;
parameters.nBinsOrientGrad = nBinsOrientGrad;
parameters.addEdges = addEdges;

end