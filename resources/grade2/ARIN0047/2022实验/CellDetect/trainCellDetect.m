function w = trainCellDetect(dataset,ctrl,parameters)
%Learn a structured output SVM classifier to detect cells based on
%non-overlapping extremal regions
%OUTPUT
%   w = weight vector learned by the structural-SVM
%INPUT
%   dataset = dataset intentifier
%   ctrl = structure of control parameters defined in setFeatures()
%   parameters = learning parameters defined in setFeatures()

%------------------------------------------------------------Some Variables
done = 0;
additionalU = 0;
outerIter = 0;

[files, imExt, dataFolder, outFolder, mserParms] = loadDatasetInfo(dataset);

while ~done
    
    patterns = {} ;
    labels = {} ;
    p = numel(files); %nImages
    
    %-----------------------------------------------------Select Classifier
    
    if exist([outFolder '/wBinary.mat'],'file') == 0
        disp('--  Training binary classifiers  --');
        disp(' ');
        structOut = 0;
        current = 1; %Auxiliar index for patterns and labels
    else
        load([outFolder '/wBinary.mat']);
        disp('Binary classifiers loaded');
        disp(' ');
             
        structOut = 1;
        outerIter = outerIter + 1;
        
        switch outerIter                
            case ctrl.maxOuterIter
                disp(' ');
                disp(['--  Training Structured Output with alpha = ' num2str(ctrl.alpha) '  --']);
                disp(' ');
            otherwise
                disp(' ');
                disp('-- Training Structured Output for GT regeneration  --');
                disp(' ');
        end
        
    end
    %--------------------------------------------Collect trainning examples
    withGT = 1;
    
    for d = 1:numel(files)
        
        disp(['Processing image ' num2str(d) '/' num2str(numel(files))])
        
        clear img gt X Y r sizeMSER gtInMSER nFeatures MSERtree
        
        if exist([dataFolder '/feats_' files{d} '.mat'],'file') == 0
            [img, gt, X, Y, r, ell, MSERtree, gtInMSER, sizeMSER, nFeatures] = ...
                encodeImage(dataFolder, files{d}, imExt, withGT, parameters, mserParms);
            save([dataFolder '/feats_' files{d} '.mat']...
                ,'img', 'gt', 'X', 'Y', 'r', 'ell', 'gtInMSER', 'sizeMSER', 'MSERtree', 'nFeatures');
        else
            load([dataFolder '/feats_' files{d} '.mat']);
        end
        
        %         imshow(img); hold on, plot(gt(:,1), gt(:,2), '*r');
        %         pause;
        
        if structOut == 1 %If Structured output
            
            X = X(1:length(r),:);
            Y = Y(1:length(r),:);
            gtInMSER = gtInMSER(1:length(r),:);
            
            Y( Y == -1 ) = 0;
            w = [];
            if exist([outFolder '/wHistory.mat'],'file') == 0
                load([outFolder '/wBinary.mat']);
            else
                load([outFolder '/wHistory.mat']);
            end
            
            prediction = (w'*X' + 1).*Y'-0.01;
            
            if exist('MSERtree','var')
                [mask,Y, MSERtree] = PylonInference(img, prediction',...
                    sizeMSER, r, additionalU, MSERtree);
            else
                [mask,Y, MSERtree] = PylonInference(img, prediction',...
                    sizeMSER, r, additionalU);
                save([dataFolder '/feats_' files{d} '.mat'],'MSERtree', '-append');
            end
            
            patterns{p} = [X p*ones(size(X,1),1)]; 
            labels{p} = Y;
            
            %classes{p} = Y.*I';
            %cost{p} = costMSER;
            rVector{p} = r;
            images{p} = img;
            placedDots{p} = size(gt,1);
            dotsInside{p} = gtInMSER;
            sizesR{p} = sizeMSER;
            trees{p} = MSERtree;
            p = p - 1;
            
        else
            
            for p = current+length(r)-1:-1:current
                patterns{p} = X(p-current+1,:)';
                labels{p}   = Y(p-current+1);
            end
            current = current + length(r);
        end
        %hold on; plot(gt(:,1), gt(:,2),'*b','LineWidth',2)
    end
    
    clear img annot r ell
    
    %-------------------------------------------------------------Train SVM
    w = [];
    
    parm.verbose = 0 ;
    
    if structOut
        parm.dimension = nFeatures;
        parm.patterns = patterns;
        parm.featureFn = @featureSOCB ;
        parm.lossFn = @lossSOCB ;
        parm.constraintFn  = @constraintSOCB ;
        parm.labels = labels;
        parm.gtInsideR = dotsInside;
        parm.rVector = rVector;
        parm.images = images;
        parm.sizes = sizesR;
        parm.trees = trees;
        parm.placedDots = placedDots;
        parm.alpha = ctrl.alpha;
        disp('Running Cutting Plane Algorithm')
        C = ctrl.c;
        model = svm_struct_learn([' -c ' num2str(C) ' -o ' num2str(ctrl.o) ' -v 2 '], parm);
        w = [w model.w];        
    else
        parm.dimension = nFeatures;
        parm.patterns = patterns;
        parm.labels = labels;
        parm.lossFn = @lossCB;
        parm.constraintFn  = @constraintCB;
        parm.featureFn = @featureCB;
        disp(['Training Binary classifier']);
        C = 1;
        model = svm_struct_learn([' -c ' num2str(C) ' -o ' num2str(ctrl.o) ' -v 1 '], parm);
        w = [w model.w] ;
        save([outFolder '/wBinary.mat'],'w');
    end
    
    if outerIter == ctrl.maxOuterIter
        save([outFolder '/wStruct_alpha_' num2str(ctrl.alpha) '.mat'],'w');
        disp(['W vector from Structured Output has been generated with alpha = ' num2str(ctrl.alpha) ]);
        done = 1;
    else
        save([outFolder '/wHistory.mat'],'w');
    end
    
end


%---------------------------------------------------------------------Clean
disp('Cleaning...');
for d = 1:numel(files)
    if ~exist([dataFolder '/feats_' files{d} '.mat'],'file') == 0
        delete([dataFolder '/feats_' files{d} '.mat']);
    end
end

if ~exist([outFolder '/wBinary.mat'],'file') == 0
    delete([outFolder '/wBinary.mat']);
end

if ~exist([outFolder '/wHistory.mat'],'file') == 0
    delete([outFolder '/wHistory.mat']);
end

%--------------------------------------------------------------------Finish
disp('Done! Structured Output W vector loaded');
disp(' ');
end



%==========================Struct-SVM CALLBACKS===========================%


%-------------------------------------------------------------Binary SVM CB

function delta = lossCB(param, y, ybar)
delta = double(y ~= ybar) ;
if param.verbose
    fprintf('delta = loss(%3d, %3d) = %f\n', y, ybar, delta) ;
end
end

function psi = featureCB(param, x, y)
psi = sparse(y*x/2) ;
if param.verbose
    fprintf('w = psi([%8.3f,%8.3f], %3d) = [%8.3f, %8.3f]\n', ...
        x, y, full(psi(1)), full(psi(2))) ;
end
end

function yhat = constraintCB(param, model, x, y)
% slack resaling: argmax_y delta(yi, y) (1 + <psi(x,y), w> - <psi(x,yi), w>)
% margin rescaling: argmax_y delta(yi, y) + <psi(x,y), w>
if dot(y*x, model.w) > 1, yhat = y ; else yhat = - y ; end
if param.verbose
    fprintf('yhat = violslack([%8.3f,%8.3f], [%8.3f,%8.3f], %3d) = %3d\n', ...
        model.w, x, y, yhat) ;
end
end

%------------------------------------------------------Structured Output CB

function psi = featureSOCB(param, x, y)
psi = y'*x(:,1:param.dimension);
psi = sparse(psi');
end

function delta = lossSOCB(param, y, ybar)
%L(y^j) = sum_i ( y_i * (|n_i - 1| - n_i) ) + D^j.
%D^j = user-palced dots in image j.

%way to get the sample number
for cl = 1:numel(param.labels)
    if numel(y) == numel(param.labels{cl})
        if y == param.labels{cl}
            sampleNum = cl;
            break;
        end
    end
end
%and get the GT for the sample
gtInside = param.gtInsideR{sampleNum};

N = param.placedDots{sampleNum}; %Number of user-placed dots
Nbar = ybar'*( abs(double(gtInside)-1) - double(gtInside) - param.alpha*(gtInside == 0) );
delta = round(N + Nbar);
end

function yhat = constraintSOCB(param, model, x, y)
% slack resaling: argmax_y delta(yi, y) (1 + <psi(x,y), w> - <psi(x,yi), w>)
% margin rescaling: argmax_y delta(yi, y) + <psi(x,y), w>

sampleNum = x(1,param.dimension + 1);
gtInside = param.gtInsideR{sampleNum};
nFeatures = param.dimension;

unary = x(:,1:nFeatures)*model.w + abs(double(gtInside)-1) - double(gtInside) - param.alpha*(gtInside == 0);

[~,yhat] = PylonInference(param.images{sampleNum}, unary, ...
    param.sizes{sampleNum}, param.rVector{sampleNum},0,param.trees{sampleNum});

end
