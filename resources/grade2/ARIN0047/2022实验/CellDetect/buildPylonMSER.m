function [MSERtree] = buildPylonMSER(img, r, areas)
%Builds the different trees that are formed when running MSER on the cell
%images.
%OUTPUT
%   MSERtree = strucutre with the field:
%       binForest: array of structures containing all binary trees. Each structure is a tree
%           of numberOfNon-leafNodesx3 = 1=parent 2=child 3=treeLevel
%       linkageStructCell: the same trees, but in the structure of 'linkage'
%       references: array of structures, each structure references a tree in the
%           forest, and contains the index r for each mser in the tree.        
%       nLeafs: indicates the number of leafs in each tree of forest
%INPUT
%   img = input image
%   r = the output structure from vl_mser
%   areas = the area of each region in 'r'

forest = cell(round(length(r)),1);
references = cell(round(length(r)),1);
pixelIndx = cell(round(length(r)),20);
positions = 2*ones(round(length(r)), 1);

%Lets arrange the regions in descending area
[areas, index] = sort(areas,'descend');
mask = uint16(zeros(size(img,1), size(img,2))); %asd = mask;
newSector = 1;
%Now lets check every region and build the trees
for k=1:length(r)
    
%    auxMask = uint8(zeros(size(img,1), size(img,2)));
    sel = vl_erfill(img,r(index(k))) ; %get the pixels of the region
%     auxMask(sel) = 1;
%     auxMask = bwmorph(auxMask, 'close');
%     sel = find(auxMask == 1);
    
    if isempty(sel)
        continue;
    end
    
    [sector,uniqCount] = count_unique(mask(sel)); %check which sector it overlaps
    %add the index r to the sector it belongs to, or create a new sector if there is nothing there yet..
    if sector == 0 %This is a new sector!
        %references{newSector} = [r(index(k)) prediction(index(k))];
        references{newSector} = r(index(k));
        pixelIndx{newSector,1} = sel;
        mask(sel) = newSector; %asd(sel) = round(255*rand(1));
        newSector = newSector + 1;
    else %The region belongs to a pre-defined sector
        
        if length(sector) > 1            
            sector = sector(find(uniqCount == max(uniqCount)));
            if length(sector) > 1
                disp(['An MSER overlaps equally with ' num2str(length(sector)) ' regions']);
                sector = sector(end);
            end
        end
        
        %references{sector} = [references{sector} ; r(index(k)) prediction(index(k))];
        references{sector} = [references{sector} ; r(index(k))];
        
        %check which region within the sector is immediately above
        if positions(sector)>2
            for i = positions(sector)-1:-1:1 %check the regions of the sector starting with the last one added
                if ismember(sel,pixelIndx{sector,i}) ~= 0
                    if i == 1
                        level = 1;
                    else
                        tree = forest{sector};
                        findParent = find(tree(:,2) == i);
                        level = tree(findParent,3)+1;
                    end
                    forest{sector} = [forest{sector} ; i positions(sector) level];
                    break;
                end
            end
        else
            forest{sector} = [1 2 1];
        end
        pixelIndx{sector,positions(sector)} = sel;
        positions(sector) = positions(sector) + 1;
        mask(sel) = sector; %asd(sel) = round(255*rand(1));
    end
    
end
forest(newSector:end) = [];
references(newSector:end) = [];
nLeafs = zeros(newSector-1,1,'uint16');

%-Count the number of leafs below each region
nLeafsBelowR = ones(length(r), 1);
for k = 1:numel(forest)
    if isempty(forest{k})
        continue;
    end
    
    tree = forest{k};
    ref = references{k};
    tempNLeafs = ones(size(ref));
    nLevels = max(tree(:,3));
    
    for lev = nLevels:-1:1
        levelMembers = find(tree(:,3) == lev);
        %membNumbers = tree(levelMembers,2);
        parentsOfMemb = unique(tree(levelMembers,1));
        for i = 1:length(parentsOfMemb)
            idxParent = find(tree(:,1) == parentsOfMemb(i));
            
            tempNLeafs(parentsOfMemb(i)) =...
                sum(tempNLeafs(tree(idxParent,2)));
        end
    end
    
    for i = 1:length(ref)
        posInR = r == ref(i);
        nLeafsBelowR(posInR) = tempNLeafs(i);
    end
    
end

%-Transformation to the Linkage structure and count number of leafs
%First, lets add dummies to make it binary
binForest = cell(numel(forest),1);
mapArray = cell(numel(forest),1);
mappedReferences = cell(numel(forest),1);
%largeVal = -50;
for k = 1:numel(forest)
    if isempty(forest{k})
        nLeafs(k) = 1;
        mappedReferences{k} = references{k};
        mapArray{k} = 1;
        continue;
    end
       
    tree = forest{k};
    nNodes = max(tree(:,2));
    dummy = nNodes + 1; %The numeration for dummy nodes starts after max
    nodesWithChildren = max(tree(:,1));
    isbinary = 0;
    
    while ~isbinary %Just add dummies: Tree has to be binary
        isbinary = 1;
        for i = 1:nodesWithChildren
            nOfChildren = numel(find(tree(:,1) == i));
            if nOfChildren > 0 && rem(nOfChildren,2) ~= 0 %Make even number of children
                nodeRow = find(tree(:,1) == i);
                level = tree(nodeRow(1),3);
                tree = [tree; i dummy level];
%                 references{k} = [references{k} ; 0 largeVal];
                references{k} = [references{k} ; 0];
                dummy = dummy + 1;
                nOfChildren = nOfChildren + 1;
            end
            if nOfChildren >= 4 %We'd have to add even more dummies for binary tree
                nPairs = nOfChildren/2;
                nodeRow = find(tree(:,1) == i);
                level = tree(nodeRow(1),3);
                for j = 1:nPairs
                    tree = [tree; i dummy level]; %Add the new dummies at the bottom
                    tree(nodeRow(2*j-1:2*j),1) = dummy; %Declare the new dummy as parent of one pair
                    tree(nodeRow(2*j-1:2*j),3) = level + 1;
                    %references{k} = [references{k} ; 0 largeVal];
                    references{k} = [references{k} ; 0];
                    dummy = dummy + 1;
                end
                %Now we need to increase the level of all dependencies
                
                baseLevel = level+1;
                dependRows = find(ismember(tree(:,1),tree(nodeRow,2)) == 1); %the children of the nodes
                while ~isempty(dependRows)
                    tree(dependRows,3) = baseLevel + 1;
                    baseLevel = baseLevel + 1;
                    dependRows = find(ismember(tree(:,1),tree(dependRows,2)) == 1);
                end
                
                if nOfChildren > 4
                    %disp(['Oh no!, it happened!! in tree ' num2str(k)]);
                    isbinary = 0;
                end
                
            end
            
        end
    end
    
    %-All dummies have been added so is possible the get binary trees.
    %Now we can pick-up the trees to form clusters as linkage function
    leafs = ismember(tree(:,2),tree(:,1));
    leafs = find(leafs == 0);
    nLeafs(k) = numel(leafs);
    nNodes = max(tree(:,2));
    nLevels = max(tree(:,3));
    %Mapping between Tree and linkageStruct. nodesMapping(i) contains the
    %index in 'reference' of node recently named 'i' for linkageStruct
    nodesMapping = zeros(nNodes-1, 1);
    nodesMapping(1:nLeafs(k)) = tree(leafs,2);
    
    linkageStruct = zeros(nLeafs(k)-1,3);
    clusterNum = 1; %Used to index linkageStruct
    mapPos = nLeafs(k) + 1;
    for i = nLevels:-1:1
        nodesInLevel = find(tree(:,3) == i); %Acutally, the rows of Tree in that level
        parents = unique(tree(nodesInLevel,1));
        for j=1:numel(parents)
            parentsRows = find(tree(:,1) == parents(j));
            pairOfChildren = tree(parentsRows,2);
            pairOfChildrenMap = [find(nodesMapping == pairOfChildren(1));...
                find(nodesMapping == pairOfChildren(2))];
            linkageStruct(clusterNum,:) = [pairOfChildrenMap' nLevels-i+1];
            nodesMapping(mapPos) = parents(j);
            clusterNum = clusterNum + 1;
            mapPos = mapPos + 1;
        end
    end
    linkageStruct(clusterNum:end,:) = [];
    binForest{k} = linkageStruct;
    mapArray{k} = nodesMapping;
    %Finally, make the mapped references
    tempReferences = references{k};
    %newReferences = zeros(size(tempReferences,1),2);
    newReferences = zeros(size(tempReferences,1),1);
    for i = 1:size(tempReferences,1)
        newReferences(i,:) = tempReferences(nodesMapping(i),:);
    end
    mappedReferences{k} = newReferences;
end

MSERtree.forest = binForest;
MSERtree.nodesMapping = mapArray;  
MSERtree.references = mappedReferences;
MSERtree.nLeafs = nLeafs;
MSERtree.nLeafsBelowR = nLeafsBelowR;

