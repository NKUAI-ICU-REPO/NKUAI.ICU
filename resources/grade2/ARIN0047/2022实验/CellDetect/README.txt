~~~~~~~~~~~~~~~~~~~~~~~~~~
     CellDetect v1.0
~~~~~~~~~~~~~~~~~~~~~~~~~~

This is a MATLAB implementation of the cell detector described in:

C. Arteta, V. Lempitsky, J. A. Noble, A. Zisserman
Learning to Detect Cells Using Non-overlapping Extremal Regions  
MICCAI 2012

This package provides code to learn a model for cell detection based on dot-annotations, 
and code to detect cells in novel images given a learnt model. 
It also contains a script to demo the functionality of the code on a data set of cell images that is also included.

Project page --> http://www.robots.ox.ac.uk/~vgg/research/cell_detection/

If you have doubts, contact me at carlos.arteta@eng.ox.ac.uk

DEPENDENCIES:
------------

This code requires the setup of:

1 - VL_feat (http://www.vlfeat.org/).
2 - SVM_struct_matlab (http://www.vlfeat.org/~vedaldi/code/svm-struct-matlab.html).
3 - Inference code for Pylon models (http://www.robots.ox.ac.uk/~vilem/ , which
depends on the QPBO from Vladimir Kolmogorov http://pub.ist.ac.at/~vnk/software.html). 
4 - MATLAB Statistics and Image Processing toolboxes. 
5 - (optional) MATLAB parallel computing toolbox 

This code has been tested using VL_feat 0.9.14, svm_struct_1.0 and pyloncode 1.0, 
under Ubuntu11.04-12.04 and Windows7, with MATLAB2011b-2012a.

DEMO:
----

To try the code quickly, just run 'demo.m'. 
The 'setup' section of 'demo.m' allows you to choose to train and/or test on the data set included.
This is a small dataset of phase contrast images for demonstration, provided with a pre-trained model. 
As it is configured in this release, 'demo.m' should run out of the box in testing mode.

HOW WOULD THIS WORK ON MY DATA?:
---------------------------

Try 'WouldThisWork.m' as a way to quickly test the applicability of the method in your data. 
Instructions can be found in the script.

USING THE CODE:
--------------

To use the cell detector in a new data set you need to:

1 - Install the dependencies.

2 - Configure 'loadDatasetInfo.m' to set-up the information of the data sets, such as paths of the training and testing data, 
image extensions, etc. This information will be related to a data set ID. Instruction can be found in the script.

3 - Configure 'setFeatures.m' if you wish to modify the default selection of features for learning, as well as
other control parameters such as the use of parallel computing*. 

4 - You can use 'demo.m' to run the training and testing functions on your data, with the appropriate data sets' IDs configured
in the 'setup' section of the script.

As described in the paper, the learning code requires cell images with dot-annotations.
For quick compatibility, the dot-annotations for the training images should be provided in '.mat' files, one 
per image and with the same name as its correspondent image. E.g. for image 'img005.png', the annotation file is 'img005.mat'. 
This '.mat' file should be a Nx2 matrix, with (:,1) being the X coordinates and (:,2) the Y coordinates of the annotation dots.

The output of the testing code is also '.mat' files containing dot-annotations for the novel images (in the same format described above),
as well as the binary masks of the extremal regions selected as cells. These masks are not being
post-processed for cell segmentation!. Final results are saved in the output folder set in 'loadDatasetInfo.m'.


*The parallel computing is done using 'parfor' loops. This requires MATLAB's parallel computing toolbox.

---------------------------------------------------------------------

Copyright (C) 2012 by Carlos Arteta

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
