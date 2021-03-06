% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
 mu = [150.0670 144.5593 60.2617];
 sig = diag([173.26 127.90 320.24]);
 thre = 0.000001;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
[x ,y ,d]=size(I);
mask=false(x,y);
for i=1:x,
    for j=1:y,
        Y=mvnpdf(double([I(i,j,1) I(i,j,2) I(i,j,3)]),mu,sig);
        if Y>thre,
            mask(i,j)=1;
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
segI = false(size(mask));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(mask);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
segI(CC.PixelIdxList{idx}) = true; 


S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
