function [IndexPair] = ImgMatchedfig(viewIDL,frame,address)
% this function adds frame # left and right images to vset
% viewID is the left images index in vset
%% left image1
imageAdd = fullfile(address,'left',sprintf('left%03d.jpg', frame));
imagesDir = imageDatastore(imageAdd);
I1 = readimage(imagesDir,1);
GIm = im2gray(I1);

pointsPrev1 = detectKAZEFeatures(GIm);
[FeaturePrev1,ValidPoints1] = extractFeatures(GIm,pointsPrev1);
%%  left image2

imageAdd = fullfile(address,'left',sprintf('left%03d.jpg', frame-1));
imagesDir = imageDatastore(imageAdd);
I2 = readimage(imagesDir,1);
GIm = im2gray(I2);

pointsPrev2 = detectKAZEFeatures(GIm);
[FeaturePrev2,ValidPoints2] = extractFeatures(GIm,pointsPrev2);

%% add connection
IndexPair = matchFeatures(FeaturePrev1,FeaturePrev2,Method="Approximate");

%%
figure
showMatchedFeatures(I1,I2,ValidPoints1(IndexPair(:,1)),ValidPoints2(IndexPair(:,2)),"montag",Parent=axes);
title("L-L fame %d",frame);
end