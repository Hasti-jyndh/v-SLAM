%% detectKAZEFeatures (1)
clear all
clc
%close all
%Loading the file and storing the images
imageFolderLeft = fullfile('C:\Users\zekim\Downloads\undergrad project\project\newtests\Test_26\left');
imageFolderRight = fullfile('C:\Users\zekim\Downloads\undergrad project\project\newtests\Test_26\right');

imdsLeft = imageDatastore(imageFolderLeft);
imdsRight = imageDatastore(imageFolderRight);

% Starting to load the frames (Our first key frame is the left image)

currFrameIdx=100;
currILeft = readimage(imdsLeft,currFrameIdx);
currIRight = readimage(imdsRight,currFrameIdx);
% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectKAZEFeatures(im2gray(currILeft));
pointsRight = detectKAZEFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectKAZEFeatures')
%% detectBRISKFeatures	Detect BRISK features (2)

% Starting to load the frames (Our first key frame is the left image)

% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectBRISKFeatures(im2gray(currILeft));
pointsRight = detectBRISKFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');

title('detectBRISKFeatures')
%% detectFASTFeatures	Detect corners using FAST algorithm (3)

% Starting to load the frames (Our first key frame is the left image)

% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectFASTFeatures(im2gray(currILeft));
pointsRight = detectFASTFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectFASTFeatures')
%% detectHarrisFeatures	Detect corners using Harris–Stephens algorithm (4)

% Starting to load the frames (Our first key frame is the left image)
 
% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectHarrisFeatures(im2gray(currILeft));
pointsRight = detectHarrisFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectHarrisFeatures')
%% detectMinEigenFeatures	Detect corners using minimum eigenvalue algorithm (5)

% Starting to load the frames (Our first key frame is the left image)

% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectMinEigenFeatures(im2gray(currILeft));
pointsRight = detectMinEigenFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectMinEigenFeatures')
%% detectMSERFeatures	Detect MSER features (6)

% Starting to load the frames (Our first key frame is the left image)
 
% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectMSERFeatures(im2gray(currILeft));
pointsRight = detectMSERFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectMSERFeatures')
%% detectORBFeatures	Detect ORB keypoints (7)

% Starting to load the frames (Our first key frame is the left image)
 
% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectORBFeatures(im2gray(currILeft));
pointsRight = detectORBFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectORBFeatures')
%% detectSIFTFeatures	Detect scale invariant feature transform (SIFT) features (8)

% Starting to load the frames (Our first key frame is the left image)

% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectSIFTFeatures(im2gray(currILeft));
pointsRight = detectSIFTFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectSIFTFeatures')
%% detectSURFFeatures (9)

% Starting to load the frames (Our first key frame is the left image)

% imshowpair(currILeft, currIRight, 'montage');
pointsLeft = detectSURFFeatures(im2gray(currILeft));
pointsRight = detectSURFFeatures(im2gray(currIRight));

[features1,valid_points1] = extractFeatures(im2gray(currILeft),pointsLeft);
[features2,valid_points2] = extractFeatures(im2gray(currIRight),pointsRight);

[indexPairs] = matchFeatures(features1,features2,Method="Approximate");
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

figure; 
showMatchedFeatures(currILeft,currIRight,matchedPoints1,matchedPoints2 , 'montage');
title('detectSURFFeatures')