function [vSet] = Img2vSet3(vSet,viewIDL,baseline,Distance,frame,address,LT)
% this function adds frame # left and right images to vset
% viewID is the left images index in vset
%% left image
imageAdd = fullfile(address,'left',sprintf('left%03d.jpg', frame));
imagesDir = imageDatastore(imageAdd);
I1 = readimage(imagesDir,1);
GIm = im2gray(I1);

pointsPrev1 = detectKAZEFeatures(GIm);
%pointsPrev1 = detectSURFFeatures(GIm);
%pointsPrev1 = detectORBFeatures(GIm);
[FeaturePrev,ValidPoints1] = extractFeatures(GIm,pointsPrev1);

% asbsPoseN is absolutePose of the new frame if the motion is linear ...
% absPoseN = vSet.Views.AbsolutePose(viewIDL-2,1) ; 
% absPoseN.Translation(3) = absPoseN.Translation(3) + Distance; % could be modified with Rotation

%absPoseN = RobotMotion(vSet,viewIDL,Distance,'l'); %'l' if the motion is lenear 't' if robot turns
%absPoseN = RobotMotion(vSet,viewIDL,Distance,LT);
[NewPoseL, NewPoseR] = RobotMotion2(vSet,viewIDL,Distance,LT);
vSet = addView(vSet,viewIDL,'Features',FeaturePrev,'Points',ValidPoints1,'absPose',NewPoseL);%absPoseN);

%%  right image

imageAdd = fullfile(address,'right',sprintf('right%03d.jpg', frame));
imagesDir = imageDatastore(imageAdd);
I2 = readimage(imagesDir,1);
GIm = im2gray(I2);

pointsPrev2 = detectKAZEFeatures(GIm);
%pointsPrev2 = detectSURFFeatures(GIm);
%pointsPrev2 = detectORBFeatures(GIm);
[FeaturePrev,ValidPoints2] = extractFeatures(GIm,pointsPrev2);

%absPoseN.Translation(1) = absPoseN.Translation(1) + baseline; % could be modified with Rotation

vSet = addView(vSet, viewIDL+1,'Features',FeaturePrev,'Points',ValidPoints2,'absPose',NewPoseR);%absPoseN);

%% add connection
IndexPair = matchFeatures(vSet.Views.Features{viewIDL,1},vSet.Views.Features{viewIDL+1,1},Method="Approximate");
vSet = addConnection(vSet,viewIDL,viewIDL+1,'Matches',IndexPair);
%%
%showMatchedFeatures(I1,I2,ValidPoints1(IndexPair(:,1)),ValidPoints2(IndexPair(:,2)),"montag",Parent=axes);
end