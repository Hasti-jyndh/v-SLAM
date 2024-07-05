function [vSet] = Img2vSet(vSet,viewIDL,baseline,Distance,frame,address)
% this function adds frame # left and right images to vset
% viewID is the left images index in vset
%load('C:\Users\zekim\Downloads\undergrad project\project\pose_initial_turn.mat')
%load('pose_initial_l2.mat')
%% left image
imageAdd = fullfile(address,'left',sprintf('left%03d.jpg', frame));
imagesDir = imageDatastore(imageAdd);
I1 = readimage(imagesDir,1);
GIm = im2gray(I1);

%pointsPrev1 = detectSURFFeatures(GIm);
pointsPrev1 = detectKAZEFeatures(GIm);
%pointsPrev1 = detectORBFeatures(GIm);


[FeaturePrev,ValidPoints1] = extractFeatures(GIm,pointsPrev1);
theta_i = -pi/4;
%Rot_E = [cos(theta_i) 0 sin(theta_i); 0 1 0; -sin(theta_i) 0 cos(theta_i)];

%vSet = addView(vSet,viewIDL,'Features',FeaturePrev,'Points',ValidPoints1,'absPose',x);%, rigid3d(eye(3,3),[0 0 (viewIDL-1)/2*Distance]));
vSet = addView(vSet,viewIDL,'Features',FeaturePrev,'Points',ValidPoints1,'absPose', rigid3d(eye(3,3),[0 0 0]));
%vSet = addView(vSet,viewIDL,'Features',FeaturePrev,'Points',ValidPoints1,'absPose', rigid3d(Rot_E,[-150 0 600]));
%%  right image

imageAdd = fullfile(address,'right',sprintf('right%03d.jpg', frame));
imagesDir = imageDatastore(imageAdd);
I2 = readimage(imagesDir,1);
GIm = im2gray(I2);
%pointsPrev2 = detectSURFFeatures(GIm);
pointsPrev2 = detectKAZEFeatures(GIm);
%pointsPrev2 = detectORBFeatures(GIm);
[FeaturePrev,ValidPoints2] = extractFeatures(GIm,pointsPrev2);
%x.Translation(1) =x.Translation(1)+baseline;
%vSet = addView(vSet, viewIDL+1,'Features',FeaturePrev,'Points',ValidPoints2,'absPose',x);%rigid3d(eye(3,3),[baseline 0 (viewIDL-1)/2*Distance]));
vSet = addView(vSet, viewIDL+1,'Features',FeaturePrev,'Points',ValidPoints2,'absPose',rigid3d(eye(3,3),[baseline 0 0]));
%vSet = addView(vSet, viewIDL+1,'Features',FeaturePrev,'Points',ValidPoints2,'absPose',rigid3d(Rot_E,[-150+baseline*cos(pi/4) 0 600-baseline*cos(pi/4)]));

%% add connection
IndexPair = matchFeatures(vSet.Views.Features{viewIDL,1},vSet.Views.Features{viewIDL+1,1},Method="Approximate");
vSet = addConnection(vSet,viewIDL,viewIDL+1,'Matches',IndexPair);
%%
figure
showMatchedFeatures(I1,I2,ValidPoints1(IndexPair(:,1)),ValidPoints2(IndexPair(:,2)),"montag",Parent=axes);
end