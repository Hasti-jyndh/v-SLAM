clc ; clear ; close all;
%% Part 0:
%% loading camera parmaeters
%load('C:\Users\zekim\Downloads\undergrad project\project\StereoParams.mat');
load('C:\Users\zekim\Downloads\undergrad project\project\calib2StereoParams.mat');
global stereoParams intrinsicsL baseline Distance
stereoParams = stereoParameters(stereoParams.CameraParameters1,stereoParams.CameraParameters2,stereoParams.RotationOfCamera2,stereoParams.TranslationOfCamera2);
intrinsicsL = stereoParams.CameraParameters1.Intrinsics  ;  %check the number
intrinsicsR = stereoParams.CameraParameters2.Intrinsics;  %redo the calibration
baseline = 48;

%% reading cameras from live stream
cam_right= ipcam('http://172.27.18.217:81/stream');
cam_left= ipcam('http://172.27.18.193:81/stream');
% preview(cam1)

%introducing directory to store image1s
Folder2 = 'C:\Users\zekim\Downloads\undergrad project\project\final_test2\Test33\left';  %change !...
Folder1 = 'C:\Users\zekim\Downloads\undergrad project\project\final_test2\Test33\right'; %change !...
address = "C:\Users\zekim\Downloads\undergrad project\project\final_test2\Test33";  %change !....

%% PART 1:
%% START
% upload t_intial and 'l' on thingSpeak
dt = 12; % t_initial [s]    could change!...
LT = 'l';
Voltage = 255;
counter = 11;
WiFiRead(counter,dt,Voltage)
% TakeIMGs for t_initial
iframe = 0;  % image saving frame
pause_time = 1;%[s] frame per sec  for "l"
t_img = 1;
pause(0.9);
while t_img < dt+1
    iframe = iframe + 1;
    frames_right = snapshot(cam_right);
    frames_left = snapshot(cam_left);
    imwrite(frames_left, fullfile(Folder2, sprintf('left%03d.jpg', iframe)));
    imwrite(frames_right, fullfile(Folder1, sprintf('right%03d.jpg', iframe)));   
    pause(pause_time);
    t_img = t_img+1;
end

%% PART 2:

%% SLAM initial frame
startframe = 1
endframe = iframe
%% load first image pair & create vset
vSet = imageviewset();
Distance = 86;%80;%86;  %changes according to motor changes !....
viewIdL  = 1;
frameId = 1;
vSet = Img2vSet(vSet,viewIdL,baseline,Distance,startframe,address);

%% create mytracks
MyTracks = IntroducingMyTracks2(vSet.Views.Points{viewIdL,1},vSet.Views.Points{viewIdL+1,1},vSet.Connections.Matches{frameId,1},viewIdL,viewIdL+1);
f_counter = ones(1,length(MyTracks));  % feature counter to find outliar
lastLtrack = 1:length(MyTracks); % indx of the features of the current frame in mytracks

%% create wpset and add 3dworldpoints to it
matchedPoints1 = vSet.Views.Points{viewIdL,1}.Location(vSet.Connections.Matches{frameId,1}(:,1),:);
matchedPoints2 = vSet.Views.Points{viewIdL+1,1}.Location(vSet.Connections.Matches{frameId,1}(:,2),:);
worldPoints = (vSet.Views.AbsolutePose(viewIdL,1).R*triangulate(matchedPoints1,matchedPoints2,stereoParams)'+vSet.Views.AbsolutePose(viewIdL,1).Translation')';

%% SLAM initial function
NFC = zeros(1,1000);
NFC(1) = length(MyTracks);
% key fram list
KFList = [1];
%refindPoseM = rigidtform3d.empty(endframe-startframe+endframe2-startframe2+endframe3-startframe3+3,0);%+endframe3-startframe3,0);
CTList = [];
CindLinst = [];
%%
keyCond = keyconditon_f(startframe,address); %% should change with density of obstacles in view
    frameId
    viewIdL_b = viewIdL
[viewIdL,frameId,vSet,MyTracks,cameraPosesRefined,worldPoints,f_counter,KFList,NFC,lastLtrack,CTList,CindLinst] = SLAM_Loop3(viewIdL,frameId,vSet,f_counter,startframe,endframe,address,LT,MyTracks,KFList,NFC,lastLtrack,worldPoints,CTList,CindLinst,keyCond);
    viewIdL_a = viewIdL
% outliar3 remover
[worldPoints_o3] = outlier3_remover(f_counter,MyTracks,worldPoints,0);%,NFC(KFList(end-1))+NFC(KFList(end)));
l_w_o3 = length(worldPoints_o3)
l_w = length(worldPoints)
ploting_pc(worldPoints_o3,cameraPosesRefined)
%% Clustering & Path planning
[centerG,LT,dt,Voltage] = Clustering_PathPlanning2(worldPoints_o3,cameraPosesRefined,viewIdL,vSet)
%%

%% PART 3 :
%% START Loop (while ?..)
while LT ~= "non"
    % upload "l/t" and tw on the cloud channel
    counter = counter+1;
    WiFiRead(counter,dt,Voltage)
    % pause(error_time)
    t_img = 1;
    %tic
    if LT == 'l'
        pause_time = 1;
    end
    if LT == 't'
        pause_time = 0.5;
        dt = dt*2;
    end
    pause(1.2)

    while t_img < dt+1
        iframe = iframe + 1;
        frames_right = snapshot(cam_right);
        frames_left = snapshot(cam_left);
        pause(pause_time);
        imwrite(frames_left, fullfile(Folder2, sprintf('left%03d.jpg', iframe)));
        imwrite(frames_right, fullfile(Folder1, sprintf('right%03d.jpg', iframe)));
        t_img = t_img+1;
    end
    t_img
    iframe
    %%
    startframe = endframe
    endframe = iframe
    %% SLAM
    keyCond = keyconditon_f(startframe,address); %% should change with density of obstacles in view
    frameId
    viewIdL_b = viewIdL
    [viewIdL,frameId,vSet,MyTracks,cameraPosesRefined,worldPoints,f_counter,KFList,NFC,lastLtrack,CTList,CindLinst] = SLAM_Loop3(viewIdL,frameId,vSet,f_counter,startframe,endframe,address,LT,MyTracks,KFList,NFC,lastLtrack,worldPoints,CTList,CindLinst,keyCond);
    viewIdL_a = viewIdL
    % outliar3 remover
    [worldPoints_o3] = outlier3_remover(f_counter,MyTracks,worldPoints,0);%NFC(KFList(end-1))+NFC(KFList(end)));
    %% Clustering & Path Planning
    [centerG,LT,dt,Voltage] = Clustering_PathPlanning2(worldPoints_o3,cameraPosesRefined,viewIdL,vSet)
    %% ploting point cloud
    ploting_pc(worldPoints_o3,cameraPosesRefined)
end
%% end loop
worldPoints1o = Clustering3_f(worldPoints_o3,cameraPosesRefined);
ploting_pc(worldPoints1o,cameraPosesRefined)


