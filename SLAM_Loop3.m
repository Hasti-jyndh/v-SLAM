function [viewIdL,frameId,vSet,MyTracks,cameraPosesRefined,worldPoints,f_counter,KFList,NFC,lastLtrack,CTList,CindLinst] = SLAM_Loop3(viewIdL,frameId,vSet,f_counter,startframe,endframe,address,LT,MyTracks,KFList,NFC,lastLtrack,worldPoints,CTList,CindLinst,keyCond)
global stereoParams intrinsicsL baseline Distance
frameId_initial = frameId;
%loop_frame_num = endframe+startframe+frameId_initial-1
while frameId<endframe%+startframe+frameId_initial-1

    % load another pair of images & add it to vset
    viewIdL = viewIdL+2;
    frameId = frameId+1;
    %figure
    vSet = Img2vSet3(vSet,viewIdL,baseline,Distance,startframe+frameId-frameId_initial,address,LT);  %
    %% ploting
    %figure
    %IndP = ImgMatchedfig(viewIdL,startframe+frameId-1,address)
    %IndexPair_LL = ImgMatchedfig(viewIdL,startframe+frameId-frameId_initial,address);
    % List of Tracks that are common btw curr-last-L (common Track List)
% % % %     if KFList(end)==frameId-1
        CTList = [];
        % common index list has records i (feature index location invset)
        CindLinst = [];
% % % %     end
    % List of New Tracks of cuurKeyFrame (New Track List)
    NTList = [];
    % New index list has records i (feature index location invset)
    NindLinst = [];

    %% updating mytracks
    IndexPair_LL = matchFeatures(vSet.Views.Features{viewIdL,1},vSet.Views.Features{viewIdL-2,1},Method="Approximate");
    l_mytracks0 = length(lastLtrack);
    % for each feature in currL-R
    checklist = ones(1,length(MyTracks));
    newTrack = zeros(1,length(vSet.Connections.Matches{frameId,1}));
    newTC = 1;
    for i = 1:length(vSet.Connections.Matches{frameId,1})
        if size(vSet.Connections.Matches{frameId,1},1) ~= 1  %% new

        % if found matching feature in currL-lastL
        frame_matched = 0;
        for j=1:size(IndexPair_LL,1) %length(IndexPair_LL)
            %while vSet.Connections.Matches{frameId,1}(i,1) <= IndexPair_LL(j,1)

            if vSet.Connections.Matches{frameId,1}(i,1) == IndexPair_LL(j,1) && ~frame_matched
                % search for match in mytracksdd
                for kk=1:l_mytracks0
                    k = lastLtrack(kk);

                    L_ind = f_counter(k)*2+1; % the current Left frame's indx in sub-mytracks

                    if IndexPair_LL(j,2) == MyTracks(1,k).FeatureIndices(L_ind-2) && frame_matched==0 && checklist(k)==1

                        % if the feature is seen in previous frame add the
                        % indces to my tracks
                        MyTracks(1,k).ViewIds(L_ind) = viewIdL;
                        MyTracks(1,k).ViewIds(L_ind+1) = viewIdL+1 ;
                        MyTracks(1,k).Points(L_ind,:) = ...
                            vSet.Views.Points{viewIdL,1}.Location(vSet.Connections.Matches{frameId,1}(i,1),:);
                        MyTracks(1,k).Points(L_ind+1,:) = ...
                            vSet.Views.Points{viewIdL+1,1}.Location(vSet.Connections.Matches{frameId,1}(i,2),:) ;
                        MyTracks(1,k).FeatureIndices(L_ind:L_ind+1) = vSet.Connections.Matches{frameId,1}(i,:) ;
                        frame_matched = 1;
                        f_counter(k) = f_counter(k)+1;
                        newTrack(newTC) = k;
                        newTC = newTC+1;
                        checklist(k)=0;
% % % %                         if KFList(end)==frameId-1
                            CTList(end+1) = k;
                            CindLinst(end+1) = i;
% % % %                         end
                    end

                end
            end
            % adding the new features in the end of mytracks
            %end
        end
        if frame_matched==0 %% && size(vSet.Connections.Matches{frameId,1},1) ~= 1
            %ifcheck = ifcheck+1;
            MyTracks(1,length(MyTracks)+1).ViewIds = [viewIdL viewIdL+1];
            MyTracks(1,length(MyTracks)).Points =  ...
                [vSet.Views.Points{viewIdL,1}.Location(vSet.Connections.Matches{frameId,1}(i,1),:);...
                vSet.Views.Points{viewIdL+1,1}.Location(vSet.Connections.Matches{frameId,1}(i,2),:)] ;
            MyTracks(1,length(MyTracks)).FeatureIndices = vSet.Connections.Matches{frameId,1}(i,:) ;
            % outliar counter:
            f_counter(length(f_counter)+1) = 1;
            newTrack(newTC) = length(MyTracks);
            newTC = newTC+1;
            %frameIdcheck = frameId
            NFC(frameId) = NFC(frameId)+1;

            NTList(end+1) = length(MyTracks);   %(k) check again??..
            NindLinst(end+1) = i;

        end  %% new
        end
    end
    lastLtrack = newTrack;
    %% key frame
    % check if the current frame is key frame
    %if (Ex. 5) frame has passed since lastkeyframe then: keyframe
    %if NFC > threshold(Ex. 80) then : keyframe
    %??if current frame is keyframe continue else go back laod another image?

%if length(CindLinst) ~=0
    if NFC(frameId)>keyCond || frameId-KFList(end)>5 
        KFList(end+1) = frameId;

        %% using MotionBA to refine camerapose
        %         %commonMyTrack = MyTracks(CTList);
        %         commonWP = worldPoints(CTList,:);
        %         imgPointsL = vSet.Views.Points{viewIdL,1}.Location(CindLinst,:) ;
        %         absPoseL = vSet.Views.AbsolutePose(viewIdL,1);
        %         absPoseR = vSet.Views.AbsolutePose(viewIdL+1,1);
        %         refindPoseM(viewIdL) = bundleAdjustmentMotion(commonWP,imgPointsL,absPoseL,intrinsicsL);
        % % %         RefindPoseL = bundleAdjustmentMotion(commonWP,imgPointsL,absPoseL,intrinsicsL);
        % % %         %vSet.Views.AbsolutePose(viewIdL,1)
        % % %         %vSet = updateView(vSet,viewIdL,refindPoseM(1,viewIdL)); %updates camerapose LEFT in vset to refinedCameraPose
        % % %         vSet = updateView(vSet,viewIdL,RefindPoseL); %updates camerapose LEFT in vset to refinedCameraPose
        % % %         RefindPoseR = RefindPoseL;
        % % %         RefindPoseR.Translation(1) = RefindPoseR.Translation(1)+baseline;
        % % %         vSet = updateView(vSet,viewIdL+1,RefindPoseR); %updates camerapose LEFT in vset to refinedCameraPose
        % % %


        %% attempt2
        %commonMyTrack = MyTracks(CTList);
        
        commonWP = worldPoints(CTList,:);
        %imgPointsL = vSet.Views.Points{viewIdL,1}.Location(CindLinst,:) ;
        imgPointsL = vSet.Views.Points{viewIdL,1}.Location(vSet.Connections.Matches{frameId,1}(CindLinst,1),:) ; %NEW
        absPoseL = vSet.Views.AbsolutePose(viewIdL,1);
        absPoseR = vSet.Views.AbsolutePose(viewIdL+1,1);
        %         refindPoseM(viewIdL) = bundleAdjustmentMotion(commonWP,imgPointsL,absPoseL,intrinsicsL);
        if length(CindLinst) ~=0
        RefindPoseL = bundleAdjustmentMotion(commonWP,imgPointsL,absPoseL,intrinsicsL);
        if norm(vSet.Views.AbsolutePose(viewIdL-2,1).Translation-RefindPoseL.Translation) <= Distance*3/2 && abs(vSet.Views.AbsolutePose(viewIdL-2,1).Translation(2)-RefindPoseL.Translation(2)) <= 10
            %thetaP = asind(-vSet.Views.AbsolutePose(viewIdL-2,1).R(3,1))
            %thetaR = asind(-RefindPoseL.R(3,1))
% %             delta_theta = abs(asind(-vSet.Views.AbsolutePose(viewIdL,1).R(3,1))-asind(-RefindPoseL.R(3,1)));
% % % % 
% % % %             sin_thetaP = -vSet.Views.AbsolutePose(viewIdL-2,1).R(1,3);
% % % %             phiP = atan2(vSet.Views.AbsolutePose(viewIdL-2,1).R(2,3),vSet.Views.AbsolutePose(viewIdL-2,1).R(3,3));
% % % %             cos_thetaP = vSet.Views.AbsolutePose(viewIdL-2,1).R(3,3)/abs(cos(phiP));
% % % %             thetaP = atan2(sin_thetaP,cos_thetaP);
% % % %             sin_thetaR = -vSet.Views.AbsolutePose(viewIdL-2,1).R(1,3);
% % % %             phiR = atan2(vSet.Views.AbsolutePose(viewIdL-2,1).R(2,3),vSet.Views.AbsolutePose(viewIdL-2,1).R(3,3));
% % % %             cos_thetaR = vSet.Views.AbsolutePose(viewIdL-2,1).R(3,3)/abs(cos(phiR));
% % % %             thetaR = atan2(sin_thetaR,cos_thetaR);
% % % %             delta_theta = abs(thetaP-thetaR);

[eu_P eu_P_alt]  = rotm2eul(vSet.Views.AbsolutePose(viewIdL-2,1).R);
if abs(eu_P(1))<abs(eu_P_alt(1))
    thetaP = eu_P(2);
else
    thetaP = eu_P_alt(2);
end


[eu_R eu_R_alt]  = rotm2eul(RefindPoseL.R);
if abs(eu_R(1))<abs(eu_R_alt(1))
    thetaR = eu_R(2);
else
    thetaR = eu_R_alt(2);
end
delta_theta = abs(thetaP-thetaR);

% % %             sin_thetaP = -vSet.Views.AbsolutePose(viewIdL-2,1).R(3,1);
% % %             psiP = atan2(vSet.Views.AbsolutePose(viewIdL-2,1).R(2,1),vSet.Views.AbsolutePose(viewIdL-2,1).R(1,1));
% % %             cos_thetaP = vSet.Views.AbsolutePose(viewIdL-2,1).R(1,1)/abs(cos(psiP));
% % %             thetaP = atan2(sin_thetaP,cos_thetaP);
%             eu_P  = rotm2eul(vSet.Views.AbsolutePose(viewIdL-2,1).R);
%             thetaP = eu_P(2);
% % % 
% % %             sin_thetaR = -vSet.Views.AbsolutePose(viewIdL-2,1).R(3,1);
% % %             psiR = atan2(vSet.Views.AbsolutePose(viewIdL-2,1).R(2,1),vSet.Views.AbsolutePose(viewIdL-2,1).R(1,1));
% % %             cos_thetaR = vSet.Views.AbsolutePose(viewIdL-2,1).R(1,1)/abs(cos(psiR));
% % %             thetaR = atan2(sin_thetaR,cos_thetaR);
% % %             delta_theta = abs(thetaP-thetaR);
% %             eu_R  = rotm2eul(RefindPoseL.R);
% %             thetaR = eu_R(2);
% %             delta_theta = abs(thetaP-thetaR);
            if delta_theta<3/180*pi
                
                %norm(vSet.Views.AbsolutePose(viewIdL,1).Translation-RefindPoseL.Translation)
                %viewIdL
                %testMc = testMc+1;
                %yd = abs(vSet.Views.AbsolutePose(viewIdL-2,1).Translation(2)-RefindPoseL.Translation(2))
% %                 eul_ref = eu_R./pi.*180

                vSet = updateView(vSet,viewIdL,RefindPoseL); %updates camerapose LEFT in vset to refinedCameraPose
                RefindPoseR = RefindPoseL;
%                 RefindPoseR.Translation(1) = RefindPoseR.Translation(1)+baseline;
% %                 theta_RefinedL = asind(-RefindPoseL.R(3,1));
                RefindPoseR.Translation = RefindPoseR.Translation+[baseline*cos(thetaR) 0 -baseline*sin(thetaR)];
                vSet = updateView(vSet,viewIdL+1,RefindPoseR); %updates camerapose LEFT in vset to refinedCameraPose
            end
        end
        end
        %% triangulate the new features found in the currkeyframe and add it to 3dwp
        matchedP1 = vSet.Views.Points{viewIdL,1}.Location(vSet.Connections.Matches{frameId,1}(NindLinst,1),:);
        matchedP2 = vSet.Views.Points{viewIdL+1,1}.Location(vSet.Connections.Matches{frameId,1}(NindLinst,2),:);
        %worldPoints(end+1:end+length(NTList),:) = triangulate(matchedP1,matchedP2,stereoParams);
        worldPoints(end+1:end+length(NTList),:) =(vSet.Views.AbsolutePose(viewIdL,1).R*triangulate(matchedP1,matchedP2,stereoParams)'+vSet.Views.AbsolutePose(viewIdL,1).Translation')';

        %% use BAstructure to refineXYZpoints from reprojection error ??...
        %         ViewId = uint32(1:frameId*2)';
        %         AbsolutePose = vSet.Views.AbsolutePose(:,1);
        %         %cameraPoses = table(ViewId,AbsolutePose);
        %         %xyzRefinedPoints = bundleAdjustmentStructure(worldPoints,MyTracks,cameraPoses,intrinsicsL); %intrinsicsL?????...
        %         %[wpSetRefined,vSetRefined,pointIndex] = bundleAdjustmentStructure(wpSet,vSet,viewID,intrinsics)
        %         %AbsolutePose = [refindPoseM';vSet.Views.AbsolutePose(end,1)];
        %         cameraPosesRefined = table(ViewId,AbsolutePose); %right views poses should be corrected later !...
        %         worldPoints = bundleAdjustmentStructure(worldPoints,MyTracks,cameraPosesRefined,intrinsicsL); %intrinsicsL?????...
        %         %?? why is xyzRefinedPoints = xyzRefinedPoints2
        %

        %% removing new features of currframe that is NOT a key frame
    else
        MyTracks(end-length(NTList)+1:end)= [];
        f_counter(end-length(NTList)+1:end) = [];
        lastLtrack = CTList;
    end
%end
end

%% use BAstructure to refineXYZpoints from reprojection error ??...
ViewId = uint32(1:frameId*2)';
AbsolutePose = vSet.Views.AbsolutePose(:,1);
%cameraPoses = table(ViewId,AbsolutePose);
%xyzRefinedPoints = bundleAdjustmentStructure(worldPoints,MyTracks,cameraPoses,intrinsicsL); %intrinsicsL?????...
%[wpSetRefined,vSetRefined,pointIndex] = bundleAdjustmentStructure(wpSet,vSet,viewID,intrinsics)
%AbsolutePose = [refindPoseM';vSet.Views.AbsolutePose(end,1)];
cameraPosesRefined = table(ViewId,AbsolutePose); %right views poses should be corrected later !...
% % worldPoints2 = bundleAdjustmentStructure(worldPoints,MyTracks,cameraPosesRefined,intrinsicsL); %intrinsicsL?????...
% % delta_wp = norm(worldPoints-worldPoints2)
% % thereshold = 1.4e3*length(worldPoints)
% % if delta_wp < thereshold
% %     worldPoints = worldPoints2;
% % end
worldPoints = bundleAdjustmentStructure(worldPoints,MyTracks,cameraPosesRefined,intrinsicsL); %intrinsicsL?????...

%?? why is xyzRefinedPoints = xyzRefinedPoints2

end