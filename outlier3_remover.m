function [worldPoints] = outlier3_remover(f_counter,MyTracks,worldPoints,last2KF_NFC)
%last2KF_NFC = NFC(KFList(end-1))+NFC(KFList(end))  
% to avoid removing new features that might be paired in feauter
outliarList = [];
for m=1:length(f_counter)-last2KF_NFC
    if f_counter(m)<3
        outliarList(end+1)=m;
    end
end
%%

MyTracks(outliarList) = [];
worldPoints(outliarList,:) = [];
f_counter(outliarList) = [];
end