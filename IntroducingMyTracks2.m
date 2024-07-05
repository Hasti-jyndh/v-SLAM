
function [MyTracks] = IntroducingMyTracks2(valid_points_left_set1,valid_points_right_set1,indexPairs,viewid1,viewid2)

for i = 1:length(indexPairs)
    MyTracks(i) = [pointTrack([1 2],...
        [valid_points_left_set1.Location(indexPairs(i,1),:) ; valid_points_right_set1.Location(indexPairs(i,2),:)],...
        indexPairs(i,:))];
end

end


% for i = 1:length(indexPairs_12)
%     MyTracks(i) = [pointTrack([1 2],...
%         [valid_points1_1.Location(indexPairs_12(i,1),:) ; valid_points2_1.Location(indexPairs_12(i,2),:)],...
%         indexPairs_12(i,:))];
% end