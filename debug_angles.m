
vSetR = zeros(3,3,size(vSet.Views,1)/2);
for i=1:size(vSet.Views,1)/2
    vSetR(:,:,i)  =vSet.Views.AbsolutePose(2*i-1,1).R ;
end
%%
[eu_hs eu_hs_alt] = rotm2eul(vSetR(:,:,:));
%%
figure
plot([1:size(vSet.Views,1)/2],eu_hs(:,1)./pi.*180)
hold on
plot([1:size(vSet.Views,1)/2],eu_hs_alt(:,1)./pi.*180)
%%
figure
plot([1:size(vSet.Views,1)/2],eu_hs(:,2)./pi.*180)
hold on
plot([1:size(vSet.Views,1)/2],eu_hs_alt(:,2)./pi.*180)
%%
figure
plot([1:size(vSet.Views,1)/2],eu_hs(:,3)./pi.*180)
hold on
plot([1:size(vSet.Views,1)/2],eu_hs_alt(:,3)./pi.*180)

%%
% for i=1:size(vSet.Views,1)/2
% asintheta(i) = asind(vSetR(3,3,i));
% end
% %%
% figure
% plot([1:size(vSet.Views,1)/2],asintheta)

 