function [centerG,LT,dt,Voltage] = Clustering_PathPlanning2(worldPoints,cameraPosesRefined,viewIdL,vSet)
%% Crop the data to contain only points within the specified region
wp=worldPoints;
% Boundaries should be set according to test space !....
xBoundP = 1500; % in mm
xBoundN = -5000;%-5210;%-1710;
yBoundP = 230;%250; % in mm
yBoundN = -500;%-900%-2000;%-400;%-2000;
zBoundP = 7500;%6500;%5420; %in mm
zBoundN = 0; % in mm

indices = wp(:,1) <= xBoundP & wp(:,1) >= xBoundN ...
    & wp(:,2) <= yBoundP & wp(:,2) >= yBoundN ...
    & wp(:,3) <= zBoundP & wp(:,3) >= zBoundN ;
wp = wp(indices,:);
size_wp = size(wp)
%scatter(wp(:,1),wp(:,2),'.');
%%
% figure
% scatter3(wp(:,1),wp(:,3),-wp(:,2),'.');
% xlabel('X')
% ylabel('Z')
% zlabel('-Y')
%% Select a value for minpts

minpts =5; % Minimum number of neighbors for a core point

% % % kD = pdist2(wp,wp,'euc','Smallest',minpts);
% % % figure
% % % plot(sort(kD(end,:)));
% % % title('k-distance graph')
% % % xlabel('Points sorted with 5th nearest distances')
% % % ylabel('5th nearest distances')
% % % grid

%% set epsilon
epsilon = 179;%175;%200;%180%140;%130;

wp2 = [wp(:,1) wp(:,3)];
labels = dbscan(wp2,epsilon,minpts);

numGroups = length(unique(labels));
figure
gscatter(wp(:,1),wp(:,3),labels,hsv(numGroups));

%% attempt 2 calculating mean and variance
%%
indxGroup = zeros(length(labels),max(labels));
Gc = zeros(1,max(labels));
for i = 1:length(labels)
    for j = 1:max(labels)
        if j == labels(i)
            Gc(j) = Gc(j)+1;
            indxGroup(Gc(j),j) = i;
        end
    end
end

%%
varG = zeros(max(labels),3);
centerG = zeros(max(labels),3);
for j = 1:max(labels)
    varG(j,:) = var(wp(indxGroup(1:Gc(j),j),:));
    centerG(j,:) = mean(wp(indxGroup(1:Gc(j),j),:));
end
%rG = sqrt(varG) + [sqrt(410^2+240^2) 0 sqrt(410^2+240^2)]; %*1.5%*2;
rG = sqrt(varG) + [sqrt(410^2+240^2)/2 0 sqrt(410^2+240^2)/2];
%% small cluster remover
smallCList = [];
for i = 1:max(labels)
    if Gc(i)< 7 %8
        smallCList(end+1) = i;
    end
end
rG(smallCList,:) = [];
varG(smallCList,:) = [];
centerG(smallCList,:) = [];
%% ploting elliptical obstacles
t = linspace(0,2*pi) ;
for n=1: size(rG,1)
    x = rG(n,1)*cos(t)+centerG(n,1);
    z = rG(n,3)*sin(t)+centerG(n,3);
    %figure
    hold on
    plot(x,z)
end
axis equal

%% ploting cluster numbers at center of it
for n=1: size(rG,1)
    hold on
    text(centerG(n,1),centerG(n,3),num2str(n))
end
%% Rectangular Obstacles
Rect = zeros(5,2, size(rG,1));
for n=1: size(rG,1)
    Rect(:,:,n) = [-rG(n,1)+centerG(n,1)  rG(n,3)+centerG(n,3);rG(n,1)+centerG(n,1)  rG(n,3)+centerG(n,3);rG(n,1)+centerG(n,1)  -rG(n,3)+centerG(n,3);-rG(n,1)+centerG(n,1)  -rG(n,3)+centerG(n,3);-rG(n,1)+centerG(n,1)  rG(n,3)+centerG(n,3)];
end
for n=1: size(rG,1)
    hold on
    plot(Rect(:,1,n),Rect(:,2,n))
end
%% increasing obstacles to convexhull
% A = [180 -70];
% B = [-240 -70];
% C = [-240 410];
% D = [180 410];
% inc_obs = zeros(13,2,size(rG,1));
% for n=1: size(rG,1)
%     VerRec = [-rG(n,1)+centerG(n,1)  rG(n,3)+centerG(n,3);rG(n,1)+centerG(n,1)  rG(n,3)+centerG(n,3);rG(n,1)+centerG(n,1)  -rG(n,3)+centerG(n,3);-rG(n,1)+centerG(n,1)  -rG(n,3)+centerG(n,3);-rG(n,1)+centerG(n,1)  rG(n,3)+centerG(n,3)];
%     VerInc = [VerRec+A; VerRec+B; VerRec+C;VerRec+D ];
%     indx_hull = convhull(VerInc);
%     plot(VerInc(:,1),VerInc(:,2),'*')
%     hold on
%     inc_obs (:,:,n) = [VerInc(indx_hull,1) VerInc(indx_hull,2)];
%     plot(inc_obs (:,1,n),inc_obs (:,2,n));
% end
%% Path Planning & intersect check
LT = "non";
Voltage = -1;  % for function to return something
dt = 6; % suggestion ...
Distance = 3450/39;
% % theta = asin(-vSet.Views.AbsolutePose(end-1,1).R(3,1));
% % theta = asin(-cameraPosesRefined.AbsolutePose(viewIdL,1).R(3,1)); %[rad]

% % % % sin_theta = -vSet.Views.AbsolutePose(viewIdL,1).R(1,3);
% % % % phi = atan2(vSet.Views.AbsolutePose(viewIdL,1).R(2,3),vSet.Views.AbsolutePose(viewIdL-2,1).R(3,3));
% % % % cos_theta = vSet.Views.AbsolutePose(viewIdL,1).R(3,3)/abs(cos(phi));
% % % % theta = atan2(sin_theta,cos_theta);

% sin_theta = -vSet.Views.AbsolutePose(viewIdL,1).R(3,1);
% psi = atan2(vSet.Views.AbsolutePose(viewIdL,1).R(2,1),vSet.Views.AbsolutePose(viewIdL-2,1).R(1,1));
% cos_theta = vSet.Views.AbsolutePose(viewIdL,1).R(1,1)/abs(cos(psi));
% theta = atan2(sin_theta,cos_theta);
% theta_deg = theta*180/pi
% psi_deg = psi*180/pi
% % % eu = rotm2eul(vSet.Views.AbsolutePose(end-1,1).R);
% % % theta = eu(2);

[eu eu_alt] = rotm2eul(vSet.Views.AbsolutePose(end-1,1).R)
if abs(eu(1))<abs(eu_alt(1))
    theta = eu(2);
else
    theta = eu_alt(2);
end



Ry = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
%Ry = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
dt_check = 0;
viewIdL
while dt_check == 0;

    %Path_Poly0 = [-10 10 10 -10 -10;0 0 0 0 0;0 0 (dt*Distance+350) (dt*Distance+350) 0];
    Path_Poly0 = [-30 30 30 -30 -30;0 0 0 0 0;0 0 (dt*Distance+440) (dt*Distance+440) 0];
    Path_Poly = cameraPosesRefined.AbsolutePose(viewIdL,1).Translation' + Ry*Path_Poly0;
    %
    %     figure
    %     hold on
    %     plot(Path_Poly0(1,:),Path_Poly0(3,:),'*')
    %     hold on
    %     plot(Path_Poly0(1,:),Path_Poly0(3,:),'r')
    %     hold on
    %     plot(Path_Poly(1,:),Path_Poly(3,:),'c')
    %     axis equal
    %     hold on
    %     plot(cameraPosesRefined.AbsolutePose(viewIdL,1).Translation(1),cameraPosesRefined.AbsolutePose(viewIdL,1).Translation(3),"*")

    intersect_check = 0;
    PolyShape_Path = polyshape(Path_Poly(1,1:end-1),Path_Poly(3,1:end-1));
    for n=1: size(rG,1)
        PolySahpe_obs = polyshape(Rect(1:4,1,n),Rect(1:4,2,n));
        Polyout = intersect(PolyShape_Path,PolySahpe_obs);
        intersect_check = intersect_check+size(Polyout.Vertices,1);
    end
    if intersect_check==0;
        dt_check = 1;
    else
        dt = dt-1;%0.5;
    end
    if dt<1
        %dt
        %dt_notfound =
        break
    end
end
%dt

if dt>0
    LT = "l"
    Voltage = 255;
else
    %Path_Poly0 = [-10 10 10 -10 -10;0 0 0 0 0;0 0 (dt*Distance+350) (dt*Distance+350) 0];
    Path_Poly0 = [-400 30 30 -400 -400;0 0 0 0 0;0 0 (dt*Distance+350) (dt*Distance+350) 0];
    Path_Poly = cameraPosesRefined.AbsolutePose(viewIdL,1).Translation' + Ry*Path_Poly0;
    PolyShape_Path = polyshape(Path_Poly(1,1:end-1),Path_Poly(3,1:end-1));
    %intersect_check = 0;
    %checking which obstacle might be in the way of the arc
    obs_arc = [];
    for n=1: size(rG,1)
        PolySahpe_obs = polyshape(Rect(1:4,1,n),Rect(1:4,2,n));
        Polyout = intersect(PolyShape_Path,PolySahpe_obs);
        %intersect_check = intersect_check+size(Polyout.Vertices,1);
        if size(Polyout.Vertices,1)>0
            obs_arc(end+1) = n
        end
    end
    %% arc points
    check_arc_in = 0;
    R_arc = 300;
    C_arc = cameraPosesRefined.AbsolutePose(viewIdL,1).Translation' + Ry*[-250; 0 ;-150];
    tt = linspace(pi/6-theta,pi*2/3-theta); %linspace(pi/3,5/6*pi);
    x_arc = R_arc*cos(tt)+C_arc(1);
    z_arc = R_arc*sin(tt)+C_arc(3);
    obs_arc
    for m=1:length(obs_arc)
        in = inpolygon(x_arc,z_arc,Rect(:,1,obs_arc(m)),Rect(:,2,obs_arc(m)))
        check_arc_in = check_arc_in + sum(in)

        if check_arc_in >0
            %figure
            hold on
            plot(Rect(:,1,obs_arc(m)),Rect(:,2,obs_arc(m))) % polygon
            axis equal

            hold on
            plot(x_arc(in),z_arc(in),'r+') % points inside
            plot(x_arc(~in),z_arc(~in),'bo') % points outside
            hold on
            
            

        end
    end

    if check_arc_in == 0
        LT = "t"
        dt = 15; % value for 90 deg turn ?/.........
        Voltage = 10;
        hold on
        plot(x_arc,z_arc)
        hold on
        plot(C_arc(1),C_arc(3),'o')
    end
end

%%
%figure
hold on
% plot(Path_Poly0(1,:),Path_Poly0(3,:),'r')
% hold on
plot(Path_Poly(1,:),Path_Poly(3,:),'c')
axis equal
hold on
plot(cameraPosesRefined.AbsolutePose(viewIdL,1).Translation(1),cameraPosesRefined.AbsolutePose(viewIdL,1).Translation(3),"*")

%len = length(labels);
end
