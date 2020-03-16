%{
 * Copyright (C) 2020-2030, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [U,center,LE,RE,LEavg,REavg,LEupper,LElower,REupper,RElower,RingNumbers,NScans,PayLoadClean, PayLoadClean2D, flag_changed] = LeftRightEdges_v01(base_line, pnts, d, ExpNmbr)

% pnts is the pioint cloud structure that Bruce builds up


%U as in [U,S,V] to determined the normal to the pointcloud. 
%U(:,3) is the normal to the point cloud
% center is the mean of the pointcould
% center + U*[0;y;z] puts a point back in teh coordinates of the pointcloud
%
%
%LE = ([y;z] corrdinates) x Rings x scans
%RE = ([y;z] corrdinates) x Rings x scans


%LEavg = (2-(y,z) corrdinates) x Rings because the values are averaged
%over all scans
%REavg = (2-(y,z) corrdinates) x Rings because the values are averaged
%over all scans

%Rings is the list of rings in LE and RE
%RingsAvg is the list of rings in LEavg REavg
%
if nargin < 4
    ExpNmbr=1;
end

%% Select Scans to use for computing the SVD

%IndScans=[5:10]; % Selected Scans
%IndScans=[40:120]; % Selected Scans
% IndScans=[20:40]; % Selected Scans
IndScans=[1:20]; % Selected Scans
%IndScans=[50:100]; % Selected Scans
%IndScans=[50:150]; % Selected Scans
%

%% Place data in a form where rings and scans are identified
[n1,~,~]=size(pnts);
PayLoad=[];
n1 = IndScans(end);
for i1=1:n1
    points = pnts(i1,:,:);
    payload = (reshape(points, size(points, 2),[]))';
    RingNotZero=find(payload(5,:)>0);
    payload=payload(:,RingNotZero);
    [~,n3]=size(payload);
    payload=[payload;i1*ones(1,n3);ExpNmbr*ones(1,n3)];
    % if min(payload(1,:)) > 0
    if min(abs(payload(1,:))) > 0
        PayLoad=[PayLoad,payload];
    end
    
    % payload(1,:) : x
    % payload(2,:) : y
    % payload(3,:) : z
    % payload(4,:) : I
    % payload(5,:) : R
    % payload(6,:) : scan
    % payload(7,:) : ExpNmbr
end

%% Find Rings
FR=min(PayLoad(5,:));
LR=max(PayLoad(5,:));
RingNumbers=[];
for i=FR:LR
    K=find(PayLoad(5,:)==i);
    if length(K)>0
        RingNumbers=[RingNumbers,i];
    end
end

if base_line.show_results
    current_img_handle = base_line.img_hangles(1);
    hold(current_img_handle, 'on');
    scatter3(current_img_handle, PayLoad(1,:), PayLoad(2,:), PayLoad(3,:), '.'), view(-90,3)  
    axis(current_img_handle,'equal')
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    zlabel(current_img_handle, 'z')
    title(current_img_handle, 'Original Data')
    hold(current_img_handle, 'off');
    set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
end

%% Clean Data 
meanData=mean(PayLoad(1:3,:),2);
error=abs(PayLoad(1:3,:)-meanData);
distance=sum(error,1);
K=find(distance < d*1.025);
PayLoadClean=PayLoad(:,K);
meanClean=mean(PayLoadClean(1:3,:),2);

if base_line.L1_cleanup 
    opt.H_TL.rpy_init = [45 2 3];
    opt.H_TL.T_init = [2, 0, 0];
    opt.H_TL.H_init = eye(4);
    opt.H_TL.method = "Constraint Customize"; 
    opt.H_TL.UseCentroid = 1;
    [~, ~, clean_up_indices, ~] = cleanLiDARTargetWithOneDataSetWithIndices(PayLoadClean, d/sqrt(2), opt.H_TL);
    PayLoadClean=PayLoad(:, clean_up_indices);
end

% Check for entire rings being removed
FirstRing=min(PayLoadClean(5,:));
LastRing=max(PayLoadClean(5,:));

RingNumbers=[FirstRing:1:LastRing];
NRings=length(RingNumbers);

if base_line.show_results
    current_img_handle = base_line.img_hangles(2);
    hold(current_img_handle, 'on');
    scatter3(current_img_handle, PayLoadClean(1,:), PayLoadClean(2,:), PayLoadClean(3,:), '.'),  view(current_img_handle, -90,3)
    set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
    axis(current_img_handle,'equal')
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    zlabel(current_img_handle,'z')
    title(current_img_handle, 'Cleaned Up Data')
    hold(current_img_handle, 'off');
end

%% Build a projection to a plane that will be used to find Edge Data
K=find( and(( PayLoadClean(6,:) > IndScans(1) ),( PayLoadClean(6,:) < IndScans(end))  ));

XYZ=PayLoadClean(1:3,K);
meanXYZ=mean(XYZ,2);
[Uc,Sc,Vc]=svd(XYZ-meanXYZ);
[Uc,Vc] = FixSignsRotation(Uc,Vc);
%Sc(:,1:3),Uc

if abs(Uc(2,1)) > abs(Uc(3,1))
    Ind2D=[1,2];
    flag_changed = 0;
else
    Ind2D=[2,1];
    flag_changed = 1;
end
% Ind2D
NScans=max(PayLoadClean(6,:))- min(PayLoadClean(6,:));

% Uc; is used for the projection;

%% Project to a plane, find ring lines and the edges of the target edges
Data=PayLoadClean(1:3,:);
temp=Uc'*(Data-mean(Data,2));
PayLoadClean2D=temp(Ind2D,:); %Project out the distance component

if base_line.show_results
    current_img_handle = base_line.img_hangles(3);
    hold(current_img_handle, 'on');
    scatter(current_img_handle, PayLoadClean2D(1,:), PayLoadClean2D(2,:), '.b')
    set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
    view(current_img_handle, -180, 90)
    axis(current_img_handle, 'equal')
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    title(current_img_handle, 'Projected 2D points')
%     hold(current_img_handle, 'off');
end

% loop over with target shaped as a diamond
LE=10*ones(2,NRings,NScans); RE=LE; i=0;
for j=1:NRings
    J=find(PayLoadClean(5,:)==RingNumbers(j));
    NJ=length(J);
    if NJ > 0
        DataCenteredRotated=PayLoadClean(:,J);DataCenteredRotated(1:3,:)=Uc'*(DataCenteredRotated(1:3,:)-meanClean);
        i=i+1;
    else
%         j,i
%         RingNumbers(j)
%         disp('WTF ?')
        break
    end
    % Code to find the edges of the target
    
    for k = 1:NScans
        K=find(DataCenteredRotated(6,:)==k);
        if length(K)>0
        [L,IL]=max(DataCenteredRotated(Ind2D(1),K));
        LE(:,i,k)=DataCenteredRotated(Ind2D,K(IL(1)));  %LeftEdge(i,1+j-FirstRing,:)=LE;
        [R,IR]=min(DataCenteredRotated(Ind2D(1),K));
        RE(:,i,k)=DataCenteredRotated(Ind2D,K(IR(1))); %RightEdge(i,1+j-FirstRing,:)=RE;
         else
%             i,j,k;
%             disp('Problem with Missing Ring Data')
        end
    end
end
Iend=i;

%find the rings for the various parts of the diamond
LEavg=zeros(2,Iend);
for i=1:Iend
    LEtemp=squeeze(LE(:,i,:));
    I=find( (LEtemp(1,:)~= 10) & (LEtemp(2,:)~= 10) ); 
    LEtemp=LEtemp(:,I);
    LEavg(:,i)=mean(LEtemp,2);
    REtemp=squeeze(RE(:,i,:));
    REtemp=REtemp(:,I);
    REavg(:,i)=mean(REtemp,2);
end
U=Uc;
center=meanClean;

% %Pick out Rings for LE and RE
[ymin,iRing]=min(REavg(1,:));
RElower=RE(:,1:iRing,:);
REupper=RE(:,iRing:end,:);
[ymax,iRing]=max(LEavg(1,:));
I=find(RingNumbers<= iRing);
LElower=LE(:,1:iRing,:);
I=find(RingNumbers >= iRing);
LEupper=LE(:,iRing:end,:);
% [ymin,iRing]=min(REavg(1,:));
% RElower=RE(:,1:iRing,:);
% none_ten = find((RElower(1,:)~= 10) & (RElower(2,:)~= 10)); 
% RElower=RElower(:,none_ten);
% 
% REupper=RE(:,iRing:end,:);
% none_ten = find((REupper(1,:)~= 10) & (REupper(2,:)~= 10)); 
% REupper=REupper(:,none_ten);
% 
% % none_ten = find((REupper(1,:)~= 10) & (REupper(2,:)~= 10)); 
% [ymax,iRing]=max(LEavg(1,:));
% I=find(RingNumbers<= iRing);
% LElower=LE(:,1:iRing,:);
% none_ten = find((LElower(1,:)~= 10) & (LElower(2,:)~= 10)); 
% LElower=LElower(:,none_ten);
% 
% I=find(RingNumbers >= iRing);
% LEupper=LE(:,iRing:end,:);
% none_ten = find((LEupper(1,:)~= 10) & (LEupper(2,:)~= 10)); 
% LEupper=LEupper(:,none_ten);

if base_line.show_results
    current_img_handle = base_line.img_hangles(3);
    plot(current_img_handle, PayLoadClean2D(1,:), PayLoadClean2D(2,:), '.k')
    hold(current_img_handle, 'on')
    scatter(current_img_handle, LEupper(1, :), LEupper(2, :), 'ro', 'filled')
    scatter(current_img_handle, LElower(1, :), LElower(2, :), 'go', 'filled')
    scatter(current_img_handle, REupper(1, :), REupper(2, :), 'bo', 'filled')
    scatter(current_img_handle, RElower(1, :), RElower(2, :), 'mo', 'filled')
%     scatter(current_img_handle, edge1(1, :), edge1(2, :), 'ro', 'filled')
%     scatter(current_img_handle, edge2(1, :), edge2(2, :), 'go', 'filled')
%     scatter(current_img_handle, edge3(1, :), edge3(2, :), 'bo', 'filled')
%     scatter(current_img_handle, edge4(1, :), edge4(2, :), 'mo', 'filled')
    set(get(current_img_handle, 'parent'),'visible','on');
    view(current_img_handle, -180, 90)
    axis(current_img_handle,'equal');
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    title(current_img_handle, 'Edge points')
    hold(current_img_handle, 'off');
end

end

function [U,V] = FixSignsRotation(U,V)
%Fix the signs
Temp=abs(U);
[junk,I]=max(Temp,[],1);
%[sign(U(I(1),1)),sign(U(I(2),2)),sign(U(I(3),3))]
Signs=diag([sign(U(I(1),1)),sign(U(I(2),2)),sign(U(I(3),3))]);
U=U*Signs;
V(:,1:3)=V(:,1:3)*Signs;
end

