function[Dockstat1_detect_accuracy,Dockstat2_detect_accuracy,Dockstat3_detect_accuracy,Dockstat4_detect_accuracy,]=CNN_DockstationDetector(Scanmsg,pozyxmsg,DockStaCord) 
% rosshutdown;
% clear;
% clc;
% rosinit;
% DockSta1Cord=[-0.0380 -8 0];
% DockSta2Cord=[8.15 -1.5 0];
% DockSta3Cord=[3.66 7.63 0];
% DockSta4Cord=[-3.88 2.03 0];
% DockStaCord=DockSta4Cord;
% distD=71;
windowsize1=10;
windowsize2=15;
window=zeros(windowsize1,1);
while(1)
% uwb_data=rossubscriber("/localization_data_topic","DataFormat","struct");
data1=receive(pozyxmsg);
x1=data1.Point.X;
y1=data1.Point.Y;
dist=sqrt((x1-DockStaCord(1))^2+(y1-DockStaCord(2))^2)
if (dist<=4)
for k=1:windowsize1
% x=rossubscriber("/scan","DataFormat","struct"); 
% uwb_data=rossubscriber("/localization_data_topic","DataFormat","struct");
scanData=receive(Scanmsg);
data1=receive(pozyxmsg);
x1=data1.Point.X;
y1=data1.Point.Y;
scans = rosReadLidarScan(scanData);
angles = rosReadScanAngles(scanData);
cart = rosReadCartesian(scanData);
maxRange = 15; % meters
resolution = 10; % cells per meter

slamObj = lidarSLAM(resolution,maxRange);
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

for i = 1:numel(scans)
    addScan(slamObj,scans(i));
    if rem(i,10) == 0
        show(slamObj);
    end
end

[scansSLAM,poses] = scansAndPoses(slamObj);
occMap = buildMap(scansSLAM,poses,resolution,maxRange);

figure(1)
show(occMap)

% axis off

title('')
% imwrite(getframe(gcf).cdata, 'mygridmap.png')
% ax = gca;
set(gca,'Visible','off');
set(gca,'XColor','none','YColor','none','TickDir','out');
% Requires R2020a or later
%  t = sprintf('stat4_%d.png',distD); % If you want to add a string variable to some constant text.
 t = sprintf('stat.png');
% F = getframe(gca);
% imwrite(F.cdata, t);
exportgraphics(gca, t ,'Resolution',400) ;
% pause(4)

%TESTING OBJECT DETECTION ALGORITHM

load('dcnn_network_50e.mat')
I=imread('/home/sagni/Desktop/ThesisMatlab/stat.png');
I=rgb2gray(I);
I=imresize(I,[1024,1024]);
figure(2)
imshow(I)
label=classify(dcnn_network_50e,I);

window(k,:)=label;
x=sprintf('detected_stat_%d.png',char(label));
title(['Docking station recognized is ' char(label)]);
exportgraphics(gca, x ,'Resolution',400) ;
   
   count1=0;count2=0;count3=0;count4=0; % counter for the number each docking station detected per window of size windosize1;
   if k==10
   for j=1:windowsize1
       if window(j,:)==1
           count1=count1+1;
       elseif window(j,:)==2
           count2=count2+1;
       elseif window(j,:)==3
           count3=count3+1;
       elseif window(j,:)==4
           count4=count4+1;
       end
   end
   Dockstat1_detect_accuracy=count1/10;
   Dockstat2_detect_accuracy=count2/10;
   Dockstat3_detect_accuracy=count3/10;
   Dockstat4_detect_accuracy=count4/10;
   end
end

end
break;
end