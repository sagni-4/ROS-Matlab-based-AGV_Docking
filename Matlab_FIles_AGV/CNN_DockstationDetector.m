function[Dockstat1_detect_accuracy,Dockstat2_detect_accuracy,Dockstat3_detect_accuracy,Dockstat4_detect_accuracy]=CNN_DockstationDetector(Scanmsg,window,windowsize,k) 
% clear all;
% clc;
% rosshutdown;
% rosinit;

count1=0;count2=0;count3=0;count4=0;
% windowsize=5;
% window=zeros(windowsize,1);
% k=1;
% while(1)
% x=rossubscriber("/scan","DataFormat","struct"); 
% Scanmsg=receive(x);
% r = rosrate(10);
% for k=1:windowsize1
scans = rosReadLidarScan(Scanmsg);
angles = rosReadScanAngles(Scanmsg);
cart = rosReadCartesian(Scanmsg);
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
title('')
set(gca,'Visible','off');
set(gca,'XColor','none','YColor','none','TickDir','out');
% Requires R2020a or later
%  t = sprintf('stat4_%d.png',distD); % If you want to add a string variable to some constant text.
t = sprintf('stat.png');
exportgraphics(gca, t ,'Resolution',400) ;

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
   
    % counter for the number each docking station detected per window of size windosize1;
%    if k==10
   for j=1:windowsize
       if window(j,:)==1
           count1=count1+1;
       elseif window(j,:)==2
           count2=count2+1;
       elseif window(j,:)==3
           count3=count3+1;
       elseif window(j,:)==4
           count4=count4+1;
       end
%     
    end
   k=k+1;
   den=count1+count2+count3+count4;
   Dockstat1_detect_accuracy=count1/den;
   Dockstat2_detect_accuracy=count2/den;
   Dockstat3_detect_accuracy=count3/den;
   Dockstat4_detect_accuracy=count4/den;
%   end
% end
    count1=0;
    count2=0;
    count3=0;
    count4=0;
if k==(windowsize+1)
    k=1;
end
% end
end

