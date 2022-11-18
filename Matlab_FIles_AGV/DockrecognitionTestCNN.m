
load('dcnn_network_50e.mat')
[filename,pathname]=uigetfile('*.*','select the input image');
filewithpath=strcat(pathname,filename);
I=imread(filewithpath);
figure
imshow(I)
label=classify(dcnn_network_50e,I);
title(['Docking station recognized is ' char(label)])