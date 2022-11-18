
clear;
clc;
input_dataset='/home/sagni/Desktop/ThesisMatlab/DCNN_IMAGES';
images=imageDatastore(input_dataset,"IncludeSubfolders",true,'LabelSource','foldernames');
numTrainFiles=70;
[TrainImages,TestImages]=splitEachLabel(images,numTrainFiles,"randomize");
 layers =[
       imageInputLayer(([1024,1024 1]),'Name','Input')
          
       convolution2dLayer(3,8,'Padding','same','Name','Conv_1')
       batchNormalizationLayer('Name','BN_1')
       reluLayer('Name','Relu_1')
       maxPooling2dLayer(2,'stride',2,'Name','MaxPool_1')
           
       convolution2dLayer(3,16,'Padding','same','Name','Conv_2')
       batchNormalizationLayer('Name','BN_2')
       reluLayer('Name','Relu_2')
       maxPooling2dLayer(2,'stride',2,'Name','MaxPool_2')

       convolution2dLayer(3,32,'Padding','same','Name','Conv_3')
       batchNormalizationLayer('Name','BN_3')
       reluLayer('Name','Relu_3')
       maxPooling2dLayer(2,'stride',2,'Name','MaxPool_3')

       convolution2dLayer(3,64,'Padding','same','Name','Conv_4')
       batchNormalizationLayer('Name','BN_4')
       reluLayer('Name','Relu_4')

       fullyConnectedLayer(4,'Name','FC')
       softmaxLayer('Name','SoftMax');
       classificationLayer('Name','Output Classification');
       ]; 
         lgraph = layerGraph(layers);
         plot(lgraph);
         options=trainingOptions('sgdm', 'InitialLearnRate', 0.01, 'MaxEpochs',20, 'shuffle','every-epoch','ValidationData', TestImages, 'ValidationFrequency',5, 'Verbose',false,'Plots','training-progress');
         dcnn_network_50e= trainNetwork(TrainImages,layers,options);
         Ypred = classify(dcnn_network_50e,TestImages);
         Yvalidation=TestImages.Labels;
         accuracy=sum(Ypred==Yvalidation)/numel(Yvalidation)
         save('dcnn_network_50e.mat')


