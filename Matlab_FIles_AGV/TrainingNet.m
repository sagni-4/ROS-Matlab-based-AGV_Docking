input_dataset='/home/sagni/Desktop/ThesisMatlab/DNN';
images=imageDatastore(input_dataset,"IncludeSubfolders",true,'LabelSource','foldernames');
numTrainFiles=15;
[input_image,TestImage]=splitEachLabel(images,numTrainFiles,"randomize");

correct_output=[1 0 0 0 
                0 1 0 0 
                0 0 1 0 
                0 0 0 1 
                ];
w1=2*rand(20,1115136)-1;
w2=2*rand(20,20)-1;
w3=2*rand(20,20)-1;
w4=2*rand(4,20)-1;
for epoch=1:10000
    [w1,w2,w3,w4]=DeepLearning(w1,w2,w3,w4,input_image,correct_output);
end
save('DeepNeuralNetwork.mat')

