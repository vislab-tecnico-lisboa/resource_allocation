function DefaultMask = makingDefaultMask()

% load('almostGoodMask.mat')
% 
% paddedImage = zeros(128,64,3);
% imshow(paddedImage)

DefaultMask{1} = zeros(128,64);
DefaultMask{2} = zeros(128,64);
DefaultMask{3} = zeros(128,64);
DefaultMask{4} = zeros(128,64);

DefaultMask{1}(6:22,24:40)=1;
DefaultMask{2}(24:62,19:46)=1;
DefaultMask{3}(64:93,21:43)=1;
DefaultMask{4}(95:126,21:43)=1;
% imshow(DefaultMask{4})
% plotBodyPartMasks(paddedImage,DefaultMask)

