function d=bhattacharyya(X1,X2)
% Bhattacharyya

avoid0 = realmin;
 
% VTd = double([X2; F97(testIm,:)]);
VTd = double(X2);
[nImages,histLen] = size(VTd); % VT should be nImages x histLen
normVec = avoid0 + sum(VTd,2); % vector with sum of each ped hist
hists_i = sqrt(VTd ./ normVec(:,ones(1,histLen))); % each hist normalized to 1, sqrt'ed
norm1 = avoid0 + sum(X1);
hist_test = sqrt(X1/norm1);
d = real(sqrt(1 - hists_i * hist_test')); % each element squared, 1 - , sqrt'ed
