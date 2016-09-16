function [ assignment, unassignedTracks, unassignedDetections ] = associateData( trackers, bvtHists )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

nTracks = size(trackers, 2);
nDetections = size(bvtHists, 1);

costMat = zeros(nTracks, nDetections);

for i=1:nTracks
   for j=1:nDetections
       costMat(i, j) = bhattacharyya(trackers(i).bvtHist, bvtHists(i, :));
   end
end

[assignmentPre,cost] = munkres(costMat);

numOfNonAssocTracks = sum(assignmentPre(:)==0);

assignment = zeros(nTracks-numOfNonAssocTracks, 2);
unassignedTracks = zeros(numOfNonAssocTracks, 1);
unassignedDetections = 

j=1;
l=1;
for i=1:size(assignment, 1)
   %If the tracker has an assigned detection
   if assignmentPre(i) > 0
     assignment(j,1) =  trackers(i).id;
     assignment(j,2) = assignmentPre(i);
     j = j+1;
   else
     unassignedTracks(l) = trackers(i).id;
     l = l + 1;
   end
   
   

end




end

