function [ assignment, unassignedTracks, unassignedDetections ] = associateData( trackers, bvtHists )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


nTracks = size(trackers, 2);
nDetections = size(bvtHists, 1);

if nDetections > 0 
unassignedDetections = 1:nDetections;
else
unassignedDetections = [];
assignment = [];
unassignedTracks = zeros(nTracks,1);
for i = 1:nTracks
 unassignedTracks(i) = trackers(i).id;
end
return;
end

if nTracks < 1
assignment = [];
unassignedTracks = [];
return
end

costMat = zeros(nTracks, nDetections);

for i=1:nTracks
   for j=1:nDetections
       custo = bhattacharyya(trackers(i).colorHist, bvtHists(j, :));
       if custo > 0.6
       costMat(i, j) = 1000;
       else
       costMat(i, j) = custo; 
       end
   end
end

[assignmentPre,cost] = munkres(costMat);

numOfNonAssocTracks = sum(assignmentPre(:)==0);

assignment = zeros(nTracks-numOfNonAssocTracks, 2);
unassignedTracks = zeros(numOfNonAssocTracks, 1);


j=1;
l=1;
for i=1:size(assignmentPre, 2)
   %If the tracker has an assigned detection
   if assignmentPre(i) > 0
     assignment(j,1) =  trackers(i).id;
     assignment(j,2) = assignmentPre(i);
     unassignedDetections(unassignedDetections == assignmentPre(i)) = [];
     j = j+1;
   else
     unassignedTracks(l) = trackers(i).id;
     l = l + 1;
   end
   
   

end

end

