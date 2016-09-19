function tracks=deleteLostTracks(tracks,...
    invisibleForTooLong,...
    ageThreshold)
if isempty(tracks)
    return;
end


% Compute the fraction of the track's age for which it was visible.
ages = [tracks(:).age];
totalVisibleCounts = [tracks(:).totalVisibleCount];
visibility = totalVisibleCounts ./ ages;

% Find the indices of 'lost' tracks.
%lostInds = (ages < ageThreshold & visibility < 0.6) | ...
%    [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;

lostInds = [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;


% Delete lost tracks.
tracks = tracks(~lostInds);

lostInds=[];
% check which tracks are outside the image and delete
for i=1:size(tracks,2)
    x=tracks(1,i).bbox(1);
    y=tracks(1,i).bbox(2);
    
    if x >0 && y>0
        lostInds=[lostInds i];
    end
end
% Delete lost tracks.
tracks = tracks(lostInds);
end