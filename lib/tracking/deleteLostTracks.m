function tracks=deleteLostTracks(tracks,...
    invisibleForTooLong)
if isempty(tracks)
    return;
end




lostInds = [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;


% Delete lost tracks.
tracks = tracks(~lostInds);

lostInds=[];
% check which tracks are outside the image and delete
for i=1:size(tracks,2)
    x=tracks(1,i).stateKalmanFilter.State(1);
    y=tracks(1,i).stateKalmanFilter.State(2);
    s=tracks(1,i).stateKalmanFilter.State(3);
    if x >0 && y>0 && s>0
        lostInds=[lostInds i];
    end
end
% Delete lost tracks.
tracks = tracks(lostInds);
end