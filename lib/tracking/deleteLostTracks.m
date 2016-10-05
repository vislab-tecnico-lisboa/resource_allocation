function tracks=deleteLostTracks(tracks,...
    invisibleForTooLong,...
    frame_size)
if isempty(tracks)
    return;
end

lostInds = [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;



% Delete lost tracks.
if ~isempty(lostInds)
    tracks = tracks(~lostInds);
end
lostInds=[];
% check which tracks are outside the image and delete
for i=1:size(tracks,2)
    x=tracks(1,i).stateKalmanFilter.State(1);
    y=tracks(1,i).stateKalmanFilter.State(2);
    s=tracks(1,i).stateKalmanFilter.State(3);
    if x<0 || y<0 || s<0
        lostInds=[lostInds 1];
    else
        lostInds=[lostInds 0];
    end
end
% Delete lost tracks.


if ~isempty(tracks) && ~isempty(lostInds)
    tracks = tracks(~lostInds);
end


lostInds=[];
for i=1:size(tracks,2)
    % check wich tracks are too small
    % discard too small bbs
    sc = tracks(i).stateKalmanFilter.State(3);
    center = tracks(i).stateKalmanFilter.State(1:2);
    min_height = tracks(i).min_height;
    min_width = tracks(i).min_width;
    
    width = min_width*sc;
    height = min_height*sc;
    
    bbox(1) = center(1)-width/2;
    bbox(2) = center(2)-height/2;
    bbox(3) = width;
    bbox(4) = height;
    
    real_height=min(bbox(2)+height,frame_size(1))- max(bbox(2),0) ;
    real_width=min(bbox(1)+width,frame_size(2)) - max(bbox(1),0);
    
    if (real_height*real_width)<(min_height*min_width) %|| real_height<min_height || real_width<min_width
        lostInds=[lostInds 1];
    else
        lostInds=[lostInds 0];
    end
end



if ~isempty(tracks) && ~isempty(lostInds)
    tracks = tracks(~lostInds);
end
end