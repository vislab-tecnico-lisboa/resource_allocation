function tracks=updateUnassignedTracks(tracks,unassignedTracks)
for i = 1:length(unassignedTracks)
    ind = unassignedTracks(i);
    
    for lol=1:size(tracks, 2)
        if tracks(lol).id == ind
            trackPosition = lol;
        end
    end
    
    tracks(trackPosition).age = tracks(trackPosition).age + 1;
    
    if tracks(trackPosition).attended==1
        tracks(trackPosition).consecutiveInvisibleCount = ...
            tracks(trackPosition).consecutiveInvisibleCount + 1;
    end
end
end