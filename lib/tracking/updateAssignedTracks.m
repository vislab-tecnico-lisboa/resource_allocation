function tracks=updateAssignedTracks(tracks,assignments,centroids,bboxes, bvtHists)
numAssignedTracks = size(assignments, 1);
for i = 1:numAssignedTracks
    trackIdx = assignments(i, 1);
    detectionIdx = assignments(i, 2);
    centroid = centroids(detectionIdx, :);
    bbox = bboxes(detectionIdx, :);
    bvtHist = bvtHists(detectionIdx, :);
    
    trackPosition = 0;
    
    for lol=1:size(tracks, 2)
        if tracks(lol).id == trackIdx
            trackPosition = lol;
        end
    end
    
    % Correct the estimate of the object's location
    % using the new detection.
    correct(tracks(trackPosition).stateKalmanFilter, [centroid bbox(4)/bbox(3)]);
    
    % Replace predicted bounding box with detected
    % bounding box.
    tracks(trackPosition).bbox = bbox;
    
    % Update track's age.
    tracks(trackPosition).age = tracks(trackPosition).age + 1;

    % Update visibility.
    tracks(trackPosition).totalVisibleCount = ...
        tracks(trackPosition).totalVisibleCount + 1;
    tracks(trackPosition).consecutiveInvisibleCount = 0;
    
    % Update the color histogram
    tracks(trackPosition).colorHist = tracks(trackPosition).colorHist*0.2+bvtHist*0.8;
    
end
end