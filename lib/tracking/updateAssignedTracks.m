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
    correct(tracks(trackPosition).stateKalmanFilter, [centroid bbox(4)/tracks(trackPosition).min_height]);
    
    % Replace predicted bounding box with detected
    % bounding box.
    sc = tracks(trackPosition).stateKalmanFilter.State(3);
    center = tracks(trackPosition).stateKalmanFilter.State(1:2);
    min_height = tracks(trackPosition).min_height;
    width = 40*sc;
    height = min_height*sc;
    
    bbox(1) = center(1)-width/2;
    bbox(2) = center(2)-height/2;
    bbox(3) = width;
    bbox(4) = height;
    
    tracks(trackPosition).bbox = bbox;
    
    % Update track's age.
    tracks(trackPosition).age = tracks(trackPosition).age + 1;

    % Update visibility.
    tracks(trackPosition).totalVisibleCount = ...
        tracks(trackPosition).totalVisibleCount + 1;
    tracks(trackPosition).consecutiveInvisibleCount = 0;
    
    % Update the color histogram
    tracks(trackPosition).colorHist = tracks(trackPosition).colorHist*0.1+bvtHist*0.9;
    
end
end