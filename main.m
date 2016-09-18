clear
close all

% add resource allocation stuff
addpath(genpath('resource_allocation'));

% add detector (Dollar) stuff
addpath(genpath('lib'));

% add dario's color features
addpath(genpath('colorfeatures'));

DefaultMask = makingDefaultMask();

% add images dir
image_dir = 'data/train/MOT16-09/img1/';

% Load the image files. Each image, a frame
image_files = dir([image_dir '*.jpg']);
n_files = length(image_files);

% Load the provided detections from a dataset.
%detections = csvread('data/train/MOT16-09/det/det.txt');
detections = csvread('data/train/MOT16-09/gt/gt.txt');

% x = [u v s u_v v_v s_v]
%% tracking parameters

%create/destroy parameters
invisibleForTooLong = 30;
ageThreshold = 5;
minVisibleCount = 1;

%Dummy value while
detThr = -1000;

%transition/observation parameters
state_transition_model=...
    [1 0 0 1 0 0;...
    0 1 0 0 1 0;...
    0 0 1 0 0 1;...
    0,0,0,1,0,0;...
    0,0,0,0,1,0;...
    0,0,0,0,0,1]; % constant position
state_measurement_model=...
    [1 0 0 0 0 0;...
    0 1 0 0 0 0;...
    0 0 1 0 0 0];
state_init_state_covariance=[...
    100 0 0 100 0 0;...
    0 100 0 0 100 0;...
    0 0 100 0 0 100;...
    0 0 0 100 0 0;...
    0 0 0 0 100 0;...
    0 0 0 0 0 100];

state_process_noise=[...
    100.0 0 0 0 0 0;...
    0 100.0 0 0 0 0;...
    0 0 100.0 0 0 0;...
    0 0 0 100.0 0 0;...
    0 0 0 0 100.0 0;...
    0 0 0 0 0 0.0001];
state_measurement_noise=[...
    1 0 0;...
    0 1 0;...
    0 0 0.001];

state_init_state_covariance = state_init_state_covariance*10000;
state_process_noise = state_process_noise*10;
state_measurement_noise = state_measurement_noise*1000;
costOfNonAssignmentState=100000000;

%% optimization parameters
capacity_constraint=0.3; % percentage of image to be process at each time instant
max_items=1;             % max regions to be process
time_horizon=2;          % planning time horizon (To do: now its 1 by default)
max_simulation_time_millis=10;
simulation_depth=3;

% min size 128x52
min_width=41;
min_height=100;

%% Create System objects used for reading video, detecting moving objects,
% and displaying the results.
obj = setupSystemObjects('data/train/MOT16-09/img1/*.jpg');
%v = VideoReader('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(imread([image_dir image_files(1).name]));
%% Initialize pedestrian detector
%detector=initializeDetector();

%% Initialize trackers
tracks = initializeTracks(min_height); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Initialize resource contraint policy optimizer
% optimization_=initializeMCTS(...
%     frame_size(2),...
%     frame_size(1),...
%     capacity_constraint,...
%     max_items,...
%     min_width,...
%     min_height,...
%     max_simulation_time_millis,...
%     simulation_depth);

%% Detect moving objects, and track them across video frames.
detection_times=[];
optimization_times=[];
tracking_times=[];
results=[];
for frame_number=1:n_files
    %Load image
    frame = imread([image_dir image_files(frame_number).name]);
    
    %% dynamic resource allocation (POMDP - input current belief; output actions)
%     [rois,optimization_time]=compute_action(tracks,optimization_);
    %optimization_times=[optimization_times optimization_time];
    %
    %     probability_maps=get_probability_maps(darap);
    
    rois=[];
    
    clear detection_bboxes;
    preBB = detections(detections(:,1) == frame_number,:);
    
    %Filter out detections with bad score
   % preBB = preBB(preBB(:, 7) > detThr, :);
    
    detection_bboxes=[preBB(:,3) preBB(:,4) preBB(:,5) preBB(:,6)];
    detection_centroids=[preBB(:,3)+preBB(:,5)*0.5 preBB(:,4)+preBB(:,6)*0.5];
    
    
    %% Extract Dario's color features
    
    linsRect = size(detection_bboxes, 1);
    
    %Each line is a color histogram of each person. A BVT Histogram has 440
    %entries.
    clear BVTHistograms;
    BVTHistograms = zeros(linsRect, 440);
    
    for i=1:linsRect
        
        %If the bbox is out of the image would have an error... this avoids
        %that but might not be the best solution
        
        beginX = detection_bboxes(i,1);
        endX = detection_bboxes(i,1)+detection_bboxes(i,3);
        beginY = detection_bboxes(i,2);
        endY = detection_bboxes(i,2)+detection_bboxes(i,4);
        
        
        if endX > size(frame, 2)
            endX = size(frame, 2);
        end
        
        if beginX <1;
            beginX = 1;
        end
        if endY > size(frame, 1)
            endY = size(frame, 1);
        end
        if beginY < 1;
            beginY = 1;
        end
        
        person = frame(beginY:endY,beginX:endX,:);
        
        [paddedImage, smallPaddedImage] = smartPadImageToBodyPartMaskSize(person);
        
        BVTHistograms(i,:) = extractBVT(smallPaddedImage,DefaultMask);
        
    end
    
    
    %% tracking
    
    %predict
    tracks=predict(tracks);
    
    %associate
    [assignments, unassignedTracks, unassignedDetections ] = associateData( tracks, BVTHistograms );
    
    %update
    tracks=updateAssignedTracks(tracks,assignments,detection_centroids,detection_bboxes, BVTHistograms);
    tracks=updateUnassignedTracks(tracks,unassignedTracks);
    tracks=deleteLostTracks(tracks,...
        invisibleForTooLong,...
        ageThreshold);
    [tracks,nextId]=createNewTracks(tracks,...
        unassignedDetections,...
        detection_centroids,...
        detection_bboxes,...
        BVTHistograms,...
        min_height, ...
        nextId,...
        state_transition_model,...
        state_measurement_model,...
        state_init_state_covariance,...
        state_process_noise,...
        state_measurement_noise);
    
    %% display results
    displayTrackingResults(obj,frame,tracks,detection_bboxes,minVisibleCount,rois);
    
    % store results
    for i=1:length(tracks)
        conf=1;
        preResults = [frame_number, tracks(i).id, tracks(i).bbox(1), tracks(i).bbox(2), tracks(i).bbox(3), tracks(i).bbox(4), -1, -1, -1, -1];
        results=[results; preResults];
    end
    
    str_disp = sprintf('processing frame %d of %d',frame_number,n_files);
    disp(str_disp);
end

%average_optimization_time=mean(optimization_times);
%average_detection_time=mean(detection_times);
%average_total_time=average_optimization_time+average_detection_time;
%average_frame_rate=1.0/average_total_time;

%% The results must be wriiten in the MoTChallenge res/data/[datasetname.txt] for evaluation
csvwrite('data/res/MOT16-09.txt', results);

%% The evaluation script has 3 arguments:
%   1 - A txt that is a list of the datasets to be evaluated (file in the
%  seqmaps folder).
%   2 - A directory containing the results
%   3 - The benchmark directory
allMets = evaluateTracking('all.txt', 'data/res/', 'data/train/');
MOTA = allMets.bmark2d(12);
