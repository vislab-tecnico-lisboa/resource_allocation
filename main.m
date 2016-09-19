clear
close all

% add resource allocation stuff
addpath(genpath('resource_allocation'));

% add detector (Dollar) stuff
addpath(genpath('lib'));
load('lib/toolbox/detector/models/LdcfInriaDetector.mat');

% add dario's color features
addpath(genpath('colorfeatures'));

DefaultMask = makingDefaultMask();

% add images dir
image_dir = 'data/train/TUD-Stadtmitte/img1/';

% Create System objects used for reading video, detecting moving objects, and displaying the results.
obj = setupSystemObjects('data/train/TUD-Stadtmitte/img1/*.jpg');

% Load the image files. Each image, a frame
image_files = dir([image_dir '*.jpg']);
n_files = length(image_files);

% Load the provided detections from a dataset.
%detections = csvread('data/train/MOT16-02/det/det.txt');
%detections = csvread('data/train/MOT16-02/gt/gt.txt');

% x = [u v s u_v v_v s_v]
%% tracking parameters

%create/destroy parameters
invisibleForTooLong = 5;
ageThreshold = 5;
minVisibleCount = 1;

%Dummy value while
detThr = 30;

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

%% optimization parameters
capacity_constraint=0.3; % percentage of image to be process at each time instant
max_items=4;             % max regions to be process
time_horizon=2;          % planning time horizon (To do: now its 1 by default)
max_simulation_time_millis=10;
simulation_depth=3;
alpha_c=0.1;
alpha_s=0.1;
overlap_ratio=0.1;
% min size 128x52
min_width=96;
min_height=128;


%v = VideoReader('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(imread([image_dir image_files(1).name]));
%% Initialize pedestrian detector
%detector=initializeDetector();
min_scale=2.0;
%% Initialize trackers
tracks = initializeTracks(min_height); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Initialize resource contraint policy optimizer
optimization_=initializeMCTS(...
    frame_size(2),...
    frame_size(1),...
    capacity_constraint,...
    max_items,...
    min_width,...
    min_height,...
    max_simulation_time_millis,...
    simulation_depth);

%% Detect moving objects, and track them across video frames.
detection_times=[];
optimization_times=[];
tracking_times=[];
results=[];
for frame_number=1:n_files
    %Load image
    frame = imread([image_dir image_files(frame_number).name]);
    
    %% dynamic resource allocation (POMDP - input current belief; output actions)
    if max_items<size(tracks,2)
        optimization_=initializeMCTS(...
            frame_size(2),...
            frame_size(1),...
            capacity_constraint,...
            max_items,...
            min_width,...
            min_height,...
            max_simulation_time_millis,...
            simulation_depth);
    else
        optimization_=initializeMCTS(...
            frame_size(2),...
            frame_size(1),...
            capacity_constraint,...
            size(tracks,2),...
            min_width,...
            min_height,...
            max_simulation_time_millis,...
            simulation_depth);
    end
    
    
    [action,optimization_time]=compute_action(tracks,optimization_);
    optimization_times=[optimization_times optimization_time];
    rois=compute_rois(tracks,action,min_width,min_height,alpha_c,alpha_s);
    %rois=[];
    clear detection_bboxes;
    BB=[];
    
    tic
    %merge overlapping rois
    if size(rois)>0
        i=1;
        while i<size(rois,1)
            % discard too small bbs
            real_height=min(rois(i,2)+rois(i,4),frame_size(1))- max(rois(i,2),0) ;
            real_width=min(rois(i,1)+rois(i,3),frame_size(2)) - max(rois(i,1),0);
                   
            if real_height*real_width<min_height*min_width || real_height<min_height || real_width<min_width
                %rois(i,3)=min_width;
                %rois(i,4)=min_height;
                rois(i,:)=[];
                continue;
            end
            j=i+1;
            while j<size(rois,1)
               
                if bboxOverlapRatio(rois(i,:),rois(j,:)) >overlap_ratio
                    upper_x=min(rois(i,1),rois(j,1));
                    upper_y=min(rois(i,2),rois(j,2));
                    down_x=max(rois(i,1)+rois(i,3),rois(j,1)+rois(j,3));
                    down_y=max(rois(i,2)+rois(i,4),rois(j,2)+rois(j,4));
                    new_width=down_x-upper_x;
                    new_height=down_y-upper_y;
                    rois(i,:)=[upper_x upper_y new_width new_height];
                    rois(j,:)=[];
                    continue;
                end
                j=j+1;
            end
            i=i+1;
        end
        
        for i=1:size(rois,1)
            image_bounding_box=imcrop(frame,rois(i,:));
            
            preBB = acfDetect(image_bounding_box, detector);
            
            %Filter out detections with bad score
            BB = [BB; preBB(preBB(:, 5) > detThr, :)];
            if rois(i,1)<0
                BB(:,1) = BB(:,1);
            else
                BB(:,1) = BB(:,1) + rois(i,1);
            end
            
            if rois(i,2)<0
                BB(:,2) = BB(:,2);
            else
                BB(:,2) = BB(:,2) + rois(i,2);
            end
        end
    else
        %preBB = detections(detections(:,1) == frame_number,:);
        
        preBB = acfDetect(frame, detector);
        
        %Filter out detections with bad score
        BB = preBB(preBB(:, 5) > detThr, :);
    end
    detection_bboxes=[BB(:,1) BB(:,2) BB(:,3) BB(:,4)];
    detection_centroids=[BB(:,1)+BB(:,3)*0.5 BB(:,2)+BB(:,4)*0.5];
    
    detection_times=[detection_times toc];
    
    %% Extract Dario's color features
    tic
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
        
        beginY = round(beginY);
        endY = round(endY);
        beginX = round(beginX);
        endX = round(endX);
        
        person = frame(beginY:endY,beginX:endX,:);
        
        [paddedImage, smallPaddedImage] = smartPadImageToBodyPartMaskSize(person);
        
        BVTHistograms(i,:) = extractBVT(smallPaddedImage,DefaultMask);
        
    end
    
    
    %% tracking
    
    %predict
    tracks=predict(tracks);
    
    %associate
    [assignments, unassignedTracks, unassignedDetections ] = associateData( tracks, BVTHistograms, detection_centroids );
    
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
    
    tracking_times=[tracking_times toc];
    
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

average_optimization_time=mean(optimization_times);
average_detection_time=mean(detection_times);
average_total_time=average_optimization_time+average_detection_time;
average_frame_rate=1.0/average_total_time;

%% The results must be wriiten in the MoTChallenge res/data/[datasetname.txt] for evaluation
csvwrite('data/res/TUD-Stadtmitte.txt', results);

%% The evaluation script has 3 arguments:
%   1 - A txt that is a list of the datasets to be evaluated (file in the
%  seqmaps folder).
%   2 - A directory containing the results
%   3 - The benchmark directory
allMets = evaluateTracking('all.txt', 'data/res/', 'data/train/');
MOTA = allMets.bmark2d(12);
