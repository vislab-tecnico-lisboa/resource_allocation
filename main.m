close all

% add resource allocation stuff
addpath(genpath('resource_allocation'));

% add detector (Dollar) stuff
addpath(genpath('libs'));

% min size 128x52
min_width=41;
min_height=100;
% x = [u v s u_v v_v s_v]
% tracking parameters
invisibleForTooLong = 10;
ageThreshold = 5;
minVisibleCount = 3;


state_transition_model=[1 0 0; 0 1 0; 0 0 1]; % constant position
state_measurement_model=[1 0 0; 0 1 0; 0 0 1];
state_init_state_covariance=[100 0 0; 0 100 0; 0 0 2.0];
state_process_noise=[50 0 0; 0 50 0; 0 0 1.0];
state_measurement_noise=[10 0 0; 0 10 0; 0 0 1.0];
costOfNonAssignmentState=100;

% optimization parameters
capacity_constraint=0.3; % percentage of image to be process at each time instant
max_items=11;            % max regions to be process (To do) IT EXPLODES RIGHT NOW!!! FIX
time_horizon=2;          % planning time horizon (To do: now its 1 by default)



%% ACF AND CNN PARAMS

% -------------
% CHOOSE parameters
% method used to generate proposals
method = 'ACF'; %'LDCF' or 'ACF';

% dataset used to train the CNN model (already pre-trained with Imagenet)
% now the VGG-VD16 CNN model is being used
dataset = 'INRIA'; %'INRIA' ou 'Caltech';

% choose CNN model architecture
modelCNN = 'VGG_VD16'; % the mean is not computed

%threshold for the probabilities in the CNN
probability_thresh=0.5;
% threshold the proposals score
enableThresh=1;
thresh=40; %45;

% % resize source images if necessary
enableResize=0;
resizeFactor=2;
% change sliding window step size
changeStepSize=0;
stepSize=8; %default is 4
% -------------


%% Create System objects used for reading video, detecting moving objects,
% and displaying the results.
obj = setupSystemObjects('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
v = VideoReader('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(read(v,1));
%% Initialize pedestrian detector
%detector=initializeDetector();

%% Initialize trackers
tracks = initializeTracks(); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Initialize resource contraint policy optimizer
darap=initializeDARAP(frame_size(2), frame_size(1),capacity_constraint,max_items,min_width,min_height);
%% Detect moving objects, and track them across video frames.
detection_times=[];
optimization_times=[];
tracking_times=[];
while ~isDone(obj.reader)
    frame=readFrame(obj);
    width=size(frame,2);
    height=size(frame,1);
    x=width-width;
    y=height-height;
    
    %% dynamic resource allocation
%     [rois,optimization_time]=imageProb(tracks,darap);
%     optimization_times=[optimization_times optimization_time];
%     
%     probability_maps=get_probability_maps(darap);
    
    rois=[];
    %     for p=size(probability_maps,2)
    %     figure(1),imagesc(probability_maps{1,p});
    %     end
    %rois=[];
    %% detection
    %     [detection_centroids,...
    %         detection_bboxes,...
    %         detection_time]=...
    %         detectObjects(detector,...
    %         uint8(frame*256),...
    %         x,y,width,height,rois);
    %     detection_times=[detection_times detection_time];
    
    %% detection with ACF
    detection_images=[];
    % get cropped images from rois
    if isempty(rois)
        rois=[1 1 width height];
    end
    
    [candidates,~] = bbApply('crop',frame,rois,[],[]);

    for i=1:size(rois,1)
        candidate=candidates{i};
        % Proposals Mbot
        if(~enableResize || ~changeStepSize)
            % do not resize the images
            [detection_images,detection_bboxes] = extractProposals(candidate, method, dataset);
            
            %         elseif(changeStepSize)
            %             %change sliding window step size
            %             [proposals_Mbot,bb_Mbot,id_Mbot] = extractProposalsStepSize(dirImgsMbot, method, dataset,stepSize);
            %         else
            %             % resize the images
            %             [proposals_Mbot,bb_Mbot,id_Mbot] = extractProposalsResized(dirImgsMbot, method, dataset,resizeFactor);
        end
    end
    
    %% refine using cnn
    % if enabled, threshold the detection confidence score
    if(enableThresh)
        % Mbot: threshold the detection confidence score
        threshIndexsMbot=find(detection_bboxes(:,5)>thresh);
        detection_images = detection_images(:,:,:,threshIndexsMbot);
        detection_bboxes = detection_bboxes(threshIndexsMbot,:);
%         detection_ids = detection_ids(threshIndexsMbot,:);
    end
    
    if isempty(detection_images)
        continue
    end
    % CNN Classification Mbot
    detection_images=single(256*detection_images);
%   [pedestrian_indices] = CNN_classification(detection_images,dataset,modelCNN);
    [pedestrian_indices, pedestrian_probs] = CNN_classification_with_prob(detection_images,dataset,modelCNN,probability_thresh);
    detection_images = detection_images(:,:,:,pedestrian_indices);
    detection_bboxes = detection_bboxes(pedestrian_indices,:);
    
    detection_centroids=[detection_bboxes(:,1)+detection_bboxes(:,3)*0.5 detection_bboxes(:,2)+detection_bboxes(:,4)*0.5];
    
%     detection_ids = detection_ids(pedestrian_indices_Mbot,:);
    
    


    %% tracking
    %predict
    tracks=predictNewLocationsOfTracks(tracks);
    %associate
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment(tracks,...
        detection_centroids,...
        detection_bboxes,...
        costOfNonAssignmentState);
    %update
    tracks=updateAssignedTracks(tracks,assignments,detection_centroids,detection_bboxes);
    tracks=updateUnassignedTracks(tracks,unassignedTracks);
    tracks=deleteLostTracks(tracks,...
        invisibleForTooLong,...
        ageThreshold);
    
    [tracks,nextId]=createNewTracks(tracks,...
        unassignedDetections,...
        detection_centroids,...
        detection_bboxes,...
        nextId,...
        state_transition_model,...
        state_measurement_model,...
        state_init_state_covariance,...
        state_process_noise,...
        state_measurement_noise);
    
    %% display results
    displayTrackingResults(obj,frame,tracks,detection_bboxes,minVisibleCount,rois);
    
end

average_optimization_time=mean(optimization_times);
average_detection_time=mean(detection_times);
average_total_time=average_optimization_time+average_detection_time;
average_frame_rate=1.0/average_total_time;

