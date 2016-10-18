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
%obj = setupSystemObjects('data/train/TUD-Stadtmitte/img1/*.jpg');

% Load the image files. Each image, a frame
image_files = dir([image_dir '*.jpg']);
n_files = length(image_files);

%% TWEEK
n_files=30;
% Load the provided detections from a dataset.
%detections = csvread('data/train/MOT16-02/det/det.txt');
%detections = csvread('data/train/MOT16-02/gt/gt.txt');

% x = [u v s u_v v_v s_v]
%% tracking parameters

%create/destroy parameters
invisibleForTooLong = 30;

%Dummy value while
detThr = 50;

%transition/observation parameters
state_transition_model=...
    [1 0 0 1 0 0;...
    0 1 0 0 1 0;...
    0 0 1 0 0 0;...
    0,0,0,1,0,0;...
    0,0,0,0,1,0;...
    0,0,0,0,0,0]; % constant position

state_measurement_model=...
    [1 0 0 0 0 0;...
    0 1 0 0 0 0;...
    0 0 1 0 0 0];

state_init_state_covariance=[...
    10 0 0 0 0 0;...
    0 10 0 0 0 0;...
    0 0  1.0 0  0 0;...
    0 0  0 10 0 0;...
    0 0  0 0 10 0;...
    0 0  0 0 0 1.0];

state_process_noise=[...
    10.0 0 0 0 0 0;...
    0 10.0 0 0 0 0;...
    0 0 0.1 0 0 0;...
    0 0 0 10.0 0 0;...
    0 0 0 0 10.0 0;...
    0 0 0 0 0 0.1];

state_measurement_noise=[...
    10 0 0;...
    0 10 0;...
    0 0 2.0];

%state_init_state_covariance = state_init_state_covariance;
%state_process_noise = state_process_noise;
%state_measurement_noise = state_measurement_noise;

%% optimization parameters
action_mode=1; %( 0 -> normal, 1->random)
max_items_=[4 3 2 1];             % max regions to be process
capacity_constraints_=[1.0 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1]; % percentage of image to be process at each time instant

max_simulation_time_millis=50;
simulation_depth=1;
alpha_c=0.5;
alpha_s=0.5;
overlap_ratio=0.5;

min_width=52;
min_height=128;

%v = VideoReader('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(imread([image_dir image_files(1).name]));
%% Initialize pedestrian detector
%detector=initializeDetector();

%% Detect moving objects, and track them across video frames.
num_exp=30;
average_detection_times_random=zeros(length(max_items_),length(capacity_constraints_),num_exp,1);
average_optimization_times_random=zeros(length(max_items_),length(capacity_constraints_),num_exp,1);
average_tracking_times_random=zeros(length(max_items_),length(capacity_constraints_),num_exp,1);
average_mot_random=zeros(length(max_items_),length(capacity_constraints_),num_exp,14);
iteration_=0;
total_iterations=length(max_items_)*length(capacity_constraints_)*num_exp*n_files;

%figure(1)
for c1=1:length(max_items_)
    max_items=max_items_(c1);
    for c2=1:length(capacity_constraints_)
        capacity_constraint=capacity_constraints_(c2);
        for exp=1:num_exp
            detection_times=[];
            optimization_times=[];
            tracking_times=[];
            results=[];
            %% Initialize trackers
            tracks = initializeTracks(min_width,min_height); % Create an empty array of tracks.
            nextId = 1; % ID of the next track

            for frame_number=1:n_files
                rois=[];
                iteration_=iteration_+1;
                
                str_disp = sprintf('processing iteration %d of %d (%f)',iteration_,total_iterations, (iteration_/total_iterations) * 100.0);
                disp(str_disp);
                
                %Load image
                frame = imread([image_dir image_files(frame_number).name]);
                
                %% dynamic resource allocation (POMDP - input current belief; output actions)
                if size(tracks,2)>0
                    
                    %predict
                    tracks=predict(tracks);
                    
                    if max_items<size(tracks,2)
                        optimization_=initializeMCTS(...
                            frame_size(2),...
                            frame_size(1),...
                            capacity_constraint,...
                            max_items,...
                            min_width,...
                            min_height,...
                            max_simulation_time_millis,...
                            simulation_depth,...
                            action_mode);
                    else
                        optimization_=initializeMCTS(...
                            frame_size(2),...
                            frame_size(1),...
                            capacity_constraint,...
                            size(tracks,2),...
                            min_width,...
                            min_height,...
                            max_simulation_time_millis,...
                            simulation_depth,...
                            action_mode);
                    end
                    
                    % Get action
                    [action,optimization_time,explored_actions,explored_nodes]=compute_action(tracks,optimization_,alpha_c,alpha_s);
                    
                    
                    
                    % Update attended tracks
                    for i=1:size(tracks,2)
                        tracks(i).attended=0;
                    end;
                    for i=1:size(tracks,2)
                        for j=1:size(action,2)
                            if action(j)==i
                                tracks(i).attended=1;
                            end
                        end
                    end
                    
                    optimization_times=[optimization_times optimization_time];
                    rois=compute_rois(tracks,action,min_width,min_height,alpha_c,alpha_s);
                    %rois=[];
                    clear detection_bboxes;
                    detection_bboxes=[];
                    
                    % DISCARD OR ADAPT INVALID ROIS
                    i=1;
                    while i<=size(rois,1)
                        if rois(i,3)<0 || rois(i,4)<0
                            rois(i,:)=[];
                            continue
                        end
                        
                         %merge overlapping bbxs
                        j=i+1;
                        while j<=size(rois,1)
                            if rois(j,3)<0 || rois(j,4)<0
                                rois(j,:)=[];
                                continue
                            end
                            
                            %merge if bbs they overlap more than overlap_ration
                            if bboxOverlapRatio(rois(i,1:4),rois(j,1:4)) >overlap_ratio
                                upper_x=min(rois(i,1),rois(j,1));
                                upper_y=min(rois(i,2),rois(j,2));
                                down_x=max(rois(i,1)+rois(i,3),rois(j,1)+rois(j,3));
                                down_y=max(rois(i,2)+rois(i,4),rois(j,2)+rois(j,4));
                                new_width=down_x-upper_x;
                                new_height=down_y-upper_y;
                                
                                rois(i,1:4)=[upper_x upper_y new_width new_height];
                                rois(j,:)=[];
                                continue;
                            end
                            j=j+1;
                        end
                        
                        % first guarantee that de BBs are inside the image
                        x_before=rois(i,1);
                        y_before=rois(i,2);
                        width_before=rois(i,3);
                        height_before=rois(i,4);
                        
                        %discard bb that are outside the image
                        if x_before>frame_size(2) || y_before>frame_size(1)
                            rois(i,:)=[]; continue; end
                        
                        %update bb upper left corner outside
                        x_after=max(x_before,0);
                        y_after=max(y_before,0);
                        
                        %update width and height
                        width_after=width_before - (x_after-x_before);
                        if width_after>frame_size(2)
                            width_after=frame_size(2)-x_after;
                        end
                        
                        height_after=height_before - (y_after-y_before);
                        if height_after>frame_size(1)
                            height_after=frame_size(1)-y_after;
                        end
                        
                        if x_after+width_after>frame_size(2)
                            width_after=frame_size(2)-x_after;
                        end
                        
                        if y_after+height_after>frame_size(1)
                            height_after=frame_size(1)-y_after;
                        end
                        
                        rois(i,1:4)=[x_after y_after width_after height_after];
                        
                        if width_after<min_width || height_after<min_height
                            rois(i,:)=[];
                            continue;
                        end
                        
                        % discard too small bbs
                        real_height=min(rois(i,2)+rois(i,4),frame_size(1))- max(rois(i,2),0) ;
                        real_width=min(rois(i,1)+rois(i,3),frame_size(2)) - max(rois(i,1),0);
                        if (real_height*real_width)<(min_height*min_width) %|| real_height<min_height || real_width<min_width
                            rois(i,:)=[];
                            continue;
                        end
                        
                        i=i+1;
                    end
                    
                    time_=0;
                    for i=1:size(rois,1)
                        
                        track_index=rois(i,5);
                        image_bounding_box=imcrop(frame,rois(i,1:4));
                        tic
                        preBB = acfDetect(image_bounding_box, detector);
                        time_aux=toc;
                        time_=time_+time_aux;
                        
                        %Filter out detections with bad score
                        preBB = preBB(preBB(:, 5) > detThr, :);
                        
                        %convert back to image coordinates
                        if rois(i,1)<0
                            preBB(:,1) = preBB(:,1);
                        else
                            preBB(:,1) = preBB(:,1) + rois(i,1);
                        end
                        
                        if rois(i,2)<0
                            preBB(:,2) = preBB(:,2);
                        else
                            preBB(:,2) = preBB(:,2) + rois(i,2);
                        end
                        
                        detection_bboxes=preBB;
                        detection_centroids=[detection_bboxes(:,1)+detection_bboxes(:,3)*0.5 detection_bboxes(:,2)+detection_bboxes(:,4)*0.5];
                        
                        


                        
                        %% tracking
                        BVTHistograms=[];

                        %predict
                        tracks(track_index)=predict(tracks(track_index));
                        
                        %associate
                        [assignments, unassignedTracks, unassignedDetections ] = associateData( tracks(track_index), BVTHistograms, detection_centroids );
                       

                        %update
                        tracks(track_index)=updateAssignedTracks(tracks(track_index),assignments,detection_centroids,detection_bboxes, BVTHistograms,min_width,min_height);
                        tracks(track_index)=updateUnassignedTracks(tracks(track_index),unassignedTracks);
                        
%                         [tracks,nextId]=createNewTracks(tracks,...
%                             unassignedDetections,...
%                             detection_centroids,...
%                             detection_bboxes,...
%                             BVTHistograms,...
%                             min_width,...
%                             min_height, ...
%                             nextId,...
%                             state_transition_model,...
%                             state_measurement_model,...
%                             state_init_state_covariance,...
%                             state_process_noise,...
%                             state_measurement_noise);
                        
                        tracking_times=[tracking_times nan];
                        
                        
                    end
                    %delete lost tracks
%                     tracks=deleteLostTracks(tracks,...
%                         invisibleForTooLong,...
%                         frame_size);
                    
                    detection_times=[detection_times time_];
                    
                else
                    
                    %% full window detector when we have no trackers
                    
                    disp('full window')
                    tic 
                    preBB = acfDetect(frame, detector);
                    toc
                    %Filter out detections with bad score
                    detection_bboxes = preBB(preBB(:, 5) > detThr, :);
                    detection_times=[detection_times nan];
                    detection_centroids=[detection_bboxes(:,1)+detection_bboxes(:,3)*0.5 detection_bboxes(:,2)+detection_bboxes(:,4)*0.5];
                    
                    
                    
                    
                    
                    %% Extract Dario's color features
                    
                    %Each line is a color histogram of each person. A BVT Histogram has 440
                    %entries.
                    clear BVTHistograms;
                    BVTHistograms = zeros(size(detection_bboxes, 1), 440);
                    
                    i=1;
                    while i<=size(BVTHistograms,1)
                        
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
                        
                        if length(person)==0
                            BVTHistograms(i,:)=[];
                            detection_centroids(i,:)=[];
                            continue;
                        end
                        
                        [paddedImage, smallPaddedImage] = smartPadImageToBodyPartMaskSize(person);
                        
                        BVTHistograms(i,:) = extractBVT(smallPaddedImage,DefaultMask);
                        i=i+1;
                    end
                    

                    %% tracking
                    
                    %predict
                    tracks=predict(tracks);
                    
                    %associate
                    [assignments, unassignedTracks, unassignedDetections ] = associateData( tracks, BVTHistograms, detection_centroids );
                    
                    %update
                    tracks=updateAssignedTracks(tracks,assignments,detection_centroids,detection_bboxes, BVTHistograms,min_width,min_height);
                    tracks=updateUnassignedTracks(tracks,unassignedTracks);
                    tracks=deleteLostTracks(tracks,...
                        invisibleForTooLong,...
                        frame_size);
                    [tracks,nextId]=createNewTracks(tracks,...
                        unassignedDetections,...
                        detection_centroids,...
                        detection_bboxes,...
                        BVTHistograms,...
                        min_width,...
                        min_height, ...
                        nextId,...
                        state_transition_model,...
                        state_measurement_model,...
                        state_init_state_covariance,...
                        state_process_noise,...
                        state_measurement_noise);
                    
                    tracking_times=[tracking_times nan];
                    
                    
                    
                end
                
                
                
                
                
                
                %% display results
                %displayTrackingResults(obj,frame,tracks,detection_bboxes,rois);
                
                %attending regions
%                 subplot(1,3,1)
%                 imshow(frame,'InitialMagnification','fit');
%                 hold on;
%                 set(gca,'Position',[0. 0 0.33 1])
%                 
%                 for i=1:size(rois,1)
%                     rectangle('Position',...
%                         rois(i,1:4),...
%                         'EdgeColor',...
%                         [0 0 1],...
%                         'LineWidth',...
%                         3);
%                     %text(detection_bboxes(i, 1), detection_bboxes(i, 2), ['id=' int2str(detection_bboxes(i, 5))], 'FontSize', 20);
%                 end
%                 drawnow
%                 hold off
%                 
%                 % detections
%                 subplot(1,3,2)
%                 imshow(frame,'InitialMagnification','fit');
%                 hold on;
%                 set(gca,'Position',[0.33 0 0.33 1]);
%                 for i=1:size(detection_bboxes,1)
%                     rectangle('Position',...
%                         detection_bboxes(i,1:4),...
%                         'EdgeColor',...
%                         'r',...
%                         'LineWidth',...
%                         1);
%                     %text(detection_bboxes(i, 1), detection_bboxes(i, 2), ['id=' int2str(detection_bboxes(i, 5))], 'FontSize', 20);
%                 end
%                 drawnow
%                 hold off
%                 
%                 % tracks
%                 subplot(1,3,3)
%                 imshow(frame,'InitialMagnification','fit');
%                 hold on;
%                 set(gca,'Position',[0.66 0 0.33 1])
%                 uncertainties=zeros(size(tracks,2),1);
%                 for i=1:size(tracks,2)
%                     uncertainties(i)=det(tracks(i).stateKalmanFilter.StateCovariance(1:3,1:3));
%                 end
%                 uncertainties=uncertainties/sum(uncertainties);
%                 for i=1:size(tracks,2)
%                     sc = tracks(i).stateKalmanFilter.State(3);
%                     center = tracks(i).stateKalmanFilter.State(1:2);
%                     
%                     width = min_width*sc;
%                     height = min_height*sc;
%                     
%                     rectangle('Position', [...
%                         center(1)-width/2,...
%                         center(2)-height/2,...
%                         width,...
%                         height],...
%                         'EdgeColor',uncertainties(i)*[0 1 0], 'LineWidth', 3);
%                     text(...
%                         center(1)-width/2,...
%                         center(2)-height/2-20,...
%                         ['unc=' num2str(uncertainties(i))],...
%                         'FontSize', 10, 'Color', [1 1 1]);
%                 end
%                 drawnow
%                 hold off
                
                
                
                %% store results
                for i=1:length(tracks)
                    conf=1;
                    preResults = [frame_number, tracks(i).id, tracks(i).bbox(1), tracks(i).bbox(2), tracks(i).bbox(3), tracks(i).bbox(4), -1, -1, -1, -1];
                    results=[results; preResults];
                end
                
                str_disp = sprintf(' processing frame %d of %d',frame_number,n_files);
                disp(str_disp);
            end
            
            %% The results must be wriiten in the MoTChallenge res/data/[datasetname.txt] for evaluation
            csvwrite('data/res/TUD-Stadtmitte.txt', results);
            
            %% The evaluation script has 3 arguments:
            %   1 - A txt that is a list of the datasets to be evaluated (file in the
            %  seqmaps folder).
            %   2 - A directory containing the results
            %   3 - The benchmark directory
            allMets = evaluateTracking('all.txt', 'data/res/', 'data/train/');
            %MOTA = allMets.bmark2d(12);
            average_mot_random(c1,c2,exp,:)=allMets.bmark2d;
            average_optimization_times_random(c1,c2,exp,:)=mean(optimization_times);
            average_detection_times_random(c1,c2,exp,:)=nanmean(detection_times);
            average_tracking_times_random(c1,c2,exp,:)=mean(tracking_times);
        end
        %end runs
    end
    %end c2 param
end
%end c1 param
mean_average_optimization_times=mean(average_optimization_times_random,3);
mean_average_detection_times=mean(average_detection_times_random,3);
mean_average_tracking_times=mean(average_tracking_times_random,3);
mean_average_total_times=mean_average_optimization_times+mean_average_detection_times+mean_average_tracking_times;
mean_average_mot=mean(average_mot_random,3);

save('random_0_5_0_5.mat',...
    'average_optimization_times_random',...
    'average_detection_times_random',...
    'average_tracking_times_random',...
    'average_mot_random',...
    'num_exp',...
    'max_items_',...
    'capacity_constraints_',...
    'max_simulation_time_millis',...
    'simulation_depth',...
    'alpha_c',...
    'alpha_s',...
    'overlap_ratio');

% figure(1)
% bar(mean_average_total_times)
% xlabel('k_{max}')
% ylabel('runtime (s)')