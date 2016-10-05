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

% Load the provided detections from a dataset.
%detections = csvread('data/train/MOT16-02/det/det.txt');
%detections = csvread('data/train/MOT16-02/gt/gt.txt');

% x = [u v s u_v v_v s_v]
%% tracking parameters

%create/destroy parameters
invisibleForTooLong = 5;

%Dummy value while
detThr = 30;

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
    10 0 0 10 0 0;...
    0 10 0 0 10 0;...
    0 0  2 0  0 2;...
    0 0  0 10 0 0;...
    0 0  0 0 10 0;...
    0 0  0 0 0 10];

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
max_items_=[4 3 2 1];             % max regions to be process
capacity_constraints_=[1.0 0.75 0.5 0.25]; % percentage of image to be process at each time instant

max_simulation_time_millis=50;
simulation_depth=3;
alpha_c=0.1;
alpha_s=0.1;
overlap_ratio=0.7;

min_width=52;
min_height=128;

%v = VideoReader('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(imread([image_dir image_files(1).name]));
%% Initialize pedestrian detector
%detector=initializeDetector();
%% Initialize trackers
tracks = initializeTracks(min_width,min_height); % Create an empty array of tracks.
nextId = 1; % ID of the next track

%% Detect moving objects, and track them across video frames.
num_exp=10;
average_detection_times=zeros(length(max_items_),length(capacity_constraints_),num_exp,1);
average_optimization_times=zeros(length(max_items_),length(capacity_constraints_),num_exp,1);
average_tracking_times=zeros(length(max_items_),length(capacity_constraints_),num_exp,1);
average_mot=zeros(length(max_items_),length(capacity_constraints_),num_exp,14);
iteration_=0;
total_iterations=length(max_items_)*length(capacity_constraints_)*num_exp*n_files;

figure(1)
for c1=1:length(max_items_)
    max_items=max_items_(c1);
    for c2=1:length(capacity_constraints_)
        capacity_constraint=capacity_constraints_(c2);
        for exp=1:num_exp
            detection_times=[];
            optimization_times=[];
            tracking_times=[];
            results=[];
            for frame_number=1:n_files
                iteration_=iteration_+1;
                
                str_disp = sprintf('processing iteration %d of %d (%f)',iteration_,total_iterations, (iteration_/total_iterations) * 100.0);
                disp(str_disp);
                
                %Load image
                frame = imread([image_dir image_files(frame_number).name]);
                
                %% dynamic resource allocation (POMDP - input current belief; output actions)
                if size(tracks,2)>0
                    
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
                    figure(1)
                    if size(explored_nodes>0)
                        subplot(1,4,1)
                        hold on
                        set(gca,'Position',[0 0 0.25 1])
                        treeplot(explored_nodes);
                        hold off
                    end
                    
                    optimization_times=[optimization_times optimization_time];
                    rois=compute_rois(tracks,action,min_width,min_height,alpha_c,alpha_s);
                    %rois=[];
                    clear detection_bboxes;
                    BB=[];
                else
                    rois=[];
                    action=0;
                end
                %merge overlapping rois
                if size(rois,1)>0
                    i=1;
                    while i<=size(rois,1)
                        if rois(i,3)<0 || rois(i,4)<0
                            rois(i,:)=[];
                            continue
                        end
                        
                        %                         %merge overlapping bbxs
                        %                         j=i+1;
                        %                         while j<=size(rois,1)
                        %                             if rois(j,3)<0 || rois(j,4)<0
                        %                                 rois(j,:)=[];
                        %                                 continue
                        %                             end
                        %
                        %                             %merge if bbs they overlap more than overlap_ration
                        %                             if bboxOverlapRatio(rois(i,:),rois(j,:)) >overlap_ratio
                        %                                 upper_x=min(rois(i,1),rois(j,1));
                        %                                 upper_y=min(rois(i,2),rois(j,2));
                        %                                 down_x=max(rois(i,1)+rois(i,3),rois(j,1)+rois(j,3));
                        %                                 down_y=max(rois(i,2)+rois(i,4),rois(j,2)+rois(j,4));
                        %                                 new_width=down_x-upper_x;
                        %                                 new_height=down_y-upper_y;
                        %
                        %                                 rois(i,:)=[upper_x upper_y new_width new_height];
                        %                                 rois(j,:)=[];
                        %                                 continue;
                        %                             end
                        %                             j=j+1;
                        %                         end
                        
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
                        
                        rois(i,:)=[x_after y_after width_after height_after];
                        
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
                    tic
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
                    
                    detection_times=[detection_times toc];
                end
                
                % full window detector
                if size(tracks,2)<1
                    disp('full window')
                    
                    tic
                    %preBB = detections(detections(:,1) == frame_number,:);
                    
                    preBB = acfDetect(frame, detector);
                    
                    %Filter out detections with bad score
                    BB = preBB(preBB(:, 5) > detThr, :);
                    detection_times=[detection_times toc];
                    if toc < 0.001
                        disp('wtf');
                    end
                end
                
                if length(BB)==0
                    preBB = acfDetect(frame, detector);
                    
                    %Filter out detections with bad score
                    BB = preBB(preBB(:, 5) > detThr, :);
                end
                detection_bboxes=[BB(:,1) BB(:,2) BB(:,3) BB(:,4)];
                detection_centroids=[BB(:,1)+BB(:,3)*0.5 BB(:,2)+BB(:,4)*0.5];
                
                
                
                
                
%                 %% display results
%                 %displayTrackingResults(obj,frame,tracks,detection_bboxes,rois);
%                 
%                 % attending regions
%                 subplot(1,4,2)
%                 imshow(frame,'InitialMagnification','fit');
%                 hold on;
%                 set(gca,'Position',[0.25 0 0.25 1])
%                 
%                 for i=1:size(rois,1)
%                     rectangle('Position',...
%                         rois(i,:),...
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
%                 subplot(1,4,3)
%                 imshow(frame,'InitialMagnification','fit');
%                 hold on;
%                 set(gca,'Position',[0.5 0 0.25 1]);
%                 for i=1:size(detection_bboxes,1)
%                     rectangle('Position',...
%                         detection_bboxes(i,1:4),...
%                         'EdgeColor',...
%                         'r',...
%                         'LineWidth',...
%                         3);
%                     %text(detection_bboxes(i, 1), detection_bboxes(i, 2), ['id=' int2str(detection_bboxes(i, 5))], 'FontSize', 20);
%                 end
%                 drawnow
%                 hold off
%                 
%                 % tracks
%                 subplot(1,4,4)
%                 imshow(frame,'InitialMagnification','fit');
%                 hold on;
%                 set(gca,'Position',[0.75 0 0.25 1])
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
                
                %% Extract Dario's color features
                tic
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
                
                tracking_times=[tracking_times toc];
                
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
            average_mot(c1,c2,exp,:)=allMets.bmark2d;
            average_optimization_times(c1,c2,exp,:)=mean(optimization_times);
            average_detection_times(c1,c2,exp,:)=mean(detection_times);
            average_tracking_times(c1,c2,exp,:)=mean(tracking_times);
        end
        %end runs
    end
    %end c2 param
end
%end c1 param
mean_average_optimization_times=mean(average_optimization_times,3);
mean_average_detection_times=mean(average_detection_times,3);
mean_average_tracking_times=mean(average_tracking_times,3);
mean_average_total_times=mean_average_optimization_times+mean_average_detection_times+mean_average_tracking_times;
mean_average_mot=mean(average_mot,3);

save('teste.mat',...
    'average_optimization_times',...
    'average_detection_times',...
    'average_tracking_times',...
    'average_mot',...
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