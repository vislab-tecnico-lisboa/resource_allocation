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
    0 0  2 0  0 0;...
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

max_simulation_time_millis=50;
simulation_depth=1;

min_width=52;
min_height=128;

%v = VideoReader('resource_allocation/dataset/cvpr10_tud_stadtmitte/cvpr10_tud_stadtmitte.avi');
frame_size = size(imread([image_dir image_files(1).name]));

%% Initialize pedestrian detector
%detector=initializeDetector();

%% Initialize trackers

%% Detect moving objects, and track them across video frames.
num_exp=100;
average_detection_times_fullwindow=zeros(num_exp,1);
average_optimization_times_fullwindow=zeros(num_exp,1);
average_tracking_times_fullwindow=zeros(num_exp,1);
average_mot_fullwindow=zeros(num_exp,14);
iteration_=0;
total_iterations=num_exp*n_files;

figure(1)

for exp=1:num_exp
    detection_times=[];
    optimization_times=[];
    tracking_times=[];
    results=[];
    tracks = initializeTracks(min_width,min_height); % Create an empty array of tracks.
    nextId = 1; % ID of the next track

    for frame_number=1:n_files
        
        iteration_=iteration_+1;
        
        str_disp = sprintf('processing iteration %d of %d (%f)',iteration_,total_iterations, (iteration_/total_iterations) * 100.0);
        disp(str_disp);
        
        %Load image
        frame = imread([image_dir image_files(frame_number).name]);
        
        disp('full window')
        tic
        preBB = acfDetect(frame, detector);
        
        %Filter out detections with bad score
        BB = preBB(preBB(:, 5) > detThr, :);
        detection_times=[detection_times toc];
        
        detection_bboxes=BB;
        detection_centroids=[BB(:,1)+BB(:,3)*0.5 BB(:,2)+BB(:,4)*0.5];
        
        
        %% Extract Dario's color features
        tic
        %Each line is a color histogram of each person. A BVT Histogram has 440
        %entries.
        BVTHistograms=[];
%         clear BVTHistograms;
%         BVTHistograms = zeros(size(detection_bboxes, 1), 440);
%         
%         i=1;
%         while i<=size(BVTHistograms,1)
%             
%             %If the bbox is out of the image would have an error... this avoids
%             %that but might not be the best solution
%             
%             beginX = detection_bboxes(i,1);
%             endX = detection_bboxes(i,1)+detection_bboxes(i,3);
%             beginY = detection_bboxes(i,2);
%             endY = detection_bboxes(i,2)+detection_bboxes(i,4);
%             
%             
%             if endX > size(frame, 2)
%                 endX = size(frame, 2);
%             end
%             
%             if beginX <1;
%                 beginX = 1;
%             end
%             if endY > size(frame, 1)
%                 endY = size(frame, 1);
%             end
%             if beginY < 1;
%                 beginY = 1;
%             end
%             
%             beginY = round(beginY);
%             endY = round(endY);
%             beginX = round(beginX);
%             endX = round(endX);
%             
%             person = frame(beginY:endY,beginX:endX,:);
%             
%             if length(person)==0
%                 BVTHistograms(i,:)=[];
%                 detection_centroids(i,:)=[];
%                 continue;
%             end
%             
%             [paddedImage, smallPaddedImage] = smartPadImageToBodyPartMaskSize(person);
%             
%             BVTHistograms(i,:) = extractBVT(smallPaddedImage,DefaultMask);
%             i=i+1;
%         end
        
       % Update attended tracks
       for i=1:size(tracks,2)
       tracks(i).attended=1;
       end;
        
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
        
        if frame_number==1
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
        end
        size(tracks)
        tracking_times=[tracking_times toc];

        % attending regions
%         figure(1)
%   
%         % detections
%         subplot(1,2,1)
%         imshow(frame,'InitialMagnification','fit');
%         hold on;
%         set(gca,'Position',[0.0 0 0.5 1]);
%         for i=1:size(detection_bboxes,1)
%             rectangle('Position',...
%                 detection_bboxes(i,1:4),...
%                 'EdgeColor',...
%                 'r',...
%                 'LineWidth',...
%                 1);
%             %text(detection_bboxes(i, 1), detection_bboxes(i, 2), ['id=' int2str(detection_bboxes(i, 5))], 'FontSize', 20);
%         end
%         drawnow
%         hold off
%         
%         % tracks
%         subplot(1,2,2)
%         imshow(frame,'InitialMagnification','fit');
%         hold on;
%         set(gca,'Position',[0.5 0 0.5 1])
%         uncertainties=zeros(size(tracks,2),1);
%         for i=1:size(tracks,2)
%             uncertainties(i)=det(tracks(i).stateKalmanFilter.StateCovariance(1:3,1:3));
%         end
%         uncertainties=uncertainties/sum(uncertainties);
%         for i=1:size(tracks,2)
%             sc = tracks(i).stateKalmanFilter.State(3);
%             center = tracks(i).stateKalmanFilter.State(1:2);
%             
%             width = min_width*sc;
%             height = min_height*sc;
%             
%             rectangle('Position', [...
%                 center(1)-width/2,...
%                 center(2)-height/2,...
%                 width,...
%                 height],...
%                 'EdgeColor',uncertainties(i)*[0 1 0], 'LineWidth', 3);
%             text(...
%                 center(1)-width/2,...
%                 center(2)-height/2-20,...
%                 ['unc=' num2str(uncertainties(i))],...
%                 'FontSize', 10, 'Color', [1 1 1]);
%         end
%         drawnow
%         hold off
        
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
    average_mot_fullwindow(exp,:)=allMets.bmark2d;
    average_optimization_times_fullwindow(exp,:)=mean(optimization_times);
    average_detection_times_fullwindow(exp,:)=nanmean(detection_times);
    average_tracking_times_fullwindow(exp,:)=mean(tracking_times);
end
%end runs

mean_average_optimization_times=mean(average_optimization_times_fullwindow,3);
mean_average_detection_times=mean(average_detection_times_fullwindow,3);
mean_average_tracking_times=mean(average_tracking_times_fullwindow,3);
mean_average_total_times=mean_average_optimization_times+mean_average_detection_times+mean_average_tracking_times;
mean_average_mot=mean(average_mot_fullwindow,3);

save('fullwindow.mat',...
    'average_optimization_times_fullwindow',...
    'average_detection_times_fullwindow',...
    'average_tracking_times_fullwindow',...
    'average_mot_fullwindow');

% figure(1)
% bar(mean_average_total_times)
% xlabel('k_{max}')
% ylabel('runtime (s)')