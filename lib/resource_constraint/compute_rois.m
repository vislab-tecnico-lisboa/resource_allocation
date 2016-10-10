function [rois]=compute_rois(tracks,action,min_width,min_height,alpha_c,alpha_s)
rois=[];
time_elapsed=0.0;

state_means=zeros(length(action),6);
%state_covariances=zeros(length(tracks),6,6);
state_covariances=cell(length(action),1);
if ~isempty(tracks)
    
for i=1:length(action)
    state_means(i,:)=tracks(action(i)).stateKalmanFilter.State';
    state_covariances{i}=tracks(action(i)).stateKalmanFilter.StateCovariance;
    center_x=state_means(i,1);
    center_y=state_means(i,2);
    width= (state_means(i,3) + alpha_s*sqrt(tracks(i).stateKalmanFilter.StateCovariance(3,3)))*min_width  + alpha_c*sqrt(tracks(i).stateKalmanFilter.StateCovariance(1,1));
    height=(state_means(i,3) + alpha_s*sqrt(tracks(i).stateKalmanFilter.StateCovariance(3,3)))*min_height + alpha_c*sqrt(tracks(i).stateKalmanFilter.StateCovariance(2,2));
    up_x=center_x-width*0.5;
    up_y=center_y-height*0.5;
        
    roi=[up_x up_y width height double(action(i))];
    rois=[rois; roi];
end


% state_means=zeros(length(tracks),6);
% %state_covariances=zeros(length(tracks),6,6);
% state_covariances=cell(length(tracks),1);
% if ~isempty(tracks)
%     
% for i=1:length(tracks)
%     state_means(i,:)=tracks(i).stateKalmanFilter.State';
%     state_covariances{i}=tracks(i).stateKalmanFilter.StateCovariance;
%     center_x=state_means(i,1);
%     center_y=state_means(i,2);
%     width=state_means(i,3)*min_width + alpha_c*sqrt(tracks(i).stateKalmanFilter.StateCovariance(1,1)) + alpha_s*sqrt(tracks(i).stateKalmanFilter.StateCovariance(3,3));
%     height=state_means(i,3)*min_height + alpha_c*sqrt(tracks(i).stateKalmanFilter.StateCovariance(1,1)) + alpha_s*sqrt(tracks(i).stateKalmanFilter.StateCovariance(3,3));
%     up_x=center_x-width*0.5;
%     up_y=center_y-height*0.5;
% 
%     roi=[up_x up_y width height];
%     rois=[rois; roi];
% end



end
end
