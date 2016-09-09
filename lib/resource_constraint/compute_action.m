function [rois,time_elapsed]=compute_action(tracks,mcts)
rois=[];
time_elapsed=0.0;

state_means=zeros(length(tracks),3);
state_covariances=zeros(length(tracks),3,3);
if ~isempty(tracks)
for i=1:length(tracks)
    state_means(i,:)=tracks(i).stateKalmanFilter.State';
    state_covariances(i,:,:)=tracks(i).stateKalmanFilter.StateCovariance;
end
%[rois,time_elapsed]=compute_probabilities(mcts,state_means',state_covariances');
rois=reshape(cell2mat(rois)',4,size(rois,2))';
end
end
